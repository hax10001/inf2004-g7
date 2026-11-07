/** @file mqtt_utk.c
 * @brief Network-owner MQTT transport with bounded TX and RX buffers.
 */
#include "modules/telemetry/buddy1_mqtt.h"
#include "mqtt_utk.h"
#include <string.h>

#define B1_QUEUE_SIZE 8u
#if defined(__GNUC__)
#define B1_BARRIER() __sync_synchronize()
#else
#define B1_BARRIER() do { } while(0)
#endif

typedef struct { uint8_t bytes[B1_MAX_FRAME], len, topic_ack; } Queued;
static Queued txq[B1_QUEUE_SIZE],rxq[B1_QUEUE_SIZE];
/* ACK is priority and may burst while historical telemetry drains. */
#define B1_ACK_QUEUE_SIZE 8u
static Queued ackq[B1_ACK_QUEUE_SIZE],stopq[2];
static volatile uint8_t tx_head,tx_tail,rx_head,rx_tail;
static volatile uint8_t ack_head,ack_tail,stop_head,stop_tail;
static uint32_t tx_drops,rx_drops;

static int offer(Queued *q,volatile uint8_t *head,volatile uint8_t *tail,
                 const uint8_t *bytes,uint8_t n,uint8_t topic_ack,
            uint8_t capacity) {
    uint8_t next=(uint8_t)((*head+1u)%capacity);
    if(next==*tail)return -1;
    q[*head].len=n;q[*head].topic_ack=topic_ack;
    memcpy(q[*head].bytes,bytes,n);
    B1_BARRIER();*head=next;return 0;
}
static int take(Queued *q,volatile uint8_t *head,volatile uint8_t *tail,
    Queued *out,uint8_t capacity) {
    if(*head==*tail)return 0;
    B1_BARRIER();*out=q[*tail];B1_BARRIER();
    *tail=(uint8_t)((*tail+1u)%capacity);return 1;
}
int b1_mqtt_submit(const B1_Frame *f) {
    uint8_t b[B1_MAX_FRAME];size_t n=b1_frame_encode(f,b,sizeof b);
    if(n==0u)return -1;
    int is_ack=f->type==B1_TYPE_ACK;
    if((is_ack?offer(ackq,&ack_head,&ack_tail,b,(uint8_t)n,1u,
                            B1_ACK_QUEUE_SIZE):
               offer(txq,&tx_head,&tx_tail,b,(uint8_t)n,0u,B1_QUEUE_SIZE))!=0) {
        tx_drops++;return -1;
    }
    return 0;
}
int b1_mqtt_receive_command(B1_Frame *f) {
    Queued q;
    if(!take(stopq,&stop_head,&stop_tail,&q,2u) &&
       !take(rxq,&rx_head,&rx_tail,&q,B1_QUEUE_SIZE))return 0;
    return b1_frame_decode(q.bytes,q.len,f)==0 ? 1 : -1;
}
uint32_t b1_mqtt_queue_drops(void) {return tx_drops+rx_drops;}
static volatile uint8_t mqtt_link_ready;
uint8_t b1_mqtt_connected(void) {
    return __atomic_load_n(&mqtt_link_ready, __ATOMIC_ACQUIRE);
}
static void publish_link_status(uint8_t ready) {
    __atomic_store_n(&mqtt_link_ready, ready, __ATOMIC_RELEASE);
}

/* This translation unit is the network owner's sole MQTT/lwIP interface.
 * Called only from the CYW43 service after lwIP is initialised. Application
 * tasks only offer/take bounded copies; callbacks never run application
 * subsystem logic and do not block. The queues have a single producer and
 * consumer each; do not add more writers without explicit synchronisation. */
#if defined(B1_ENABLE_MQTT) && TM_WIFI_NETIF
#include "lwip/tcp.h"
#include "lwip/ip4_addr.h"
#include "lwip/netif.h"
#include "lwip/pbuf.h"
#include "lwip/sys.h"
#if defined(__has_include)
#if __has_include("wifi_secrets.h")
#include "wifi_secrets.h"
#elif __has_include("buddy1_broker_local.h")
#include "modules/telemetry/buddy1_broker_local.h"
#endif
#endif
#ifndef B1_BROKER_IPV4
#error "Provide a private buddy1_broker_local.h configuration"
#endif
#ifndef B1_BROKER_PORT
#define B1_BROKER_PORT 1883
#endif
#ifndef B1_MQTT_USER
#define B1_MQTT_USER ""
#endif
#ifndef B1_MQTT_PASSWORD
#define B1_MQTT_PASSWORD ""
#endif
#ifndef B1_MQTT_TOPIC_ROOT
#define B1_MQTT_TOPIC_ROOT "g7/b1/robot"
#endif
#ifndef B1_MQTT_CLIENT_ID
#define B1_MQTT_CLIENT_ID "g7-b1-car01"
#endif
#define B1_TOPIC_TELEM B1_MQTT_TOPIC_ROOT "/telemetry"
#define B1_TOPIC_COMMAND B1_MQTT_TOPIC_ROOT "/command"
#define B1_TOPIC_ACK B1_MQTT_TOPIC_ROOT "/ack"
#define B1_TOPIC_STATUS B1_MQTT_TOPIC_ROOT "/availability"
/* Plain-text copy of the serial console (USB-console builds only). */
#define B1_TOPIC_LOG B1_MQTT_TOPIC_ROOT "/log"
/* Log chunks: whole lines where possible, at most this many bytes. */
#define B1_LOG_CHUNK 240u
/* Send a partial chunk after this long, so a quiet log still arrives. */
#define B1_LOG_FLUSH_MS 100u
/* Average and burst log rate. Normal runs print well under 1 KB/s; this
 * only stops a dump (e.g. raw barcode samples) from flooding the link. */
#define B1_LOG_BYTES_PER_SEC 8000u
#define B1_LOG_BURST 2048u
/* TCP send buffer kept free for ACKs and telemetry: logs never fill it. */
#define B1_LOG_TCP_RESERVE 1024u

#define B1_TXBUF_CAP 320u
#define B1_RXBUF_CAP 256u
#define B1_KEEPALIVE_MS 30000u
static struct tcp_pcb *pcb;
static uint8_t connected,subscribed,tcp_ready,awaiting_ping;
static uint8_t rx[B1_RXBUF_CAP];static uint16_t rxlen;
static uint32_t next_connect_ms,last_tx_ms,last_rx_ms,last_ping_ms;
static uint16_t packet_id=1u;
static uint8_t backoff=1u;

static size_t mqtt_varlen(uint8_t *out,size_t n) {
    size_t pos=0;
    do {uint8_t v=(uint8_t)(n%128u);n/=128u;if(n)v|=0x80u;out[pos++]=v;}
        while(n!=0u&&pos<4u);
    return pos;
}
static size_t mqtt_string(uint8_t *out,const char *str) {
    size_t n=strlen(str);
    if(n>65535u)return 0;
    out[0]=(uint8_t)(n>>8);out[1]=(uint8_t)n;
    memcpy(out+2,str,n);return n+2u;
}
static int mqtt_write(uint8_t type,const uint8_t *data,size_t n) {
    uint8_t packet[B1_TXBUF_CAP],vl[4];size_t offset=0,vn;
    err_t result;
    if(pcb==NULL||!tcp_ready||n>300u)return -1;
    vn=mqtt_varlen(vl,n);
    packet[offset++]=type;memcpy(packet+offset,vl,vn);offset+=vn;
    if(offset+n>sizeof packet)return -1;
    memcpy(packet+offset,data,n);offset+=n;
    result=tcp_write(pcb,packet,(u16_t)offset,TCP_WRITE_FLAG_COPY);
    if(result!=ERR_OK)return -1;
    result=tcp_output(pcb);
    if(result!=ERR_OK)return -1;
    last_tx_ms=sys_now();return 0;
}
static void discard(void) {
    if(pcb!=NULL){struct tcp_pcb *p=pcb;pcb=NULL;
        tcp_arg(p,NULL);tcp_recv(p,NULL);tcp_err(p,NULL);tcp_sent(p,NULL);
        if(tcp_close(p)!=ERR_OK)tcp_abort(p);
    }
    publish_link_status(0u);
    tcp_ready=0;connected=0;subscribed=0;awaiting_ping=0;rxlen=0;
    rx_tail=rx_head;stop_tail=stop_head;
        /* Never replay queued START on reconnect. */
    next_connect_ms=sys_now()+(uint32_t)backoff*1000u;
    if(backoff<16u)backoff=(uint8_t)(backoff*2u);
}
static void tcp_error(void *arg,err_t err) {
    (void)arg;(void)err;publish_link_status(0u);
    pcb=NULL;tcp_ready=0;connected=0;
    subscribed=0;awaiting_ping=0;rxlen=0;
    rx_tail=rx_head;stop_tail=stop_head;
    next_connect_ms=sys_now()+(uint32_t)backoff*1000u;
    if(backoff<16u)backoff=(uint8_t)(backoff*2u);
}
static int connect_mqtt(void) {
    uint8_t p[256];size_t n=0;
        const char *user=B1_MQTT_USER,*pass=B1_MQTT_PASSWORD;
    uint8_t flags=0x26u; /* clean session + retained Last Will */
    if(user[0])flags|=0x80u;if(pass[0])flags|=0x40u;
    n+=mqtt_string(p+n,"MQTT");p[n++]=4u;p[n++]=flags;p[n++]=0u;p[n++]=30u;
    if(strlen(user)>32u || strlen(pass)>64u ||
       strlen(B1_MQTT_CLIENT_ID)>32u || strlen(B1_TOPIC_STATUS)>60u)return -1;
    n+=mqtt_string(p+n,B1_MQTT_CLIENT_ID);
    n+=mqtt_string(p+n,B1_TOPIC_STATUS);
    n+=mqtt_string(p+n,"offline");
    if(user[0])n+=mqtt_string(p+n,user);
    if(pass[0])n+=mqtt_string(p+n,pass);
    return mqtt_write(0x10u,p,n);
}
static int mqtt_publish(const Queued *q) {
    uint8_t p[145];size_t n=0;
    const char *topic=q->topic_ack?B1_TOPIC_ACK:B1_TOPIC_TELEM;
    n+=mqtt_string(p+n,topic);
    if(n+q->len>sizeof p)return -1;
    memcpy(p+n,q->bytes,q->len);n+=q->len;
    return mqtt_write(0x30u,p,n);
}
/* Console copy kept by the USB console (lib/libtm); absent in UART-console
 * builds, where the log topic stays silent. */
extern int tm_netlog_take(unsigned char *out,int max,unsigned int *p_dropped)
    __attribute__((weak));
extern int tm_netlog_pending(void) __attribute__((weak));
static uint8_t log_buf[B1_LOG_CHUNK];static uint16_t log_len;
static uint32_t log_last_ms,log_tokens=B1_LOG_BURST,log_token_ms;
static uint32_t log_lost;
/* Lowest priority: called only when no ACK is waiting. Sends whole lines,
 * keeping a partial line for the next chunk unless it fills the chunk or
 * has waited B1_LOG_FLUSH_MS. Bytes lost (ring full, or a refused write)
 * are reported in-band as one "[netlog] lost" line. */
static void log_poll(uint32_t now) {
    uint8_t p[B1_LOG_CHUNK+40u];size_t n=0,send,i;unsigned int dropped=0u;
    char note[48];int note_len=0;
    if(tm_netlog_take==NULL||tm_netlog_pending==NULL)return;
    log_tokens+=(now-log_token_ms)*B1_LOG_BYTES_PER_SEC/1000u;
    log_token_ms=now;
    if(log_tokens>B1_LOG_BURST)log_tokens=B1_LOG_BURST;
    if(log_len<B1_LOG_CHUNK){
        log_len=(uint16_t)(log_len+tm_netlog_take(log_buf+log_len,
            (int)(B1_LOG_CHUNK-log_len),&dropped));
        log_lost+=dropped;
    }
    if(log_len==0u&&log_lost==0u){log_last_ms=now;return;}
    if(log_len<B1_LOG_CHUNK&&now-log_last_ms<B1_LOG_FLUSH_MS)return;
    if(log_tokens<log_len||pcb==NULL||
       tcp_sndbuf(pcb)<B1_LOG_CHUNK+40u+B1_LOG_TCP_RESERVE||
       tcp_sndqueuelen(pcb)>=TCP_SND_QUEUELEN/2u)return;
    /* Up to the last newline; a chunk with none goes out whole. */
    send=log_len;
    for(i=log_len;i>0u;i--)if(log_buf[i-1u]=='\n'){send=i;break;}
    if(log_lost!=0u){
        unsigned int v=(unsigned int)log_lost;char d[12];int k=0;
        do{d[k++]=(char)('0'+v%10u);v/=10u;}while(v!=0u&&k<11);
        memcpy(note,"[netlog] lost ",14u);note_len=14;
        while(k>0)note[note_len++]=d[--k];
        memcpy(note+note_len," bytes\n",7u);note_len+=7;
    }
    n+=mqtt_string(p+n,B1_TOPIC_LOG);
    memcpy(p+n,note,(size_t)note_len);n+=(size_t)note_len;
    memcpy(p+n,log_buf,send);n+=send;
    if(mqtt_write(0x30u,p,n)==0){
        log_lost=0u;log_tokens-=(uint32_t)send;
    } else log_lost+=(uint32_t)send;
    memmove(log_buf,log_buf+send,log_len-send);
    log_len=(uint16_t)(log_len-send);log_last_ms=now;
}
static void process_packet(uint8_t head,const uint8_t *p,uint16_t n) {
    uint8_t type=(uint8_t)(head>>4);
    if(type==2u && n==2u) {
        if(p[1]!=0u){discard();return;}
        connected=1u;backoff=1u;
        uint8_t subscribe[100];size_t len=0;
        const char *topic=B1_TOPIC_COMMAND;
        subscribe[len++]=(uint8_t)(packet_id>>8);
            subscribe[len++]=(uint8_t)packet_id;
        len+=mqtt_string(subscribe+len,topic);subscribe[len++]=0u;
        (void)mqtt_write(0x82u,subscribe,len);
    } else if(type==9u && n>=3u) {
        if(p[2]==0u){
            /* Availability is a soft status indicator; no motor authority. */
            uint8_t b[100];size_t n=mqtt_string(b,B1_TOPIC_STATUS);
            memcpy(b+n,"online",6u);n+=6u;
            (void)mqtt_write(0x31u,b,n); /* QoS 0 retained */
            subscribed=1u;
            publish_link_status(1u);
        }
    } else if(type==13u)awaiting_ping=0u;
    else if(type==3u && n>=2u && (head&7u)==0u) { /* QoS0, non-retained only */
        uint16_t len=(uint16_t)((uint16_t)p[0]<<8|p[1]);
        const char *topic=B1_TOPIC_COMMAND;
        if((size_t)len==strlen(topic) && n>=2u+len &&
           memcmp(p+2,topic,len)==0 && n-2u-len<=B1_MAX_FRAME) {
            B1_Frame decoded;B1_Command command;
            int urgent=0;
            if(b1_frame_decode(p+2+len,n-2u-len,&decoded)==0 &&
               decoded.type==B1_TYPE_COMMAND &&
               b1_decode_command(decoded.payload,decoded.payload_len,
            &command)==0 &&
               command.command==B1_CMD_STOP)urgent=1;
            /* A saturated ordinary command ring cannot suppress STOP. */
            if(urgent) {
                if(offer(stopq,&stop_head,&stop_tail,p+2+len,
                         (uint8_t)(n-2u-len),0u,2u)!=0)rx_drops++;
            } else if(offer(rxq,&rx_head,&rx_tail,p+2+len,
                            (uint8_t)(n-2u-len),0u,B1_QUEUE_SIZE)!=0)rx_drops++;
        }
    }
}
static err_t tcp_recv_cb(void *arg,struct tcp_pcb *c,struct pbuf *p,err_t err) {
    uint16_t pos=0;
    (void)arg;(void)c;
    if(err!=ERR_OK||p==NULL){if(p)pbuf_free(p);discard();return ERR_OK;}
    tcp_recved(c,p->tot_len);last_rx_ms=sys_now();
    if((uint32_t)rxlen+p->tot_len>sizeof rx){pbuf_free(p);discard();
        return ERR_OK;}
    (void)pbuf_copy_partial(p,rx+rxlen,p->tot_len,0u);
        rxlen=(uint16_t)(rxlen+p->tot_len);pbuf_free(p);
    while(rxlen>=2u){
        uint32_t remain=0,mul=1u;uint8_t b;uint16_t body=1u;
        do {if(body>=rxlen)return ERR_OK;b=rx[body++];
            remain+=(uint32_t)(b&127u)*mul;mul*=128u;
            if(body>5u||remain>200u){discard();return ERR_OK;}
        } while((b&0x80u)!=0u);
        if(remain>rxlen-body)return ERR_OK;
        process_packet(rx[0],rx+body,(uint16_t)remain);
        pos=(uint16_t)(body+remain);
        if(pos<rxlen)memmove(rx,rx+pos,rxlen-pos);
        rxlen=(uint16_t)(rxlen-pos);
        if(pcb==NULL)return ERR_OK;
    }
    return ERR_OK;
}
static err_t tcp_connected_cb(void *arg,struct tcp_pcb *c,err_t err) {
    (void)arg;(void)c;
    if(err!=ERR_OK){discard();return ERR_OK;}
    tcp_ready=1u;last_rx_ms=sys_now();last_ping_ms=last_rx_ms;
    if(connect_mqtt()!=0)discard();return ERR_OK;
}
void mqtt_utk_poll(struct netif *netif) {
    uint32_t now=sys_now();Queued q;ip4_addr_t ip;err_t result;
    if(netif==NULL||!netif_is_up(netif)||!netif_is_link_up(netif)){
        if(pcb!=NULL)discard();return;
    }
    if(pcb==NULL){
        if((int32_t)(now-next_connect_ms)<0)return;
        if(!ip4addr_aton(B1_BROKER_IPV4,&ip)||ip4_addr_isany_val(ip))return;
        pcb=tcp_new();last_rx_ms=now;if(pcb==NULL){next_connect_ms=now+2000u;
            return;}
        tcp_bind_netif(pcb,netif);tcp_arg(pcb,NULL);
        tcp_err(pcb,tcp_error);tcp_recv(pcb,tcp_recv_cb);
        result=tcp_connect(pcb,(const ip_addr_t *)&ip,(u16_t)B1_BROKER_PORT,
            tcp_connected_cb);
        if(result!=ERR_OK)discard();return;
    }
    if(!tcp_ready){if(now-last_rx_ms>10000u)discard();return;}
    if(!connected){if(now-last_rx_ms>10000u)discard();return;}
    if(now-last_rx_ms>45000u){discard();return;}
    /* Ping every 15 s even while telemetry and logs keep sending: the
     * broker then answers with PINGRESP, which is what keeps last_rx_ms
     * fresh. Pinging only after 15 s of *no transmit* meant a busy robot
     * never pinged, heard nothing back while the dashboard was silent, and
     * dropped the link every ~45 s ("MQTT lost -> motors STOPPED"). */
    if(now-last_ping_ms>15000u){
        if(mqtt_write(0xc0u,(const uint8_t *)"",0u)==0){
            awaiting_ping=1u;last_ping_ms=now;
        }
    }
    if(subscribed && ack_head!=ack_tail) {
        q=ackq[ack_tail];
        if(mqtt_publish(&q)==0){
            B1_BARRIER();ack_tail=(uint8_t)((ack_tail+1u)%B1_ACK_QUEUE_SIZE);
        }
    } else if(subscribed && tx_head!=tx_tail){
        /* Do not advance queue tail until tcp_write accepts the packet. */
        q=txq[tx_tail];
        if(mqtt_publish(&q)==0){
            B1_BARRIER();tx_tail=(uint8_t)((tx_tail+1u)%B1_QUEUE_SIZE);
        }
        /* Telemetry is one frame per 50 ms; the 2 ms poll leaves room. */
    } else if(subscribed) log_poll(now);
}
#else
void mqtt_utk_poll(struct netif *netif) {(void)netif;}
#endif
/*** end of file ***/

