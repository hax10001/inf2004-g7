/** @file buddy1_rt.c
 * @brief Buddy 1 nonblocking RTOS telemetry adapter.
 */
/* µT-Kernel application adapter; never calls raw lwIP on this RTOS task. */
#include "buddy1_rt.h"
#if defined(B1_ROBOT_APP)
#include "buddy1.h"
#include "buddy1_mqtt.h"
#include "telemetry_peers.h"
#include "app/controller.h"
#include "app/task_controller.h"
#include "modules/barcode_scanning/bc_wave.h"
#include "modules/barcode_scanning/bc_port.h"
#include "drivers/motor.h"
#include "board/board_config.h"
#include "g7_odometry_config.h"
#include "cyw43_utk.h"
#include <tk/tkernel.h>
#include <tm/tmonitor.h>

static B1_Store store;
static G7_TelemetryPeers peers;
static uint16_t mission_seq;
static uint8_t last_mission=255u;
static ID mutex_id;
#define B1_TASK_STACK 4096
static void acquire(void *ctx){(void)ctx;(void)tk_loc_mtx(mutex_id,TMO_FEVR);}
static void release(void *ctx){(void)ctx;(void)tk_unl_mtx(mutex_id);}

/* Teammate adapters deliberately arrive through the B1_Store interfaces.
 * Do not create a fake Mission Coordinator which claims START stopped PWM. */
B1_Store *b1_runtime_store(void){return &store;}
/* One bounded mailbox, protected by existing store mutex. MQTT/telemetry
 * owns mapping; controller alone consumes events and writes motor output. */
void g7_nav_note_decoded(uint8_t letter, uint16_t sequence)
{
    /* Compatibility hook; navigation now receives directly from the worker. */
    (void)letter; (void)sequence;
}
void g7_nav_clear_decoded(void)
{
    (void)bc_port_nav_enable(0U);
    bc_wave_request_cancel();
}
int g7_nav_begin_run(void)
{
    /* Held suppression prevents a new capture here. Reset only mailbox
     * ownership; never cancel the scanner that just passed readiness. */
    if (!bc_wave_is_ready()) return 0;
    return bc_port_nav_enable(1U) >= E_OK;
}
int g7_nav_take_scan(uint8_t *letter, uint8_t *action, uint16_t *sequence,
                     uint32_t *captured_msec, uint32_t *completed_msec,
                     uint8_t *pending)
{
    scan_handoff_t scan = {0};
    int result = 0;
    if (!letter || !action || !sequence || !captured_msec ||
        !completed_msec || !pending)
        return -3;
    /* On contention do not pretend the decoder is idle. Never consume a
     * result until mapping can be read under its existing mutex. */
    *pending = 1U;
    if (mutex_id <= 0 || tk_loc_mtx(mutex_id, TMO_POL) < E_OK) return 0;
    if (bc_port_nav_take(&scan) < E_OK) {
        (void)tk_unl_mtx(mutex_id);
        return 0;
    }
    *pending = (uint8_t)(scan.busy || scan.ready);
    if (scan.ready) {
        uint8_t i;
        *letter = (uint8_t)scan.letter;
        *sequence = scan.sequence;
        *captured_msec = scan.captured_msec;
        *completed_msec = scan.completed_msec;
        result = (scan.letter < 26U) ? -1 : -2;
        if (scan.overflow) result = -3;
        if (scan.letter < 26U && store.mapping_valid) {
            for (i = 0U; i < 4U; ++i) {
                if (store.mapping[i] == *letter) {
                    *action = i;
                    result = 1;
                    break;
                }
            }
        }
    }
    (void)tk_unl_mtx(mutex_id);
    return result;
}
int g7_nav_take_decoded(uint8_t *letter, uint8_t *action, uint16_t *sequence)
{
    uint32_t captured = 0U;
    uint32_t completed = 0U;
    uint8_t pending = 0U;
    return g7_nav_take_scan(letter, action, sequence, &captured, &completed,
                            &pending);
}


static void network_application_task(INT stacd,void *exinf){
    B1_Frame f,ack;ER er;
    uint32_t now;
    uint32_t boot_ms=0u;
    uint8_t reported_network_deadline=0u;
    uint8_t previous_connected=255u;
    uint32_t last_net_log_ms=0u;
    uint16_t nav_reported_seq=0u;
    (void)stacd;(void)exinf;
    for(;;){
        /* Commands received by CYW43 owner are passed via an SPSC queue. */
        /* Mapping/START use a recently issued STATUS challenge from this boot.
         * Do not estimate device uptime from arrival times after reconnect.
         * STOP bypasses clock synchronization for emergency hold. */
        {
            extern uint64_t time_us_64(void);
            now=(uint32_t)(time_us_64()/1000ULL);
        }
        if(boot_ms==0u)boot_ms=now;
        if(!reported_network_deadline && now-boot_ms>=10000u) {
            T_CYW43_UTK_STATUS wifi={0};
            cyw43_utk_get_status(&wifi);
            reported_network_deadline=1u;
            if(!wifi.link_up) {
                tm_printf((UB *)
                    "[buddy1] WiFi not ready after 10s; "
                    "autonomous controller continues.\n");
            } else {
                tm_printf((UB *)
                    "[buddy1] WiFi link up; MQTT is optional "
                    "for autonomous operation.\n");
            }
        }
        {
            uint8_t is_connected=b1_mqtt_connected();
            if(is_connected!=previous_connected ||
               now-last_net_log_ms>=5000u) {
                T_CYW43_UTK_STATUS wifi={0};
                cyw43_utk_get_status(&wifi);
#ifndef B1_BARCODE_TEST /* quiet barcode test log */
                tm_printf((UB *)
                    "[NET] wifi=%u mqtt=%u init=%d scan=%u "
                    "join_req=%d join=%d link=%d dhcp=%u "
                    "ip=%u.%u.%u.%u retry=%u\n",
                    (UINT)wifi.link_up, (UINT)is_connected,
                    (INT)wifi.init_result, (UINT)wifi.scan_complete,
                    (INT)wifi.join_request_result,
                    (INT)wifi.join_result, (INT)wifi.link_status,
                    (UINT)wifi.dhcp_complete,
                    (UINT)wifi.dhcp_address[0],
                    (UINT)wifi.dhcp_address[1],
                    (UINT)wifi.dhcp_address[2],
                    (UINT)wifi.dhcp_address[3],
                    (UINT)(!wifi.link_up));
#endif
                previous_connected=is_connected;
                last_net_log_ms=now;
            }
        }
        while(b1_mqtt_receive_command(&f)>0){
            B1_Command request_command={0};
            int valid=b1_decode_command(f.payload,f.payload_len,
                &request_command)==0;
            if(valid && request_command.command!=B1_CMD_STOP &&
               !b1_recent_status_challenge(&store,f.time_ms,now)){
                /* Never execute stale commands. Return an explicit rejection
                 * where possible rather than creating a silent timeout. */
                acquire(NULL);store.invalid++;release(NULL);
                if(f.session==store.session && valid){
                    ack.type=B1_TYPE_ACK;ack.flags=0u;
                    ack.session=store.session;ack.time_ms=now;
                    acquire(NULL);ack.seq=++store.wire_seq;release(NULL);
                    ack.payload_len=4u;
                    ack.payload[0]=request_command.command;
                    b1_put16(ack.payload+1,request_command.transaction);
                    ack.payload[3]=B1_STALE_COMMAND;
                    if(b1_mqtt_submit(&ack)<0){
                        acquire(NULL);store.dropped++;release(NULL);
                    }
                }
            } else if(b1_handle_command(&store,&f,&ack)>=0){
                if(b1_mqtt_submit(&ack)<0){
                    acquire(NULL);store.dropped++;store.uncertain=1u;
            release(NULL);
                }
            }
        }
        /* Only copy buddy-owned values; no I2C, GPIO or PWM operations here. */
        (void)g7_telemetry_poll(&peers,&store);
#ifdef B1_LIVE_COMMANDS
        {
            B1_Mission mission={0};
            mission.ms=now;
            mission.mission=g7_controller_mission_state();
            mission.ready_blockers=g7_controller_remote_blockers();
            mission.ready=(uint8_t)(mission.ready_blockers==0u);
            mission.remote_hold=g7_controller_remote_hold_applied();
            mission.barcode_hold=bc_wave_motion_hold();
            motor_last_command(&mission.pwm_left, &mission.pwm_right);
            mission.running=(mission.mission==2u || mission.mission==3u);
            {
                uint8_t kind=0u,value=0u;
                uint16_t seq=0u;
                g7_controller_navigation_report(&kind,&value,&seq);
                if(seq!=nav_reported_seq && kind!=0u) {
                    mission.event_kind=kind;
                    mission.event_value=value;
                    mission.event_id=++mission_seq;
                    nav_reported_seq=seq;
                }
            }
            if(mission.mission!=last_mission && mission.event_kind==0u){
                mission.event_kind=10u;
                mission.event_id=++mission_seq;
                last_mission=mission.mission;
            }
            mission.seq=++mission_seq;
            (void)b1_ingest_mission(&store,&mission);
        }
#endif
        for(int i=0;i<6 && b1_next_frame(&store,now,&f)>0;i++){
            if(b1_mqtt_submit(&f)<0){
                acquire(NULL);store.dropped++;store.uncertain=1u;release(NULL);
            }
        }
        er=tk_dly_tsk(50);
        (void)er;
    }
}
int b1_rt_start(void){
    static const T_CMTX mutex_cfg={.mtxatr=TA_TFIFO|TA_INHERIT};
    static const T_CTSK task_cfg={.itskpri=9,.stksz=B1_TASK_STACK,
                                  .task=network_application_task,
            .tskatr=TA_HLNG|TA_RNG3};
    ID tid;
    mutex_id=tk_cre_mtx((T_CMTX*)&mutex_cfg);
    if(mutex_id<=0){tm_printf((UB*)"[buddy1] mutex create failed\n");return 1;}
    /* Session is a boot-local identifier, not cryptographic identity. */
    {
        extern uint64_t time_us_64(void);
        uint64_t boot=time_us_64();
        (void)b1_store_init(&store,acquire,release,NULL,
                             (uint16_t)((boot^(boot>>16)^0xB176u)|1u));
    }
#ifdef B1_LIVE_COMMANDS
    b1_set_coordinator(&store,g7_controller_remote_command,NULL);
#endif
#if G7_ODOMETRY_MODE != 0
    /* Telemetry ONLY. Nominal estimates are flagged as provisional.
     * The controller, barcode capture, GPIO and motor PWM are untouched. */
    peers.left_mm_per_count=(3.14159265f*G7_WHEEL_DIAMETER_MM)/
                            (float)G7_ENCODER_COUNTS_PER_REV;
    peers.right_mm_per_count=peers.left_mm_per_count;
    peers.odom_confidence=G7_ODOMETRY_MODE;
#endif
    tid=tk_cre_tsk((T_CTSK*)&task_cfg);
    if(tid<=0){tm_printf((UB*)"[buddy1] task create failed\n");return 2;}
    (void)tk_sta_tsk(tid,0);
    tm_printf((UB*)
        "[buddy1] MQTT via WiFi owner; telemetry task started.\n");
    return 0;
}
#else
int b1_rt_start(void) {return -1;}
#endif
/*** end of file ***/

