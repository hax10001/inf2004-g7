/** @file buddy1_rt.c
 * @brief Buddy 1 telemetry task: gathers subsystem state, sends frames and
 * applies commands. Never calls lwIP directly.
 */
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

#define B1_TASK_STACK 4096
#define B1_TASK_PRIORITY 9
/* Task period (ms) and frames sent per period. */
#define B1_TASK_PERIOD_MSEC 50
#define B1_FRAMES_PER_PERIOD 6
/* Wi-Fi status is reported once after this boot deadline, then the
 * network line is logged on every change or at this interval. */
#define B1_WIFI_DEADLINE_MSEC 10000u
#define B1_NET_LOG_MSEC 5000u
/* Mission state that never occurs: forces the first mission EVENT. */
#define B1_MISSION_NONE 255u
/* MQTT link state that never occurs: forces the first [NET] line. */
#define B1_LINK_UNKNOWN 255u
#define B1_US_PER_MS 1000ULL      /* microseconds per millisecond */
/* g7_nav_take_scan() results besides 1 (mapped) and 0 (nothing). */
#define B1_SCAN_NOT_MAPPED (-1)   /* valid letter, not in the mapping */
#define B1_SCAN_NO_LETTER (-2)    /* scan finished without a letter */
#define B1_SCAN_FAULT (-3)        /* bad argument or result overflow */
/* DHCP address octets three and four in the [NET] line. */
#define B1_IP_OCTET_3 2
#define B1_IP_OCTET_4 3
/* Session seed: fold the boot time and mix in a fixed salt. */
#define B1_SESSION_FOLD 16
#define B1_SESSION_SALT 0xB176u
#define B1_PI_F 3.14159265f       /* wheel circumference factor */
#define B1_RT_TASK_FAILED 2       /* b1_rt_start(): task not created */

static b1_store_t g_store;
static g7_telemetry_peers_t g_peers;
static uint16_t g_mission_seq;
/* Mission EVENT IDs have their own counter so the record sequence has no
 * gaps; the store treats a gap as a lost record and flags it uncertain. */
static uint16_t g_mission_event_seq;
static uint8_t g_last_mission=B1_MISSION_NONE;
static ID g_mutex_id;
/** @brief Lock the telemetry store mutex. */
static void acquire(void *p_ctx)
{
    (void)p_ctx;
    (void)tk_loc_mtx(g_mutex_id,TMO_FEVR);
}
/** @brief Unlock the telemetry store mutex. */
static void release(void *p_ctx)
{
    (void)p_ctx;
    (void)tk_unl_mtx(g_mutex_id);
}

void g7_nav_clear_decoded(void)
{
    (void)bc_port_nav_enable(0U);
    bc_wave_request_cancel();
}
int32_t g7_nav_begin_run(void)
{
    /* Enable the handoff only; never cancel a scanner that just reported
     * ready. */
    if (!bc_wave_is_ready())
    {
        return 0;
    }
    return bc_port_nav_enable(1U) >= E_OK;
}
int32_t g7_nav_continue_run(void)
{
    /* The robot keeps driving while the scanner rearms, as it already does
     * after every barcode. Only a capture that begins after this call is
     * owned by the run (scan_handoff_begin), so a result from before the
     * bypass or lift can never be handed to navigation. */
    return bc_port_nav_enable(1U) >= E_OK;
}
int32_t g7_nav_take_scan(uint8_t *p_letter, uint8_t *p_action,
    uint16_t *p_sequence,
    uint32_t *p_captured_msec, uint32_t *p_completed_msec,
    uint8_t *p_pending)
{
    scan_handoff_t scan = {0};
    int32_t result = 0;
    if (!p_letter || !p_action || !p_sequence || !p_captured_msec ||
        !p_completed_msec || !p_pending)
    {
        return B1_SCAN_FAULT;
    }
    /* On contention do not pretend the decoder is idle. Never consume a
     * result until mapping can be read under its existing mutex. */
    *p_pending = 1U;
    if (g_mutex_id <= 0 || tk_loc_mtx(g_mutex_id, TMO_POL) < E_OK)
    {
        return 0;
    }
    if (bc_port_nav_take(&scan) < E_OK)
    {
        (void)tk_unl_mtx(g_mutex_id);
        return 0;
    }
    *p_pending = (uint8_t)(scan.busy || scan.ready);
    if (0U != scan.ready)
    {
        uint8_t i;
        *p_letter = (uint8_t)scan.letter;
        *p_sequence = scan.sequence;
        *p_captured_msec = scan.captured_msec;
        *p_completed_msec = scan.completed_msec;
        result = (scan.letter < B1_LETTER_COUNT) ? B1_SCAN_NOT_MAPPED :
            B1_SCAN_NO_LETTER;
        if (0U != scan.overflow)
        {
            result = B1_SCAN_FAULT;
        }
        if (scan.letter < B1_LETTER_COUNT && g_store.mapping_valid)
        {
            for (i = 0U; i < B1_MAP_LETTERS; ++i)
            {
                if (g_store.mapping[i] == *p_letter)
                {
                    *p_action = i;
                    result = 1;
                    break;
                }
            }
        }
    }
    (void)tk_unl_mtx(g_mutex_id);
    return result;
}
int32_t g7_nav_take_decoded(uint8_t *p_letter, uint8_t *p_action,
    uint16_t *p_sequence)
{
    uint32_t captured = 0U;
    uint32_t completed = 0U;
    uint8_t pending = 0U;
    return g7_nav_take_scan(p_letter, p_action, p_sequence, &captured,
        &completed,
        &pending);
}


/** @brief Telemetry task: gather, publish and receive commands. */
static void network_application_task(INT stacd,void *p_exinf)
{
    b1_frame_t f,ack;
    ER er;
    uint32_t now;
    uint32_t boot_ms=0u;
    uint8_t reported_network_deadline=0u;
    uint8_t previous_connected=B1_LINK_UNKNOWN;
    uint32_t last_net_log_ms=0u;
    uint16_t nav_reported_seq=0u;
    (void)stacd;
    (void)p_exinf;
    for(;;)
    {
        /* Commands arrive from the CYW43 owner through an SPSC queue. All
         * but STOP must echo a STATUS time sent recently in this boot. */
        {
            extern uint64_t time_us_64(void);
            now=(uint32_t)(time_us_64()/B1_US_PER_MS);
        }
        if(boot_ms==0u)
        {
            boot_ms=now;
        }
        if(!reported_network_deadline && now-boot_ms>=B1_WIFI_DEADLINE_MSEC)
        {
            T_CYW43_UTK_STATUS wifi={0};
            cyw43_utk_get_status(&wifi);
            reported_network_deadline=1u;
            if(!wifi.link_up)
            {
                tm_printf((UB *)
                    "[buddy1] WiFi not ready after 10s; "
                    "autonomous controller continues.\n");
            }
            else
            {
                tm_printf((UB *)
                    "[buddy1] WiFi link up; MQTT is optional "
                    "for autonomous operation.\n");
            }
        }
        {
            uint8_t is_connected=b1_mqtt_connected();
            if(is_connected!=previous_connected ||
                now-last_net_log_ms>=B1_NET_LOG_MSEC)
            {
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
                    (UINT)wifi.dhcp_address[B1_IP_OCTET_3],
                    (UINT)wifi.dhcp_address[B1_IP_OCTET_4],
                    (UINT)(!wifi.link_up));
#endif
                previous_connected=is_connected;
                last_net_log_ms=now;
            }
        }
        while(b1_mqtt_receive_command(&f)>0)
        {
            b1_command_t request_command={0};
            int32_t valid=b1_decode_command(f.payload,f.payload_len,
                &request_command)==0;
            if(valid && request_command.command!=B1_CMD_STOP &&
                !b1_recent_status_challenge(&g_store,f.time_ms,now))
            {
                /* Never execute stale commands. Return an explicit rejection
                 * where possible rather than creating a silent timeout. */
                acquire(NULL);
                g_store.invalid++;
                release(NULL);
                if(f.session==g_store.session && valid)
                {
                    ack.type=B1_TYPE_ACK;
                    ack.flags=0u;
                    ack.session=g_store.session;
                    ack.time_ms=now;
                    acquire(NULL);
                    ack.seq=++g_store.wire_seq;
                    release(NULL);
                    ack.payload_len=B1_ACK_LEN;
                    ack.payload[0]=request_command.command;
                    b1_put16(ack.payload+B1_ACK_OFF_TRANSACTION,
                        request_command.transaction);
                    ack.payload[B1_ACK_OFF_RESULT]=B1_STALE_COMMAND;
                    if(b1_mqtt_submit(&ack)<0)
                    {
                        acquire(NULL);
                        g_store.dropped++;
                        release(NULL);
                    }
                }
            }
            else if(b1_handle_command(&g_store,&f,&ack)>=0)
            {
                if(b1_mqtt_submit(&ack)<0)
                {
                    acquire(NULL);
                    g_store.dropped++;
                    g_store.uncertain=1u;
                    release(NULL);
                }
            }
        }
        /* Copies subsystem values only; no I2C, GPIO or PWM access. */
        (void)g7_telemetry_poll(&g_peers,&g_store);
#ifdef B1_LIVE_COMMANDS
        {
            b1_mission_t mission={0};
            mission.ms=now;
            mission.mission=g7_controller_mission_state();
            mission.ready_blockers=g7_controller_remote_blockers();
            mission.ready=(uint8_t)(mission.ready_blockers==0u);
            mission.remote_hold=g7_controller_remote_hold_applied();
            mission.barcode_hold=bc_wave_motion_hold();
            motor_last_command(&mission.pwm_left, &mission.pwm_right);
            mission.running=(mission.mission==B1_MISSION_RUNNING ||
                mission.mission==B1_MISSION_OBSTACLE);
            {
                uint8_t kind=0u;
                uint8_t value=0u;
                uint16_t seq=0u;
                g7_controller_navigation_report(&kind,&value,&seq);
                if(seq!=nav_reported_seq && kind!=0u)
                {
                    mission.event_kind=kind;
                    mission.event_value=value;
                    if(++g_mission_event_seq==0u)
                    {
                        g_mission_event_seq=1u;
                    }
                    mission.event_id=g_mission_event_seq;
                    nav_reported_seq=seq;
                }
            }
            if(mission.mission!=g_last_mission && mission.event_kind==0u)
            {
                mission.event_kind=B1_EVT_MISSION_STATE;
                if(++g_mission_event_seq==0u)
                {
                    g_mission_event_seq=1u;
                }
                mission.event_id=g_mission_event_seq;
                g_last_mission=mission.mission;
            }
            mission.seq=++g_mission_seq;
            (void)b1_ingest_mission(&g_store,&mission);
        }
#endif
        /* While MQTT is down (or the queue is full) frames stay in the
         * store's bounded rings, 32 ODOM and 24 EVENT, and are sent in
         * order after reconnect; ring overflow is counted as dropped. */
        for(int32_t i=0;
            i<B1_FRAMES_PER_PERIOD && b1_mqtt_can_send() &&
            b1_next_frame(&g_store,now,&f)>0;
            i++)
        {
            if(b1_mqtt_submit(&f)<0)
            {
                acquire(NULL);
                g_store.dropped++;
                g_store.uncertain=1u;
                release(NULL);
            }
        }
        er=tk_dly_tsk(B1_TASK_PERIOD_MSEC);
        (void)er;
    }
}
/** @brief Create and start the telemetry task. */
int32_t b1_rt_start(void)
{
    static const T_CMTX mutex_cfg={.mtxatr=TA_TFIFO|TA_INHERIT};
    static const T_CTSK task_cfg={.itskpri=B1_TASK_PRIORITY,
        .stksz=B1_TASK_STACK,
        .task=network_application_task,
        .tskatr=TA_HLNG|TA_RNG3};
    ID tid;
    g_mutex_id=tk_cre_mtx((T_CMTX*)&mutex_cfg);
    if(g_mutex_id<=0)
    {
        tm_printf((UB*)"[buddy1] mutex create failed\n");
        return 1;
    }
    /* Session is a boot-local identifier, not cryptographic identity. */
    {
        extern uint64_t time_us_64(void);
        uint64_t boot=time_us_64();
        (void)b1_store_init(&g_store,acquire,release,NULL,
            (uint16_t)((boot^(boot>>B1_SESSION_FOLD)^B1_SESSION_SALT)|1u));
    }
#ifdef B1_LIVE_COMMANDS
    b1_set_coordinator(&g_store,g7_controller_remote_command,NULL);
#endif
#if G7_ODOMETRY_MODE != 0
    /* Odometry scale for telemetry only; odom_confidence marks nominal
     * values. Control, barcode capture and PWM do not use it. */
    g_peers.left_mm_per_count=(B1_PI_F*G7_WHEEL_DIAMETER_MM)/
    (float)G7_ENCODER_COUNTS_PER_REV;
    g_peers.right_mm_per_count=g_peers.left_mm_per_count;
    g_peers.odom_confidence=G7_ODOMETRY_MODE;
#endif
    tid=tk_cre_tsk((T_CTSK*)&task_cfg);
    if(tid<=0)
    {
        tm_printf((UB*)"[buddy1] task create failed\n");
        return B1_RT_TASK_FAILED;
    }
    (void)tk_sta_tsk(tid,0);
    tm_printf((UB*)
        "[buddy1] MQTT via WiFi owner; telemetry task started.\n");
    return 0;
}
#else
/** @brief Create and start the telemetry task. */
int32_t b1_rt_start(void)
{
    return -1;
}
#endif
/*** end of file ***/

