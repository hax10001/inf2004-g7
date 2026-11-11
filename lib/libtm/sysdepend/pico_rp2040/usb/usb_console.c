/*
 * Copyright (c) 2026 Muhamed Fauzi Bin Abbas
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 *	usb_console.c
 *	USB CDC-ACM backend for the T-Monitor console (RaspberryPi Pico / Pico W)
 *
 *	Producers (tm_snd_dat, called from any context including handlers) only
 *	append to a ring buffer and return.  A service task owns TinyUSB and
 *	drains the ring to the CDC endpoint.  An SMP producer may spin briefly
 *	behind the other processor's ring update, but it never waits for the host
 *	or performs a scheduler wait.  An unplugged or unenumerated host therefore
 *	cannot stall the kernel -- output is dropped and counted instead.
 */

#include <tk/tkernel.h>
#include <tk/typedef.h>
#include <sys/sysdef.h>

#if TK_SUPPORT_SMP
#include <tk/smp.h>
#endif

#if USE_TMONITOR && TM_CONSOLE_USB_CDC

/* usb_tinyusb_glue.c -- keeps TinyUSB headers out of this translation unit. */
IMPORT int	tm_tusb_init(void);
IMPORT void	tm_tusb_service(void);
IMPORT int	tm_tusb_event_ready(void);
IMPORT int	tm_tusb_mounted(void);
IMPORT int	tm_tusb_connected(void);
IMPORT unsigned	tm_tusb_write_available(void);
IMPORT void	tm_tusb_write_char(char value);
IMPORT void	tm_tusb_write_flush(void);
IMPORT void	tm_tusb_discard_input(void);

#define USB_TASK_PRIORITY	3
#if TK_SUPPORT_SMP
#define USB_TASK_STACK		4096
#else
#define USB_TASK_STACK		2048
#endif
#define USB_POLL_MS		10
#define USB_TX_SIZE		4096U
#define USB_TX_MASK		(USB_TX_SIZE - 1U)
#define USB_TX_CHUNK		64U
/* Diagnostic marker is emitted by the USB owner, not tm_printf().
 * If this arrives but normal [MAIN]/[NET] lines do not, USB itself works
 * and the failure lies in firmware startup or its normal log pipeline.
 * It is intentionally low-rate and only emitted after CDC enumeration. */
#ifdef B1_BARCODE_TEST
/* Quiet barcode test log: once every 10 minutes. */
#define USB_SELFTEST_INTERVAL_US 600000000U
#else
#define USB_SELFTEST_INTERVAL_US 5000000U
#endif
static const char usb_selftest[] =
    "\r\n[USB SELFTEST G7DIAG-V7-9-RECONNECT-20261102] CDC alive; BOOTNET below\r\n";

#if defined(B1_ROBOT_APP) && TM_WIFI_CYW43
#include "g7_wifi_scan_diag.h"
/* Independent of USB console ring and application task scheduling. */
IMPORT volatile unsigned int g7_usermain_stage;
IMPORT volatile int g7_usermain_error;
IMPORT void g7_usb_net_snapshot(W out[14]);

LOCAL void diag_text(UB *line, UW *len, UW cap, const char *text)
{
    while(*text != '\0' && *len + 1U < cap) {
        line[(*len)++] = (UB)*text++;
    }
}

LOCAL void diag_number(UB *line, UW *len, UW cap, W number)
{
    UW value = (UW)number;
    char digits[12];
    UW count = 0U;
    if(number < 0) {
        diag_text(line, len, cap, "-");
        value = 0U - (UW)number;
    }
    do {
        digits[count++] = (char)('0' + (value % 10U));
        value /= 10U;
    } while(value != 0U && count < sizeof(digits));
    while(count > 0U && *len + 1U < cap) {
        line[(*len)++] = (UB)digits[--count];
    }
}

LOCAL void usb_bootnet_selftest(void)
{
    static const char * const labels[] = {
        " stage=", " init=", " init_rc=", " join_started=", " join_req=",
        " join_done=", " join_rc=", " wifi=", " dhcp=", " dhcp_rc=",
        " mqtt="
    };
    W result[14] = {0};
    UB line[192];
    UW len = 0U;
    UW pos = 0U;
    UW remaining;
    unsigned available;

    g7_usb_net_snapshot(result);
    diag_text(line, &len, sizeof(line), "[BOOTNET]");
    diag_text(line, &len, sizeof(line), labels[0]);
    diag_number(line, &len, sizeof(line), (W)g7_usermain_stage);
    diag_text(line, &len, sizeof(line), " app_rc=");
    diag_number(line, &len, sizeof(line), (W)g7_usermain_error);
    for(UW i = 0U; i < 10U; i++) {
        diag_text(line, &len, sizeof(line), labels[i + 1U]);
        diag_number(line, &len, sizeof(line), result[i]);
    }
    diag_text(line, &len, sizeof(line), " ip=");
    for(UW i = 10U; i < 14U; i++) {
        if(i != 10U) diag_text(line, &len, sizeof(line), ".");
        diag_number(line, &len, sizeof(line), result[i]);
    }
    diag_text(line, &len, sizeof(line), "\r\n");
    /* Never wait for USB output. This is only a diagnostic. */
    while(pos < len) {
        available = tm_tusb_write_available();
        if(available == 0U) break;
        remaining = len - pos;
        if(available > remaining) available = (unsigned)remaining;
        for(unsigned j = 0U; j < available; j++) {
            tm_tusb_write_char((char)line[pos++]);
        }
        tm_tusb_write_flush();
    }
}
#endif

/* One reader over the existing CDC port: print discovered SSIDs once
 * per completed scan, without touching the Wi-Fi radio from USB context. */
#if defined(B1_ROBOT_APP) && TM_WIFI_CYW43
LOCAL void usb_scan_diagnostics(void)
{
    static UW printed_generation;
    static UW repeat_count;
    G7_WIFI_SCAN scan;
    UB line[128];
    UW len;
    UW i, j, pos;
    unsigned available;

    g7_wifi_scan_snapshot(&scan);
    if(scan.generation == 0U || !scan.complete) return;
    if(scan.generation == printed_generation) {
        /* Reprint every ~30s: a terminal opened later still sees the scan. */
        if(++repeat_count < 6U) return;
    }
    repeat_count = 0U;
    printed_generation = scan.generation;

    len = 0U;
    diag_text(line, &len, sizeof(line), "[WIFI SCAN] rc=");
    diag_number(line, &len, sizeof(line), scan.request_rc);
    diag_text(line, &len, sizeof(line), " total=");
    diag_number(line, &len, sizeof(line), (W)scan.seen_total);
    diag_text(line, &len, sizeof(line), " shown=");
    diag_number(line, &len, sizeof(line), (W)scan.stored);
    diag_text(line, &len, sizeof(line), " Pookie_found=");
    diag_number(line, &len, sizeof(line), (W)scan.target_seen);
    diag_text(line, &len, sizeof(line), "\r\n");
    for(pos = 0U; pos < len; ) {
        available = tm_tusb_write_available();
        if(available == 0U) break;
        for(j = 0U; j < available && pos < len; j++) {
            tm_tusb_write_char((char)line[pos++]);
        }
        tm_tusb_write_flush();
    }

    for(i = 0U; i < scan.stored && i < G7_SCAN_MAX_APS; i++) {
        len = 0U;
        diag_text(line, &len, sizeof(line), "[WIFI AP] ssid=\"");
        diag_text(line, &len, sizeof(line), scan.ap[i].ssid);
        diag_text(line, &len, sizeof(line), "\" ch=");
        diag_number(line, &len, sizeof(line), scan.ap[i].channel);
        diag_text(line, &len, sizeof(line), " rssi=");
        diag_number(line, &len, sizeof(line), scan.ap[i].rssi);
        diag_text(line, &len, sizeof(line), " auth=");
        diag_number(line, &len, sizeof(line), scan.ap[i].auth);
        diag_text(line, &len, sizeof(line), "\r\n");
        for(pos = 0U; pos < len; ) {
            available = tm_tusb_write_available();
            if(available == 0U) break;
            for(j = 0U; j < available && pos < len; j++) {
                tm_tusb_write_char((char)line[pos++]);
            }
            tm_tusb_write_flush();
        }
    }
}
#endif

static volatile UW const * const usb_timer_low =
    (volatile UW const *)0x40054028U;

LOCAL UB		tx_buffer[USB_TX_SIZE];
LOCAL volatile UW	tx_head;
LOCAL volatile UW	tx_tail;
LOCAL volatile UB	usb_started;

#if TK_SUPPORT_SMP
LOCAL T_SPLOCK		tx_lock;
#endif

/*
 * Keep the producer ring safe in all supported configurations.  Interrupt
 * masking is sufficient on UP; SMP additionally needs a cross-core lock.
 * The interrupt-preserving form is required because T-Monitor output is
 * permitted from both task and handler context.
 */
LOCAL void tx_lock_enter(UINT *intsts)
{
#if TK_SUPPORT_SMP
	(void)ISpinLock(&tx_lock, intsts);
#else
	DI(*intsts);
#endif
}

LOCAL void tx_lock_leave(UINT intsts)
{
#if TK_SUPPORT_SMP
	(void)ISpinUnlock(&tx_lock, intsts);
#else
	EI(intsts);
#endif
}

/* Bytes discarded because the ring was full.  Exposed so a test can assert
   that a run produced no loss rather than assuming it. */
EXPORT volatile UW	tm_usb_dropped;

#ifdef B1_ENABLE_MQTT
/*
 * Copy of the console output for the MQTT log topic, so the robot can be
 * debugged without a USB cable.  Filled under the same lock as the USB ring
 * and drained by the network owner through tm_netlog_take().  When the link
 * is slower than the output the newest bytes are dropped and counted, as for
 * USB; console output itself is never delayed by the network.
 */
#define NETLOG_SIZE		4096U
#define NETLOG_MASK		(NETLOG_SIZE - 1U)
LOCAL UB		netlog_buffer[NETLOG_SIZE];
LOCAL UW		netlog_head;
LOCAL UW		netlog_tail;
LOCAL UW		netlog_dropped;

/* Copy up to max queued bytes to out; *p_dropped receives (and clears) the
   bytes lost since the previous call.  Returns the number copied. */
EXPORT INT tm_netlog_take(UB *out, INT max, UW *p_dropped)
{
	UINT	intsts;
	INT	n = 0;

	tx_lock_enter(&intsts);
	while((n < max) && (netlog_tail != netlog_head)) {
		out[n++] = netlog_buffer[netlog_tail];
		netlog_tail = (netlog_tail + 1U) & NETLOG_MASK;
	}
	*p_dropped = netlog_dropped;
	netlog_dropped = 0U;
	tx_lock_leave(intsts);
	return n;
}

/* Queued bytes, without taking them. */
EXPORT INT tm_netlog_pending(void)
{
	UINT	intsts;
	UW	pending;

	tx_lock_enter(&intsts);
	pending = (netlog_head - netlog_tail) & NETLOG_MASK;
	tx_lock_leave(intsts);
	return (INT)pending;
}
#endif

/*
 * Link state, for diagnostics that cannot rely on the console itself:
 *	0  service task not started
 *	1  started, not enumerated by a host
 *	2  enumerated (mounted)
 *	3  enumerated and host asserted DTR
 */
EXPORT volatile UW	tm_usb_link_state;

EXPORT volatile UW	tm_usb_init_ok;

EXPORT UW tm_usb_state(void)
{
	return tm_usb_link_state;
}

/* Bytes still queued for the host.  A producer that wants its output to
   survive must wait for this to reach zero before doing anything that could
   overrun the ring. */
EXPORT UW tm_usb_pending(void)
{
	UINT	intsts;
	UW	pending;

	tx_lock_enter(&intsts);
	pending = (tx_head - tx_tail) & USB_TX_MASK;
	tx_lock_leave(intsts);

	return pending;
}

/*
 * Ordered bring-up checks, for readout over GPIO when USB does not
 * enumerate and the console therefore cannot describe its own failure.
 * Returns the number of consecutive checks that passed.
 */
IMPORT volatile UW	tm_usb_irq_count;
IMPORT volatile W	tm_usb_def_int_ercd;
IMPORT volatile UW	tm_usb_nvic_enabled;
IMPORT UW		tm_tusb_diag_controller_en(void);
IMPORT UW		tm_tusb_diag_pullup_en(void);
IMPORT UW		tm_tusb_diag_bus_seen(void);

EXPORT INT tm_usb_diag_level(void)
{
	if(!tm_usb_init_ok)		return 0;	/* tusb_init failed      */
	if(tm_usb_def_int_ercd != 0)	return 1;	/* tk_def_int failed     */
	if(!tm_usb_nvic_enabled)	return 2;	/* NVIC not unmasked     */
	if(!tm_tusb_diag_controller_en()) return 3;	/* controller disabled   */
	if(!tm_tusb_diag_pullup_en())	return 4;	/* D+ pull-up off        */
	if(tm_usb_irq_count == 0)	return 5;	/* no USB interrupt      */
	if(!tm_tusb_diag_bus_seen())	return 6;	/* no host bus activity  */
	if(tm_usb_link_state < 2)	return 7;	/* not enumerated        */
	return 8;					/* fully up              */
}

IMPORT volatile W	tm_usb_service_tskid;

LOCAL void usb_service_task(INT stacd, void *exinf)
{
	unsigned	available;
	UB		chunk[USB_TX_CHUNK];
	UW		count;
	UINT		intsts;
	UW		last_test_us = 0U;

	(void)stacd;
	(void)exinf;

	/* Publish the task id before the controller is initialised, so an
	   interrupt arriving during enumeration always has somewhere to go. */
	tm_usb_service_tskid = tk_get_tid();

	tm_usb_init_ok = tm_tusb_init() ? 1 : 0;
	if(!tm_usb_init_ok) {
		tm_usb_service_tskid = 0;
		tk_ext_tsk();
		return;
	}

	tm_usb_link_state = 1;

	for(;;) {
		/* Drain the whole event burst before yielding.  Returning to
		   the scheduler between events would put a tick period into
		   each stage of every control transfer. */
		do {
			tm_tusb_service();
		} while(tm_tusb_event_ready());

		tm_usb_link_state = tm_tusb_mounted()
				  ? (tm_tusb_connected() ? 3 : 2)
				  : 1;

		/* Gate on enumeration, not on DTR.  If the host is not reading,
		   write_available() falls to zero and the loop simply stops --
		   it never waits. */
		if(tm_tusb_mounted()) {
            /* Independently validate USB CDC even if usermain never starts.
             * Never call tm_printf from the USB owner (would feed its own ring). */
            UW now_us = *usb_timer_low;
            if(last_test_us == 0U ||
               (UW)(now_us - last_test_us) >= USB_SELFTEST_INTERVAL_US) {
                unsigned required = (unsigned)(sizeof(usb_selftest) - 1U);
                if(tm_tusb_write_available() >= required) {
                    for(unsigned i = 0U; i < required; i++) {
                        tm_tusb_write_char(usb_selftest[i]);
                    }
                    tm_tusb_write_flush();
#if defined(B1_ROBOT_APP) && TM_WIFI_CYW43
                    usb_bootnet_selftest();
                    usb_scan_diagnostics();
#else
                    /* Explicitly identify a build lacking the expected
                     * networking flags, rather than silently omitting it. */
                    static const char no_net[] =
                        "[BOOTNET] DISABLED_IN_BUILD\r\n";
                    if(tm_tusb_write_available() >= sizeof(no_net) - 1U) {
                        for(unsigned i=0U; i<sizeof(no_net)-1U; i++) {
                            tm_tusb_write_char(no_net[i]);
                        }
                        tm_tusb_write_flush();
                    }
#endif
                    last_test_us = now_us == 0U ? 1U : now_us;
                }
            }
			available = tm_tusb_write_available();
			while(available > 0) {
				count = 0;
				tx_lock_enter(&intsts);
				while(count < USB_TX_CHUNK && count < available &&
				      tx_tail != tx_head) {
					chunk[count++] = tx_buffer[tx_tail];
					tx_tail = (tx_tail + 1U) & USB_TX_MASK;
				}
				tx_lock_leave(intsts);

				if(count == 0) break;
				for(UW i = 0; i < count; i++) {
					tm_tusb_write_char((char)chunk[i]);
				}
				available -= count;
			}
			tm_tusb_write_flush();
		}
		tm_tusb_discard_input();

		/* Sleep until the controller interrupts, with a periodic
		   fallback so the ring still drains when the link is idle. */
		tk_slp_tsk(USB_POLL_MS);
	}
}

/*
 * Start the console service.  Called from knl_start_device(), i.e. from the
 * initial task once the kernel is fully up, so every application gets the
 * console without having to wire it in itself.
 */
EXPORT ER tm_usb_console_start(void)
{
	T_CTSK	ctsk;
	ID	tskid;
	ER	ercd;

	if(usb_started) return E_OK;

#if TK_SUPPORT_SMP
	ercd = InitSpinLock(&tx_lock);
	if(ercd < E_OK) return ercd;
#endif

	/* TinyUSB is initialised inside the task itself, so the task id is
	   published before the controller can raise its first interrupt. */
	/*
	 * Pinned to processor 1 (core 0).
	 *
	 * TinyUSB masks the USB interrupt by writing the calling core's NVIC,
	 * and the NVIC is per core.  A service task free to migrate would
	 * enable the interrupt on whichever core happened to run tusb_init()
	 * and then take its critical sections on the other one, where the mask
	 * protects nothing -- and enumeration fails, which is precisely how
	 * this resurfaced once the global scheduler was allowed to place this
	 * task.  Interrupt ownership and the task that services it must live on
	 * the same processor.
	 */
	ctsk.exinf   = NULL;
#if TK_SUPPORT_SMP
	ctsk.tskatr  = TA_HLNG | TA_RNG0 | TA_ASSPRC;
	ctsk.assprc  = TP_PRC1;
#else
	ctsk.tskatr  = TA_HLNG | TA_RNG0;
	ctsk.assprc  = 0;
#endif
	ctsk.task    = (FP)usb_service_task;
	ctsk.itskpri = USB_TASK_PRIORITY;
	ctsk.stksz   = USB_TASK_STACK;
	ctsk.bufptr  = NULL;

	tskid = tk_cre_tsk(&ctsk);
	if(tskid <= 0) return (ER)tskid;

	ercd = tk_sta_tsk(tskid, 0);
	if(ercd < E_OK) {
		(void)tk_del_tsk(tskid);
		return ercd;
	}

	usb_started = 1;
	return E_OK;
}

/*
 * Append to the ring.  This may spin on the short SMP ring lock, but never
 * waits on the host and never performs a scheduler wait.
 */
EXPORT void tm_usb_console_put(const UB *buf, INT size)
{
	UW	next;
	UINT	intsts;

	tx_lock_enter(&intsts);
#ifdef B1_ENABLE_MQTT
	{
		INT	i;

		for(i = 0; i < size; i++) {
			next = (netlog_head + 1U) & NETLOG_MASK;
			if(next == netlog_tail) {
				netlog_dropped++;
				continue;
			}
			netlog_buffer[netlog_head] = buf[i];
			netlog_head = next;
		}
	}
#endif
	while(size-- > 0) {
		next = (tx_head + 1U) & USB_TX_MASK;
		if(next == tx_tail) {		/* full: drop, and count it */
			tm_usb_dropped++;
			buf++;
			continue;
		}
		tx_buffer[tx_head] = *buf++;
		tx_head = next;
	}
	tx_lock_leave(intsts);
}

#endif /* USE_TMONITOR && TM_CONSOLE_USB_CDC */

/* test changed source */
