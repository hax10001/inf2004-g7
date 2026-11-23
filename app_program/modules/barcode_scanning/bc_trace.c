/** @file bc_trace.c
 * @brief One diagnostic capture per boot; bounded storage and paced output.
 */
#include "bc_trace.h"
#include "bc_port.h"
#include "bc_cal.h"
#include "app/app_config.h"
#include <tm/tmonitor.h>

#if APP_BARCODE_ANALOG_ENABLED && APP_BARCODE_TRACE_ENABLED && BC_CAL_READY
#define BC_TRACE_CAPACITY 4096U
#define BC_TRACE_LEAD_IN 128U
#define BC_TRACE_QUIET_USEC 1200000U
#define BC_TRACE_REASON_QUIET 1U
#define BC_TRACE_REASON_FULL 2U

typedef struct
{
    uint32_t time_usec;
    int32_t left;
    int32_t right;
    uint16_t adc;
    uint8_t level;
    uint8_t flags; /**< Bit 0 capture validity, bit 1 ADC validity. */
} bc_trace_row_t;

/* 4096 x 16 bytes = 64 KiB, preserving the eight-second trace window. */
static bc_trace_row_t g_trace[BC_TRACE_CAPACITY];
static uint32_t g_write;
static uint32_t g_count;
static uint32_t g_last_change_usec;
static uint32_t g_read;
static uint32_t g_remaining;
static uint32_t g_origin_usec;
static int32_t g_last_left;
static int32_t g_last_right;
static uint8_t g_last_level;
static uint8_t gb_have_sample;
static uint8_t gb_moved;
static uint8_t gb_triggered;
static uint8_t g_reason;
static uint8_t gb_dump_started;
static uint8_t gb_dump_done;

void bc_trace_record (bc_fastq_sample_t const * p_sample)
{
    if (0U == g_reason)
    {
        if (0U != gb_have_sample)
        {
            if ((p_sample->left_count != g_last_left) ||
                (p_sample->right_count != g_last_right))
            {
                gb_moved = 1U;
            }
            if ((0U != p_sample->adc_valid) &&
                (p_sample->gpio_level != g_last_level))
            {
                g_last_change_usec = p_sample->time_usec;
            }
        }
        if ((0U == gb_triggered) && (0U != gb_moved) &&
            (0U != p_sample->adc_valid) &&
            (p_sample->gpio_level == BC_CAL_BLACK_LEVEL))
        {
            gb_triggered = 1U;
            g_last_change_usec = p_sample->time_usec;
        }

        bc_trace_row_t * const p_row = &g_trace[g_write];
        p_row->time_usec = p_sample->time_usec;
        p_row->left = p_sample->left_count;
        p_row->right = p_sample->right_count;
        p_row->adc = p_sample->adc_raw;
        p_row->level = p_sample->gpio_level;
        p_row->flags = (uint8_t)((p_sample->valid ? 1U : 0U) |
                                  (p_sample->adc_valid ? 2U : 0U));
        g_write = (g_write + 1U) % BC_TRACE_CAPACITY;
        if (g_count < BC_TRACE_CAPACITY)
        {
            ++g_count;
        }
        if (0U == gb_triggered)
        {
            if (g_count > BC_TRACE_LEAD_IN)
            {
                g_count = BC_TRACE_LEAD_IN;
            }
        }
        else if (g_count == BC_TRACE_CAPACITY)
        {
            g_reason = BC_TRACE_REASON_FULL;
        }
        else if ((uint32_t)(p_sample->time_usec - g_last_change_usec) >=
                 BC_TRACE_QUIET_USEC)
        {
            g_reason = BC_TRACE_REASON_QUIET;
        }
        g_last_left = p_sample->left_count;
        g_last_right = p_sample->right_count;
        if (0U != p_sample->adc_valid)
        {
            g_last_level = p_sample->gpio_level;
        }
        gb_have_sample = 1U;
    }
}

void bc_trace_poll (void)
{
    if ((0U != g_reason) && (0U == gb_dump_done))
    {
        if (0U == gb_dump_started)
        {
            bc_scan_t scan = {0};
            if (E_OK == bc_port_snapshot(&scan))
            {
                tm_printf((UB *)"[BC TRACE SCAN] state=%u runs=%u/29 "
                                  "error=%u\n",
                          (UINT)scan.state, (UINT)scan.run_count,
                          (UINT)scan.error);
                /* Include partial widths: ordinary FRAME only reports
                 * terminal states. This snapshot is at dump start.
                 */
                for (uint32_t i = 0U;
                     (i < scan.run_count) && (i < BC_CODE_ELEMENTS); ++i)
                {
                    tm_printf((UB *)"[BC TRACE WIDTH] i=%u units=%u\n",
                              (UINT)i, (UINT)scan.widths[i]);
                }
            }
            g_read = (g_write + BC_TRACE_CAPACITY - g_count) %
                     BC_TRACE_CAPACITY;
            g_remaining = g_count;
            g_origin_usec = g_trace[g_read].time_usec;
            tm_printf((UB *)"[BC TRACE BEGIN] reason=%u rows=%u "
                              "threshold=%u hysteresis=%u "
                              "columns=dt_us,adc,left,right,level,flags\n",
                      (UINT)g_reason, (UINT)g_count,
                      (UINT)APP_BARCODE_ADC_THRESHOLD,
                      (UINT)APP_BARCODE_ADC_HYSTERESIS);
            gb_dump_started = 1U;
        }
        else if (0U != g_remaining)
        {
            bc_trace_row_t const * const p_row = &g_trace[g_read];
            tm_printf((UB *)"[BC TRACE] %u,%u,%d,%d,%u,%u\n",
                      (UINT)(p_row->time_usec - g_origin_usec),
                      (UINT)p_row->adc, (INT)p_row->left, (INT)p_row->right,
                      (UINT)p_row->level, (UINT)p_row->flags);
            g_read = (g_read + 1U) % BC_TRACE_CAPACITY;
            --g_remaining;
        }
        else
        {
            tm_printf((UB *)"[BC TRACE END] rows=%u; reboot to rearm\n",
                      (UINT)g_count);
            gb_dump_done = 1U;
        }
    }
}
#else
void bc_trace_record (bc_fastq_sample_t const * p_sample)
{
    (void)p_sample;
}

void bc_trace_poll (void)
{
}
#endif

/*** end of file ***/

