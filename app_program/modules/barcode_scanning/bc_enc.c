/** @file bc_enc.c
 * @brief Three hand-roll encoder passes with motors held at zero PWM.
 */
#include "bc_enc.h"
#include "bc_cal.h"
#include "drivers/encoder.h"
#include "drivers/motor.h"
#include "platform/console.h"
#include <tm/tmonitor.h>

#if BC_CAL_ENCODER_ONLY && defined(_PICO_RP2040_) && \
    (!defined(TM_CONSOLE_USB_CDC) || !TM_CONSOLE_USB_CDC)
#error Encoder calibration requires CONSOLE=usb_cdc; GP0/GP1 are encoders
#endif

#define BC_ENC_PASSES 3U
#define BC_ENC_PREP_SECONDS 10U
#define BC_ENC_ROLL_MSEC 15000U
#define BC_ENC_POLL_MSEC 20U
#define BC_ENC_STILL_MSEC 1000U
#define BC_ENC_SIGNED_MAX 2147483647LL

static void bc_enc_read (encoder_sample_t * p_left,
                         encoder_sample_t * p_right)
{
    encoder_read(ENCODER_LEFT, p_left);
    encoder_read(ENCODER_RIGHT, p_right);
}

static void bc_enc_pass (UINT pass)
{
    encoder_sample_t left_start = {0};
    encoder_sample_t right_start = {0};
    encoder_sample_t left = {0};
    encoder_sample_t right = {0};
    uint32_t still_msec = 0U;
    uint32_t start_usec;
    uint32_t last_move_usec;

    tm_printf((UB *)"[ENC CAL] PREP pass=%u: place at ruler start; "
                      "keep still until GO\n", pass);
    for (UINT seconds = BC_ENC_PREP_SECONDS; seconds > 0U; --seconds)
    {
        tm_printf((UB *)"[ENC CAL] GO in %u seconds\n", seconds);
        tk_dly_tsk(1000U);
    }
    bc_enc_read(&left_start, &right_start);
    left = left_start;
    right = right_start;
    tm_printf((UB *)"[ENC CAL] GO pass=%u L_start=%d R_start=%d: "
                      "push forward 100 mm, stop; window=15 seconds\n",
              pass, (INT)left_start.count, (INT)right_start.count);
    start_usec = encoder_now_usec();
    last_move_usec = start_usec;

    for (; (uint32_t)(encoder_now_usec() - start_usec) <
           BC_ENC_ROLL_MSEC * 1000U; )
    {
        int32_t previous_left = left.count;
        int32_t previous_right = right.count;
        tk_dly_tsk(BC_ENC_POLL_MSEC);
        bc_enc_read(&left, &right);
        uint32_t now_usec = encoder_now_usec();
        if ((left.count != previous_left) ||
            (right.count != previous_right))
        {
            last_move_usec = now_usec;
        }
        still_msec = (uint32_t)(now_usec - last_move_usec) / 1000U;
    }

    int64_t delta_left = (int64_t)left.count - left_start.count;
    int64_t delta_right = (int64_t)right.count - right_start.count;
    if ((delta_left > BC_ENC_SIGNED_MAX) ||
        (delta_left < -BC_ENC_SIGNED_MAX) ||
        (delta_right > BC_ENC_SIGNED_MAX) ||
        (delta_right < -BC_ENC_SIGNED_MAX))
    {
        tm_printf((UB *)"[ENC CAL] INVALID pass=%u reason=count_range\n",
                  pass);
    }
    else
    {
        tm_printf((UB *)"[ENC CAL CSV] %u,100,%d,%d,%d,%d,%d,%d,%u\n",
                  pass, (INT)left_start.count, (INT)left.count,
                  (INT)right_start.count, (INT)right.count,
                  (INT)delta_left, (INT)delta_right,
                  (UINT)still_msec);
        if ((0 == delta_left) || (0 == delta_right) ||
            (still_msec < BC_ENC_STILL_MSEC))
        {
            tm_printf((UB *)"[ENC CAL] INVALID pass=%u: wheel did not "
                              "count or robot not stopped; repeat session\n",
                      pass);
        }
        else
        {
            tm_printf((UB *)"[ENC CAL] DONE pass=%u: confirm actual "
                              "forward distance was 100 mm, without slip\n",
                      pass);
        }
    }
}

INT bc_enc_run (void)
{
    /* Stop before waiting for USB. No motion subsystem is initialized. */
    motor_init();
    motor_stop();
    ER error = encoder_init();

    /* State 3 means the serial terminal is open, not just USB enumerated. */
    for (; console_state() < 3U; )
    {
        tk_dly_tsk(100U);
    }
    tk_dly_tsk(500U);
    if (error < E_OK)
    {
        tm_printf((UB *)"[ENC CAL] FAILED encoder_init=%d; motors off\n",
                  (INT)error);
        return 1;
    }
    tm_printf((UB *)"\n[ENC CAL] ENCODER-ONLY; motor PWM=0; "
                      "controller/IR/IMU/barcode tasks OFF\n");
    tm_printf((UB *)"[ENC CAL] Keep motor power disconnected. Mark a "
                      "100 mm track; each pass has 10s prep + 15s roll.\n");
    tm_printf((UB *)"[ENC CAL] Reposition only during PREP. "
                      "Save this entire session and report any bad pass.\n");
    tm_printf((UB *)"[ENC CAL CSV] pass,requested_mm,L_start,L_end,"
                      "R_start,R_end,L_delta,R_delta,still_msec\n");
    for (UINT pass = 1U; pass <= BC_ENC_PASSES; ++pass)
    {
        bc_enc_pass(pass);
    }
    tm_printf((UB *)"[ENC CAL] SESSION COMPLETE; motors remain off. "
                      "Reset to repeat.\n");
    for (;;)
    {
        tk_slp_tsk(TMO_FEVR);
    }
}

/*** end of file ***/

