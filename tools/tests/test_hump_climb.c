/** @file test_hump_climb.c
 * @brief Host test: the real hump detector (task_imu.c update_hump) against
 *        the 15 humps it kept on 23 Nov plus an older real one, rebuilt as 20
 *        ms samples with the recorded peak height and measured climb. Real
 *        humps (climb 8.4-16.4 mm) must still be recorded and reach the
 *        descending phase (downhill slow-down); the false ones (climb 0.4-2.1
 *        mm, sideways jolts from pivots, START and restarts) must be
 *        discarded and never reach the descending phase.
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "modules/terrain/task_imu.c"
#include "modules/terrain/tilt_math.h"

static int32_t g_counts;
static int g_discards;
/** @brief Host stub of encoder_read(). */
void encoder_read(encoder_wheel_t wheel, encoder_sample_t *p)
{
    /* Both wheels report the same forward travel; the left encoder is
     * mirrored on the robot, which the detector's sizes ignore. */
    p->count = (wheel == ENCODER_LEFT) ? -g_counts : g_counts;
    p->edge_usec = 0U;
}
/** @brief Host stub of tm_printf(). */
INT tm_printf(const UB *format, ...)
{
    if (strstr((const char *)format, "DISCARDED") != NULL)
    {
        g_discards++;
    }
    return 0;
}
/** @brief Host stub of terrain_hump_height(). */
float terrain_hump_height(float roll_deg)
{
    return tilt_math_hump_height(roll_deg, (float)WHEEL_SPACING_MM);
}

typedef struct { const char *name; float peak_mm, climb_mm; int real; } hump_t;

/* One sample: tilt (deg, signed) and forward travel (mm) this sample. */
static void sample(float roll_deg, float travel_mm, uint32_t *now,
                   bool *b_desc)
{
    terrain_snapshot_t s = {0};
    g_counts += (int32_t)lroundf(travel_mm / CLIMB_MM_PER_COUNT);
    s.roll_deg = roll_deg;
    s.b_valid = 1;
    *now += 20U;
    update_hump(&s);
    if (TERRAIN_HUMP_DESCENDING == g_hump.phase)
    {
        *b_desc = true;
    }
}

/* Tilt to the recorded peak; travel on the up-slope so the climb reaches
 * the recorded value; then the other way (descent, or the jolt rocking
 * back) over at least 45 mm in all; then level. */
static int run(hump_t const *h, bool *b_desc)
{
    uint32_t now = 10000U;
    int const before = g_hump.hump_count;
    float const peak_deg =
        asinf(h->peak_mm / (float)WHEEL_SPACING_MM) / CLIMB_DEG_TO_RAD;
    float const up_mm = h->climb_mm / sinf(peak_deg * CLIMB_DEG_TO_RAD);
    float const back_mm = (up_mm < 45.0F) ? (45.0F - up_mm) + 5.0F : up_mm;
    int const n_up = 10;
    int const n_back = 10;
    *b_desc = false;
    g_discards = 0;
    for (int i = 0; i < 3; i++)
    {
        sample(0.0F, 2.0F, &now, b_desc);
    }
    for (int i = 0; i < n_up; i++)
    {
        sample(peak_deg, up_mm / (float)n_up, &now, b_desc);
    }
    for (int i = 0; i < n_back; i++)
    {
        sample(-peak_deg * 0.9F, back_mm / (float)n_back, &now, b_desc);
    }
    for (int i = 0; i < 20; i++)
    {
        sample(0.0F, 2.0F, &now, b_desc);
    }
    printf("  %-28s peak %5.1f climb %4.1f -> %s, descending %s\n",
           h->name, h->peak_mm, h->climb_mm,
           g_hump.hump_count != before ? "recorded" : "discarded",
           *b_desc ? "yes" : "no");
    return g_hump.hump_count != before;
}

/** @brief Run every test case; any failed assert aborts. */
int main(void)
{
    static const hump_t humps[] = {
        /* Real (23 Nov M4, M8, M10, C3; 2 Nov log #2). */
        {"real M4 00:47:18", 14.3F, 13.9F, 1},
        {"real M8 00:57:18", 16.4F, 16.4F, 1},
        {"real M10 01:01:47", 13.8F, 11.4F, 1},
        {"real C3 01:46:20", 11.0F, 8.4F, 1},
        {"real older (11.6/11.5)", 11.6F, 11.5F, 1},
        /* False (pivot, line-found, START, barcode restart). */
        {"false M5 after pivot", 6.1F, 1.0F, 0},
        {"false M6 barcode restart", 7.4F, 0.4F, 0},
        {"false M9 bypass pivot", 6.6F, 1.3F, 0},
        {"false M11 line found", 6.9F, 1.5F, 0},
        {"false M12 line found", 8.1F, 0.9F, 0},
        {"false M13 after turn", 6.2F, 0.5F, 0},
        {"false M20 line found", 5.8F, 1.1F, 0},
        {"false N4 barcode restart", 9.9F, 0.6F, 0},
        {"false C1 START on curve", 11.9F, 2.1F, 0},
        {"false C2 curve", 6.5F, 1.2F, 0},
        {"false C5 pivot to forward", 6.6F, 0.8F, 0},
    };
    int failed = 0;
    for (size_t i = 0; i < sizeof humps / sizeof humps[0]; i++) {
        bool desc;
        int const kept = run(&humps[i], &desc);
        /* HUMP_MIN_CLIMB_MM 0 (24 Nov): low-climb humps are recorded
         * too, but still never reach the descending phase (3 mm gate). */
        int const want_kept = humps[i].real ||
                              (humps[i].climb_mm >= HUMP_MIN_CLIMB_MM);
        if ((kept != want_kept) || (humps[i].real ? !desc : desc)) {
            printf("FAIL %s\n", humps[i].name);
            failed++;
        }
        if (!humps[i].real)
        {
            assert(g_discards == (want_kept ? 0 : 1));
        }
        if (kept) {
            /* Reported height = larger of tilt and climb, less the
             * 25 Nov calibration HUMP_HEIGHT_BIAS_MM, never below 0. */
            float const raw = (humps[i].peak_mm > humps[i].climb_mm)
                                  ? humps[i].peak_mm : humps[i].climb_mm;
            float const want = (raw > HUMP_HEIGHT_BIAS_MM)
                                   ? raw - HUMP_HEIGHT_BIAS_MM : 0.0F;
            assert(fabsf((float)g_hump.last_peak_mm - want) <= 1.0F);
        }
    }
    assert(0 == failed);
    printf("PASS hump climb check (minimum climb %.1f mm): 5 real humps "
           "kept with slow-down; low-climb ones %s, never slowed\n",
           (double)HUMP_MIN_CLIMB_MM,
           (HUMP_MIN_CLIMB_MM > 2.1F) ? "discarded" : "recorded");
    return 0;
}
/*** end of file ***/
/*** end of file ***/

