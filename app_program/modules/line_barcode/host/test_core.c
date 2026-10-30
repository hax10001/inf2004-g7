/* Host-only regression tests; firmware builds compile an empty unit. */
#ifdef BC_HOST_TEST

#include "bc_decode.h"
#include "bc_scan.h"
#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "pdf_fixtures.h"

static unsigned g_checks;
static unsigned g_failures;

#define CHECK(expression) do { \
    g_checks++; \
    if (!(expression)) { \
        printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #expression); \
        g_failures++; \
    } \
} while (0)

/* Independent reference: ZXing-C++ ODCode39Reader.cpp, CHARACTER_ENCODINGS.
 * These test constants are not read from the production lookup table.
 */
static uint16_t const g_reference[26] = {
    0x109, 0x049, 0x148, 0x019, 0x118, 0x058, 0x00d,
    0x10c, 0x04c, 0x01c, 0x103, 0x043, 0x142, 0x013,
    0x112, 0x052, 0x007, 0x106, 0x046, 0x016, 0x181,
    0x0c1, 0x1c0, 0x091, 0x190, 0x0d0
};

typedef struct {
    bc_scan_t scanner;
    uint64_t travel;
    uint32_t now;
    uint32_t left_origin;
    uint32_t right_origin;
    unsigned events;
} simulation_t;

static uint32_t frame_width(uint32_t const runs[BC_RUN_COUNT])
{
    uint32_t width = 0;
    for (unsigned i = 0; i < BC_RUN_COUNT; ++i) {
        width += runs[i];
    }
    return width;
}

static void make_frame(uint16_t data, uint32_t narrow, uint32_t wide,
                       uint32_t runs[BC_RUN_COUNT])
{
    uint16_t const symbols[3] = {0x094, data, 0x094};
    for (unsigned s = 0; s < 3; ++s) {
        for (unsigned bit = 0; bit < 9; ++bit) {
            runs[s * 10 + bit] =
                (symbols[s] & (1U << (8U - bit))) ? wide : narrow;
        }
    }
    runs[9] = narrow;
    runs[19] = narrow;
}

static uint32_t surface(uint32_t const runs[BC_RUN_COUNT], uint32_t lead,
                        uint64_t position, uint32_t black)
{
    if (position < lead) {
        return 1U - black;
    }
    position -= lead;
    for (unsigned i = 0; i < BC_RUN_COUNT; ++i) {
        if (position < runs[i]) {
            return (i & 1U) ? 1U - black : black;
        }
        position -= runs[i];
    }
    return 1U - black;
}

static bc_scan_config_t config(void)
{
    bc_scan_config_t result = {
        .black_level = 1,
        .left_sign = 1,
        .right_sign = -1,
        .max_gap_usec = 15000,
        .timeout_usec = 60000000,
        .stop_usec = 200000,
        .max_skew_units = 2
    };
    return result;
}

static void start_simulation(simulation_t *s, uint32_t now,
                              uint32_t left, uint32_t right, uint32_t black)
{
    bc_scan_config_t c = config();
    memset(s, 0, sizeof(*s));
    c.black_level = black;
    CHECK(bc_scan_init(&s->scanner, &c));
    s->now = now;
    s->left_origin = left;
    s->right_origin = right;
    CHECK(!bc_scan_feed(&s->scanner, 1U - black, (int32_t)left,
                       (int32_t)right, now));
}

static void sample(simulation_t *s, uint32_t raw, uint32_t step,
                    uint32_t elapsed)
{
    s->travel += step;
    s->now += elapsed;
    uint32_t const ticks = (uint32_t)(s->travel / 2U);
    if (bc_scan_feed(&s->scanner, raw,
        (int32_t)(s->left_origin + ticks),
        (int32_t)(s->right_origin - ticks), s->now)) {
        s->events++;
    }
}

static void scan_to(simulation_t *s, uint32_t const runs[BC_RUN_COUNT],
                     uint32_t lead, uint64_t end, unsigned speed,
                     bool noise)
{
    unsigned index = 0;
    while (s->travel < end) {
        uint32_t const step = (speed == 0) ? 4U :
                              (speed == 1) ? 8U : 2U * (index % 3U + 1U);
        uint64_t const position = s->travel + step;
        uint32_t raw = surface(runs, lead, position,
                               s->scanner.config.black_level);
        if (noise && ((position == 200) || (position == lead + 20))) {
            raw = 1U - raw;
        }
        sample(s, raw, step, 10000);
        index++;
    }
}

static void test_pdf_fixtures(void)
{
    for (unsigned i = 0; i < 4; ++i) {
        bc_decoded_t result = {0};
        CHECK(bc_decode(g_pdf_runs[i], 100, &result) == BC_OK);
        CHECK(result.letter == g_pdf_letters[i]);
        CHECK(result.width_units == frame_width(g_pdf_runs[i]));

        /* Scale the measured PDF widths to plausible encoder units;
         * preserve the actual independently extracted pattern/size ratio.
         */
        uint32_t measured[BC_RUN_COUNT];
        for (unsigned r = 0; r < BC_RUN_COUNT; ++r) {
            measured[r] = (g_pdf_runs[i][r] + 12U) / 25U;
        }
        uint32_t const narrow = (g_pdf_runs[i][0] + 12U) / 25U;
        simulation_t s;
        start_simulation(&s, 100000, 0, 0, 1);
        uint32_t const lead = narrow * 12;
        scan_to(&s, measured, lead,
                lead + frame_width(measured) + narrow * 12, 0, false);
        CHECK(s.events == 1);
        CHECK(s.scanner.state == BC_COMPLETE);
        CHECK(s.scanner.decoded.letter == g_pdf_letters[i]);

        /* PDF artwork alone has only 0.727*n preceding white. External
         * white approach/departure is required for a standard quiet zone.
         */
        start_simulation(&s, 100000, 0, 0, 1);
        uint32_t const short_lead = narrow * 727 / 1000;
        scan_to(&s, measured, short_lead,
                short_lead + frame_width(measured) + narrow * 12,
                0, false);
        CHECK(s.events == 0);
        CHECK(s.scanner.status == BC_ERR_QUIET);
    }
}

static void test_alphabet_and_scale(void)
{
    uint32_t const scales[] = {40, 100, 1000};
    for (unsigned letter = 0; letter < 26; ++letter) {
        for (unsigned scale = 0; scale < 3; ++scale) {
            for (unsigned ratio = 2; ratio <= 3; ++ratio) {
                uint32_t runs[BC_RUN_COUNT];
                bc_decoded_t decoded = {0};
                uint32_t const n = scales[scale];
                make_frame(g_reference[letter], n, n * ratio, runs);
                CHECK(bc_decode(runs, n / 10, &decoded) == BC_OK);
                CHECK(decoded.letter == (char)('A' + letter));
                CHECK(decoded.narrow_units == n);
                CHECK(decoded.width_units == n * (20 + 9 * ratio));

                for (unsigned i = 0; i < BC_RUN_COUNT; ++i) {
                    int const percent = ((int)(i % 5U) - 2) * 4;
                    runs[i] = (uint32_t)((int64_t)runs[i] *
                                         (100 + percent) / 100);
                }
                CHECK(bc_decode(runs, n / 10, &decoded) == BC_OK);
                CHECK(decoded.letter == (char)('A' + letter));
            }
        }
    }
}

static void test_malformed_widths(void)
{
    uint32_t runs[BC_RUN_COUNT];
    bc_decoded_t decoded = {'?', 123, 456};
    CHECK(bc_decode(NULL, 1, &decoded) == BC_ERR_PATTERN);
    make_frame(g_reference[0], 40, 120, runs);
    CHECK(bc_decode(runs, 1, NULL) == BC_ERR_PATTERN);
    CHECK(bc_decode(runs, 10, &decoded) == BC_ERR_RESOLUTION);
    CHECK(decoded.letter == '?');
    CHECK(decoded.narrow_units == 123);
    CHECK(decoded.width_units == 456);

    runs[0] = 120;
    runs[1] = 40;
    CHECK(bc_decode(runs, 4, &decoded) == BC_ERR_PATTERN);
    make_frame(g_reference[0], 40, 120, runs);
    runs[20] = 120;
    runs[21] = 40;
    CHECK(bc_decode(runs, 4, &decoded) == BC_ERR_PATTERN);
    make_frame(g_reference[0], 40, 120, runs);
    runs[9] = 120;
    runs[10] = 40;
    CHECK(bc_decode(runs, 4, &decoded) == BC_ERR_PATTERN);
    make_frame(0x034, 40, 120, runs); /* Digit zero: outside A-Z contract. */
    CHECK(bc_decode(runs, 4, &decoded) == BC_ERR_PATTERN);
    make_frame(g_reference[0], 40, 60, runs);
    CHECK(bc_decode(runs, 2, &decoded) == BC_ERR_PATTERN);
    make_frame(g_reference[0], 40, 160, runs);
    CHECK(bc_decode(runs, 2, &decoded) == BC_ERR_PATTERN);
    make_frame(g_reference[0], 40, 120, runs);
    runs[5] = 0;
    CHECK(bc_decode(runs, 4, &decoded) == BC_ERR_PATTERN);
    for (unsigned i = 0; i < BC_RUN_COUNT; ++i) {
        runs[i] = UINT32_MAX;
    }
    CHECK(bc_decode(runs, 4, &decoded) == BC_ERR_OVERFLOW);
    make_frame(g_reference[0], 40, 120, runs);
    for (unsigned i = 0; i < BC_RUN_COUNT / 2; ++i) {
        uint32_t const save = runs[i];
        runs[i] = runs[BC_RUN_COUNT - i - 1];
        runs[BC_RUN_COUNT - i - 1] = save;
    }
    CHECK(bc_decode(runs, 4, &decoded) == BC_ERR_PATTERN);
}

static void test_sampled_alphabet(void)
{
    for (unsigned letter = 0; letter < 26; ++letter) {
        for (unsigned speed = 0; speed < 3; ++speed) {
            simulation_t s;
            uint32_t runs[BC_RUN_COUNT];
            make_frame(g_reference[letter], 40, 120, runs);
            start_simulation(&s, 0xfffff000U, 0x7ffffff0U,
                              0x80000010U, letter & 1U);
            scan_to(&s, runs, 480, 480 + frame_width(runs) + 480,
                    speed, false);
            CHECK(s.events == 1);
            CHECK(s.scanner.state == BC_COMPLETE);
            CHECK(s.scanner.decoded.letter == (char)('A' + letter));
            CHECK(s.scanner.decoded.width_units >= frame_width(runs) - 8);
            CHECK(s.scanner.decoded.width_units <= frame_width(runs) + 8);
            for (unsigned hold = 0; hold < 100; ++hold) {
                sample(&s, 1U - s.scanner.config.black_level, 0, 10000);
            }
            CHECK(s.events == 1); /* Result remains held until reset. */
        }
    }
}

static void test_debounce_and_rearm(void)
{
    simulation_t s;
    uint32_t runs[BC_RUN_COUNT];
    make_frame(g_reference[0], 40, 120, runs);
    start_simulation(&s, 100000, 0, 0, 1);
    scan_to(&s, runs, 400, 400 + frame_width(runs) + 420, 0, true);
    CHECK(s.events == 1);
    CHECK(s.scanner.decoded.letter == 'A');
    bc_scan_reset(&s.scanner);
    CHECK(s.scanner.state == BC_WAIT_WHITE);
    for (unsigned hold = 0; hold < 100; ++hold) {
        sample(&s, 0, 0, 10000);
    }
    CHECK(s.events == 1);

    /* After consuming a result, a second scan requires new white travel. */
    uint64_t const start = s.travel;
    while (s.travel < start + 400 + frame_width(runs) + 420) {
        uint64_t const relative = s.travel + 4 - start;
        sample(&s, surface(runs, 400, relative, 1), 4, 10000);
    }
    CHECK(s.events == 2);
    CHECK(s.scanner.decoded.letter == 'A');

    /* Starting directly on black cannot borrow the old frame's quiet zone. */
    bc_scan_reset(&s.scanner);
    sample(&s, 1, 0, 10000);
    sample(&s, 1, 0, 10000);
    CHECK(s.scanner.state == BC_WAIT_WHITE);
    CHECK(s.events == 2);
}

static void prepare_active(simulation_t *s, uint32_t runs[BC_RUN_COUNT])
{
    make_frame(g_reference[0], 40, 120, runs);
    start_simulation(s, 100000, 0, 0, 1);
    scan_to(s, runs, 400, 408, 0, false);
    CHECK(s->scanner.state == BC_SCANNING);
}

static void test_capture_faults(void)
{
    simulation_t s;
    uint32_t runs[BC_RUN_COUNT];
    prepare_active(&s, runs);
    sample(&s, 1, 4, 15001);
    CHECK(s.scanner.status == BC_ERR_SAMPLE);
    CHECK(s.events == 0);

    prepare_active(&s, runs);
    CHECK(!bc_scan_feed(&s.scanner, 1, s.scanner.last_left - 1,
                       s.scanner.last_right, s.now + 10000));
    CHECK(s.scanner.status == BC_ERR_REVERSE);

    prepare_active(&s, runs);
    CHECK(!bc_scan_feed(&s.scanner, 1, s.scanner.last_left + 8,
                       s.scanner.last_right, s.now + 10000));
    CHECK(s.scanner.status == BC_ERR_SKEW);

    prepare_active(&s, runs);
    for (unsigned i = 0; i < 22; ++i) {
        sample(&s, 1, 0, 10000);
    }
    CHECK(s.scanner.status == BC_ERR_STOPPED);
    CHECK(s.events == 0);

    prepare_active(&s, runs);
    s.scanner.config.timeout_usec = 300000;
    for (unsigned i = 0; i < 32; ++i) {
        sample(&s, 1, 4, 10000);
    }
    CHECK(s.scanner.status == BC_ERR_TIMEOUT);
    CHECK(s.events == 0);

    /* An incomplete prefix followed by stationary white never publishes. */
    prepare_active(&s, runs);
    scan_to(&s, runs, 400, 1000, 0, false);
    for (unsigned i = 0; i < 24; ++i) {
        sample(&s, 0, 0, 10000);
    }
    CHECK(s.events == 0);
    CHECK(s.scanner.status == BC_ERR_STOPPED);

    /* A 30th run before sufficient trailing quiet is a bounded overflow. */
    prepare_active(&s, runs);
    scan_to(&s, runs, 400, 400 + frame_width(runs) + 40, 0, false);
    CHECK(s.scanner.state == BC_TRAILING_WHITE);
    sample(&s, 1, 4, 10000);
    sample(&s, 1, 4, 10000);
    CHECK(s.scanner.status == BC_ERR_OVERFLOW);
    CHECK(s.scanner.run_count == BC_RUN_COUNT);
    CHECK(s.events == 0);

    /* Distance accumulation is bounded before writing a uint32_t run. */
    prepare_active(&s, runs);
    sample(&s, 1, UINT32_C(2147483648), 10000);
    sample(&s, 1, UINT32_C(2147483648), 10000);
    CHECK(s.scanner.status == BC_ERR_OVERFLOW);
    CHECK(s.events == 0);

    bc_scan_reset(&s.scanner);
    CHECK(!bc_scan_feed(&s.scanner, 2, 0, 0, 100));
    CHECK(s.scanner.status == BC_ERR_SAMPLE);
    CHECK(!s.scanner.b_seeded);
}

static void test_resolution_at_start_and_finish(void)
{
    simulation_t s;
    uint32_t runs[BC_RUN_COUNT];
    make_frame(g_reference[0], 40, 120, runs);

    /* First candidate's displacement must survive the second sample. */
    start_simulation(&s, 100000, 0, 0, 1);
    scan_to(&s, runs, 400, 360, 0, false);
    sample(&s, 1, 40, 10000);
    scan_to(&s, runs, 400, 400 + frame_width(runs) + 420, 0, false);
    CHECK(s.events == 0);
    CHECK(s.scanner.status == BC_ERR_RESOLUTION);

    /* Sparse leading-white samples could hide bars in the quiet zone. */
    start_simulation(&s, 100000, 0, 0, 1);
    sample(&s, 0, 200, 10000);
    scan_to(&s, runs, 400, 400 + frame_width(runs) + 420, 0, false);
    CHECK(s.events == 0);
    CHECK(s.scanner.status == BC_ERR_RESOLUTION);

    /* Jumping across the trailing white might hide extra narrow bars. */
    prepare_active(&s, runs);
    scan_to(&s, runs, 400, 400 + frame_width(runs) + 40, 0, false);
    CHECK(s.scanner.state == BC_TRAILING_WHITE);
    sample(&s, 0, 400, 10000);
    CHECK(s.events == 0);
    CHECK(s.scanner.status == BC_ERR_RESOLUTION);

    /* A pending black candidate suppresses completion until confirmed. */
    prepare_active(&s, runs);
    scan_to(&s, runs, 400, 400 + frame_width(runs) + 396, 0, false);
    CHECK(s.scanner.state == BC_TRAILING_WHITE);
    sample(&s, 1, 4, 10000);
    CHECK(s.events == 0);
    sample(&s, 1, 400, 10000);
    CHECK(s.events == 0);
    CHECK(s.scanner.status == BC_ERR_RESOLUTION);
}

static void test_invalid_configuration(void)
{
    bc_scan_t scanner;
    bc_scan_config_t c = config();
    CHECK(!bc_scan_init(NULL, &c));
    CHECK(!bc_scan_init(&scanner, NULL));
    c.black_level = 2;
    CHECK(!bc_scan_init(&scanner, &c));
    c = config();
    c.left_sign = 0;
    CHECK(!bc_scan_init(&scanner, &c));
    c = config();
    c.right_sign = 2;
    CHECK(!bc_scan_init(&scanner, &c));
    c = config();
    c.max_gap_usec = 0;
    CHECK(!bc_scan_init(&scanner, &c));
    c = config();
    c.stop_usec = c.max_gap_usec;
    CHECK(!bc_scan_init(&scanner, &c));
    c = config();
    c.timeout_usec = c.stop_usec;
    CHECK(!bc_scan_init(&scanner, &c));
    c = config();
    c.timeout_usec = UINT32_MAX;
    CHECK(!bc_scan_init(&scanner, &c));
    c = config();
    c.max_skew_percent = 100;
    CHECK(!bc_scan_init(&scanner, &c));
}

static void test_pid_corrections(void)
{
    /* Both wheels remain forward while the steering correction alternates.
     * The reference surface follows summed wheel travel. This models the
     * distance estimator; it does not qualify an offset sensor on hardware.
     */
    for (unsigned letter = 0; letter < 26; ++letter) {
        for (unsigned scale = 1; scale <= 2; ++scale) {
            uint32_t runs[BC_RUN_COUNT];
            uint32_t const narrow = 120U * scale;
            uint32_t const lead = 12U * narrow;
            bc_scan_config_t c = config();
            bc_scan_t scanner;
            c.max_skew_units = 6;
            c.max_skew_percent = 60;
            make_frame(g_reference[letter], narrow, 3U * narrow, runs);
            CHECK(bc_scan_init(&scanner, &c));
            uint32_t now = 100000;
            uint32_t left = 0, right = 0, position = 0, events = 0;
            CHECK(!bc_scan_feed(&scanner, 0, 0, 0, now));
            uint32_t const end = lead + frame_width(runs) + 12U * narrow;
            for (unsigned i = 0; position < end; ++i) {
                bool const correct_left = ((i / 5U) % 2U) != 0U;
                left += correct_left ? 3U : 12U;
                right -= correct_left ? 12U : 3U;
                position += 15U;
                now += 10000U;
                if (bc_scan_feed(&scanner, surface(runs, lead, position, 1),
                                 (int32_t)left, (int32_t)right, now)) {
                    events++;
                }
            }
            CHECK(events == 1);
            CHECK(scanner.state == BC_COMPLETE);
            CHECK(scanner.status == BC_OK);
            CHECK(scanner.decoded.letter == (char)('A' + letter));
        }
    }
}

static void test_pivot_windows(void)
{
    unsigned const percentages[] = {0, 60, 90, 99};
    for (unsigned p = 0; p < 4; ++p) {
        bc_scan_config_t c = config();
        bc_scan_t scanner;
        c.max_skew_units = 6;
        c.max_skew_percent = percentages[p];
        CHECK(bc_scan_init(&scanner, &c));
        uint32_t now = 100000;
        CHECK(!bc_scan_feed(&scanner, 0, 0, 0, now));
        unsigned distance;
        for (distance = 1; distance <= 1000; ++distance) {
            now += 10000U;
            CHECK(!bc_scan_feed(&scanner, 0, (int32_t)distance, 0, now));
            if (scanner.status == BC_ERR_SKEW) {
                break;
            }
        }
        CHECK(distance <= 1000);
        CHECK(scanner.status == BC_ERR_SKEW);
        CHECK(!scanner.b_seeded);
        CHECK(scanner.motion_left_units == 0);
        CHECK(scanner.motion_right_units == 0);
    }

    /* Opposite pulse timing on consecutive samples is not a sustained turn. */
    bc_scan_config_t c = config();
    bc_scan_t scanner;
    c.max_skew_units = 6;
    c.max_skew_percent = 60;
    CHECK(bc_scan_init(&scanner, &c));
    uint32_t left = 0, right = 0, now = 100000;
    CHECK(!bc_scan_feed(&scanner, 0, 0, 0, now));
    for (unsigned i = 0; i < 100; ++i) {
        if ((i % 2U) == 0U) { left += 10; } else { right -= 10; }
        now += 10000;
        CHECK(!bc_scan_feed(&scanner, 0, (int32_t)left,
                           (int32_t)right, now));
        CHECK(scanner.status == BC_OK);
    }

    /* A partial window must be validated before final publication too. */
    CHECK(bc_scan_init(&scanner, &c));
    CHECK(!bc_scan_feed(&scanner, 0, 0, 0, now));
    make_frame(g_reference[0], 40, 120, scanner.runs);
    scanner.state = BC_TRAILING_WHITE;
    scanner.quiet_units = 400;
    scanner.position = 380;
    scanner.start_usec = now;
    bool published = false;
    for (unsigned i = 1; i <= 20; ++i) {
        now += 10000;
        published |= bc_scan_feed(&scanner, 0, (int32_t)i, 0, now);
    }
    CHECK(!published);
    CHECK(scanner.status == BC_ERR_SKEW);
}

static bc_decode_report_t check_report_failure(
    uint32_t const runs[BC_RUN_COUNT], uint32_t step, bc_status_t status,
    bc_check_t check, uint32_t failed_index, bool metrics_valid)
{
    bc_decoded_t decoded = {'?', 123, 456};
    bc_decode_report_t report;
    memset(&report, 0xa5, sizeof(report));
    CHECK(bc_decode_inspect(runs, step, &decoded, &report) == status);
    CHECK(report.check == check);
    CHECK(report.failed_index == failed_index);
    CHECK(report.b_metrics_valid == metrics_valid);
    CHECK(report.max_step_units == step);
    CHECK(decoded.letter == '?');
    CHECK(decoded.narrow_units == 123);
    CHECK(decoded.width_units == 456);
    CHECK(bc_decode_inspect(runs, step, &decoded, NULL) == status);
    CHECK(bc_decode(runs, step, &decoded) == status);
    CHECK(decoded.letter == '?');
    CHECK(decoded.narrow_units == 123);
    CHECK(decoded.width_units == 456);
    return report;
}

static void test_decode_resolution_reports(void)
{
    uint32_t runs[BC_RUN_COUNT];
    bc_decoded_t decoded = {0};
    bc_decode_report_t report;

    /* Both errors return BC_ERR_RESOLUTION, but need different explanations. */
    make_frame(g_reference[0], 40, 120, runs);
    report = check_report_failure(runs, 10, BC_ERR_RESOLUTION,
                                  BC_CHECK_MIN_WIDTH, 0, true);
    CHECK(report.min_width == 40);
    CHECK(report.narrow_max == 40);
    CHECK(report.wide_min == 120);
    CHECK(report.required_min == 42);
    CHECK(report.required_separation == 22);
    CHECK(report.narrow_sum == 800);
    CHECK(report.wide_sum == 1080);
    CHECK(report.start_pattern == 0x094);
    CHECK(report.letter_pattern == 0x109);
    CHECK(report.stop_pattern == 0x094);

    /* Minimum width is inclusive: 42 passes, while 41 does not. */
    make_frame(g_reference[0], 42, 126, runs);
    CHECK(bc_decode_inspect(runs, 10, &decoded, &report) == BC_OK);
    CHECK(report.check == BC_CHECK_OK);
    CHECK(report.failed_index == BC_RUN_COUNT);
    CHECK(report.min_width == report.required_min);
    CHECK(decoded.letter == 'A');
    CHECK(decoded.width_units == 1974);
    CHECK(bc_decode_inspect(runs, 10, &decoded, NULL) == BC_OK);
    make_frame(g_reference[0], 41, 123, runs);
    (void)check_report_failure(runs, 10, BC_ERR_RESOLUTION,
                               BC_CHECK_MIN_WIDTH, 0, true);

    /* An unusual minimum must identify its original, not sorted, index. */
    make_frame(g_reference[0], 100, 300, runs);
    runs[16] = 41;
    report = check_report_failure(runs, 10, BC_ERR_RESOLUTION,
                                  BC_CHECK_MIN_WIDTH, 16, true);
    CHECK(report.min_width == 41);
    CHECK(report.narrow_max == 100);
    CHECK(report.wide_min == 300);
    CHECK(report.narrow_sum == 1941);
    CHECK(report.wide_sum == 2700);

    /* One short wide bar makes the two clusters touch the strict boundary. */
    make_frame(g_reference[0], 100, 300, runs);
    runs[21] = 122;
    report = check_report_failure(runs, 10, BC_ERR_RESOLUTION,
                                  BC_CHECK_SEPARATION, BC_RUN_COUNT, true);
    CHECK(report.min_width == 100);
    CHECK(report.narrow_max == 100);
    CHECK(report.wide_min == 122);
    CHECK(report.required_min == 42);
    CHECK(report.required_separation == 22);
    CHECK(report.wide_min - report.narrow_max == report.required_separation);
    CHECK(report.narrow_sum == 2000);
    CHECK(report.wide_sum == 2522);

    /* One unit more clears separation, revealing the later tolerance error. */
    runs[21] = 123;
    report = check_report_failure(runs, 10, BC_ERR_PATTERN,
                                  BC_CHECK_WIDE_TOLERANCE, 21, true);
    CHECK(report.wide_min - report.narrow_max == 23);
    CHECK(report.wide_sum == 2523);

    make_frame(g_reference[0], 100, 300, runs);
    report = check_report_failure(runs, UINT32_MAX, BC_ERR_RESOLUTION,
                                  BC_CHECK_MIN_WIDTH, 0, true);
    CHECK(report.required_min == UINT64_C(17179869182));
    CHECK(report.required_separation == UINT64_C(8589934592));
}

static void test_decode_pattern_reports(void)
{
    uint32_t runs[BC_RUN_COUNT];
    bc_decode_report_t report;
    bc_decoded_t decoded = {0};

    report = check_report_failure(NULL, 4, BC_ERR_PATTERN,
                                  BC_CHECK_INPUT, BC_RUN_COUNT, false);
    CHECK(report.min_width == 0);
    CHECK(report.narrow_sum == 0);
    CHECK(report.start_pattern == 0);
    make_frame(g_reference[0], 100, 300, runs);
    CHECK(bc_decode_inspect(runs, 4, NULL, &report) == BC_ERR_PATTERN);
    CHECK(report.check == BC_CHECK_INPUT);
    CHECK(!report.b_metrics_valid);
    CHECK(report.failed_index == BC_RUN_COUNT);
    CHECK(bc_decode_inspect(NULL, 4, NULL, NULL) == BC_ERR_PATTERN);
    runs[17] = 0;
    (void)check_report_failure(runs, 4, BC_ERR_PATTERN,
                               BC_CHECK_ZERO_WIDTH, 17, false);
    for (unsigned i = 0; i < BC_RUN_COUNT; ++i) {
        runs[i] = UINT32_MAX;
    }
    (void)check_report_failure(runs, 4, BC_ERR_OVERFLOW,
                               BC_CHECK_TOTAL_OVERFLOW, BC_RUN_COUNT, false);

    /* Width-ratio endpoints are inclusive and use no rounded means. */
    make_frame(g_reference[0], 100, 180, runs);
    CHECK(bc_decode_inspect(runs, 10, &decoded, &report) == BC_OK);
    CHECK(report.check == BC_CHECK_OK);
    make_frame(g_reference[0], 100, 179, runs);
    (void)check_report_failure(runs, 10, BC_ERR_PATTERN,
                               BC_CHECK_WIDTH_RATIO, BC_RUN_COUNT, true);
    make_frame(g_reference[0], 100, 350, runs);
    CHECK(bc_decode_inspect(runs, 10, &decoded, &report) == BC_OK);
    CHECK(report.check == BC_CHECK_OK);
    make_frame(g_reference[0], 100, 351, runs);
    (void)check_report_failure(runs, 10, BC_ERR_PATTERN,
                               BC_CHECK_WIDTH_RATIO, BC_RUN_COUNT, true);

    /* Preserve the cluster mean while probing +/-30% tolerance boundaries. */
    make_frame(g_reference[0], 100, 300, runs);
    runs[17] = 70;
    runs[2] = 130;
    CHECK(bc_decode_inspect(runs, 10, &decoded, &report) == BC_OK);
    CHECK(report.narrow_sum == 2000);
    runs[17] = 69;
    runs[2] = 131;
    (void)check_report_failure(runs, 10, BC_ERR_PATTERN,
                               BC_CHECK_NARROW_TOLERANCE, 17, true);
    make_frame(g_reference[0], 100, 300, runs);
    runs[27] = 131;
    runs[2] = 84;
    runs[3] = 85;
    (void)check_report_failure(runs, 10, BC_ERR_PATTERN,
                               BC_CHECK_NARROW_TOLERANCE, 27, true);
    make_frame(g_reference[0], 100, 300, runs);
    runs[1] = 210;
    runs[4] = 390;
    CHECK(bc_decode_inspect(runs, 10, &decoded, &report) == BC_OK);
    CHECK(report.wide_sum == 2700);
    runs[1] = 209;
    runs[4] = 391;
    (void)check_report_failure(runs, 10, BC_ERR_PATTERN,
                               BC_CHECK_WIDE_TOLERANCE, 1, true);
    make_frame(g_reference[0], 100, 300, runs);
    runs[26] = 391;
    runs[1] = 254;
    runs[4] = 255;
    (void)check_report_failure(runs, 10, BC_ERR_PATTERN,
                               BC_CHECK_WIDE_TOLERANCE, 26, true);

    /* Swaps preserve all widths so failure really concerns gap/symbol shape. */
    make_frame(g_reference[0], 100, 300, runs);
    runs[9] = 300;
    runs[10] = 100;
    (void)check_report_failure(runs, 10, BC_ERR_PATTERN,
                               BC_CHECK_GAP_FIRST, 9, true);
    make_frame(g_reference[0], 100, 300, runs);
    runs[19] = 300;
    runs[10] = 100;
    (void)check_report_failure(runs, 10, BC_ERR_PATTERN,
                               BC_CHECK_GAP_SECOND, 19, true);
    make_frame(g_reference[0], 100, 300, runs);
    runs[0] = 300;
    runs[1] = 100;
    report = check_report_failure(runs, 10, BC_ERR_PATTERN,
                                  BC_CHECK_START_SYMBOL, 0, true);
    CHECK(report.start_pattern == 0x114);
    CHECK(report.letter_pattern == 0x109);
    CHECK(report.stop_pattern == 0x094);
    make_frame(g_reference[0], 100, 300, runs);
    runs[20] = 300;
    runs[21] = 100;
    report = check_report_failure(runs, 10, BC_ERR_PATTERN,
                                  BC_CHECK_STOP_SYMBOL, 20, true);
    CHECK(report.stop_pattern == 0x114);
    make_frame(0x034, 100, 300, runs);
    report = check_report_failure(runs, 10, BC_ERR_PATTERN,
                                  BC_CHECK_LETTER, 10, true);
    CHECK(report.letter_pattern == 0x034);

    char const * const names[] = {
        "NOT_RUN", "OK", "INPUT", "ZERO_WIDTH", "TOTAL_OVERFLOW",
        "MIN_WIDTH", "SEPARATION", "WIDTH_RATIO", "NARROW_TOLERANCE",
        "WIDE_TOLERANCE", "GAP_FIRST", "GAP_SECOND", "START_SYMBOL",
        "STOP_SYMBOL", "LETTER"};
    for (unsigned i = 0; i < sizeof(names) / sizeof(names[0]); ++i) {
        CHECK(strcmp(bc_check_name((bc_check_t)i), names[i]) == 0);
    }
    CHECK(strcmp(bc_check_name((bc_check_t)-1), "UNKNOWN") == 0);
}

static void test_approach_resolution_history(void)
{
    simulation_t s;
    uint32_t runs[BC_RUN_COUNT];
    make_frame(g_reference[0], 20, 60, runs);
    start_simulation(&s, 100000, 0, 0, 1);
    sample(&s, 0, 10, 10000);
    uint32_t const lead = 240;
    uint32_t const end = lead + frame_width(runs) + 240;
    while (s.travel < end) {
        sample(&s, surface(runs, lead, s.travel + 2, 1), 2, 10000);
    }

    /* Intentional conservatism: early white movement remains in the bound
     * even after more than ten narrow widths of accurately sampled white.
     * This is not an off-by-one error in the subsequently captured bars.
     */
    CHECK(s.events == 0);
    CHECK(s.scanner.status == BC_ERR_RESOLUTION);
    CHECK(s.scanner.run_count == BC_RUN_COUNT);
    CHECK(s.scanner.max_step_units == 10);
    for (unsigned i = 0; i < BC_RUN_COUNT; ++i) {
        CHECK(s.scanner.runs[i] == runs[i]);
    }
    bc_decoded_t decoded = {0};
    bc_decode_report_t report;
    CHECK(bc_decode_inspect(s.scanner.runs, 10, &decoded, &report) ==
          BC_ERR_RESOLUTION);
    CHECK(report.check == BC_CHECK_MIN_WIDTH);
    CHECK(report.min_width == 20);
    CHECK(report.required_min == 42);
    CHECK(bc_decode(s.scanner.runs, 2, &decoded) == BC_OK);
    CHECK(decoded.letter == 'A');
}

static void test_edge_backdating_with_speed_changes(void)
{
    simulation_t s;
    uint32_t runs[BC_RUN_COUNT];
    make_frame(g_reference[0], 40, 120, runs);
    start_simulation(&s, 100000, 0, 0, 1);
    while (s.travel < 396) {
        sample(&s, 0, 4, 10000);
    }
    sample(&s, 1, 4, 10000); /* First black at position 400. */
    uint32_t const first_black_usec = s.now;
    sample(&s, 1, 8, 10000); /* Confirm at 408, after accelerating. */
    CHECK(s.scanner.run_start == 400);
    CHECK(s.scanner.start_usec == first_black_usec);
    while (s.travel < 436) {
        sample(&s, 1, 4, 10000);
    }
    sample(&s, 0, 4, 10000); /* First white at 440. */
    sample(&s, 0, 2, 10000); /* Confirm at 442, after slowing down. */
    CHECK(s.scanner.run_count == 1);
    CHECK(s.scanner.runs[0] == 40); /* Not 442 - 408 == 34. */
    CHECK(s.scanner.run_start == 440);
    while (s.travel < 558) {
        sample(&s, 0, 2, 10000);
    }
    sample(&s, 1, 2, 10000);
    sample(&s, 1, 6, 10000);
    CHECK(s.scanner.run_count == 2);
    CHECK(s.scanner.runs[1] == 120);
    CHECK(s.scanner.run_start == 560);
    uint32_t const end = 400 + frame_width(runs) + 480;
    while (s.travel < end) {
        sample(&s, surface(runs, 400, s.travel + 2, 1), 2, 10000);
    }
    CHECK(s.events == 1);
    CHECK(s.scanner.state == BC_COMPLETE);
    CHECK(s.scanner.max_step_units == 8);
    CHECK(s.scanner.decoded.letter == 'A');
    for (unsigned i = 0; i < BC_RUN_COUNT; ++i) {
        CHECK(s.scanner.runs[i] == runs[i]);
    }
}

static void test_black_boot_and_single_line(void)
{
    simulation_t s = {0};
    bc_scan_config_t const c = config();
    CHECK(bc_scan_init(&s.scanner, &c));
    s.now = 100000;
    CHECK(!bc_scan_feed(&s.scanner, 1, 0, 0, s.now));
    for (unsigned i = 0; i < 100; ++i) {
        sample(&s, 1, 4, 10000);
    }
    CHECK(s.scanner.state == BC_WAIT_WHITE);
    CHECK(s.scanner.run_count == 0);
    CHECK(s.events == 0);
    for (unsigned i = 0; i < 100; ++i) {
        sample(&s, 0, 4, 10000);
    }
    CHECK(s.scanner.state == BC_WAIT_WHITE);
    CHECK(s.scanner.run_count == 0);
    CHECK(s.events == 0);

    start_simulation(&s, 100000, 0, 0, 1);
    while (s.travel < 396) {
        sample(&s, 0, 4, 10000);
    }
    for (unsigned i = 0; i < 10; ++i) {
        sample(&s, 1, 4, 10000);
    }
    for (unsigned i = 0; i < 500; ++i) {
        sample(&s, 0, 4, 10000);
    }
    CHECK(s.scanner.state == BC_SCANNING);
    CHECK(s.scanner.run_count == 1);
    CHECK(s.scanner.runs[0] == 40);
    CHECK(s.events == 0);
    for (unsigned i = 0; i < 22; ++i) {
        sample(&s, 0, 0, 10000);
    }
    CHECK(s.scanner.status == BC_ERR_STOPPED);
    CHECK(s.events == 0);
}

static void test_mapping_independence(void)
{
    /* The consumer owns action values. Changing its table is deliberately
     * not an input to acquisition or decoding. No robot action is executed.
     */
    unsigned consumer_mapping[26] = {0};
    uint32_t runs[BC_RUN_COUNT];
    bc_decoded_t before = {0};
    bc_decoded_t after = {0};
    make_frame(g_reference[25], 40, 120, runs);
    CHECK(bc_decode(runs, 4, &before) == BC_OK);
    CHECK(consumer_mapping[(unsigned)(before.letter - 'A')] == 0);
    consumer_mapping[25] = 7; /* Opaque consumer test value, not a mission. */
    CHECK(bc_decode(runs, 4, &after) == BC_OK);
    CHECK(after.letter == before.letter);
    CHECK(consumer_mapping[(unsigned)(after.letter - 'A')] == 7);
}

int main(void)
{
    test_pdf_fixtures();
    test_alphabet_and_scale();
    test_malformed_widths();
    test_sampled_alphabet();
    test_debounce_and_rearm();
    test_capture_faults();
    test_resolution_at_start_and_finish();
    test_invalid_configuration();
    test_pid_corrections();
    test_pivot_windows();
    test_decode_resolution_reports();
    test_decode_pattern_reports();
    test_approach_resolution_history();
    test_edge_backdating_with_speed_changes();
    test_black_boot_and_single_line();
    test_mapping_independence();
    printf("Barcode core: %u checks, %u failures\n", g_checks, g_failures);
    return g_failures ? 1 : 0;
}

#endif /* BC_HOST_TEST */
