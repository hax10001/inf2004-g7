/* Host replay of saved barcode logs through the firmware's bc_layout.c.
 *
 *   gcc -std=c99 -O2 -Wall -Wextra -Iapp_program/modules/barcode_scanning \
 *       tools/tests/barcode_layout.c \
 *       app_program/modules/barcode_scanning/bc_layout.c \
 *       app_program/modules/barcode_scanning/bc_code.c -o layout_replay
 *   ./layout_replay tools/barcode_data/2026-11-05/small_F_push.log ...
 *
 * Prints one line per pass: the expected letter from the file name
 * (size_LETTER_method.log), then what the firmware decoder decides.
 * Should agree with tools/barcode_replay.py (default combined mode).
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "bc_layout.h"

#define MAX_ROWS 8192

static unsigned g_units[MAX_ROWS];
static unsigned g_step[MAX_ROWS];
static int g_signal[MAX_ROWS];
static unsigned g_rows;

static void read_sample(uint32_t index, uint32_t *p_units,
                        uint32_t *p_left_step, int32_t *p_signal)
{
    *p_units = g_units[index];
    *p_left_step = g_step[index];
    *p_signal = g_signal[index];
}

static char expected(const char *path)
{
    const char *name = strrchr(path, '/');
    const char *alt = strrchr(path, '\\');
    if (alt && (!name || alt > name)) name = alt;
    name = name ? name + 1 : path;
    const char *u = strchr(name, '_');
    if (u && u[1] && u[2] == '_' && u[1] >= 'A' && u[1] <= 'Z') return u[1];
    if (u && u[1] && u[2] == '_' && u[1] >= 'a' && u[1] <= 'z')
        return (char)(u[1] - 32);
    return '?';
}

static void decode(const char *path, int pass, int has_left,
                   int *p_ok, int *p_wrong, int *p_total)
{
    bc_layout_result_t r;
    unsigned polls = 0;
    char want = expected(path);
    if (g_rows == 0) return;
    bc_layout_begin(g_rows, read_sample, (uint8_t)has_left);
    while (!bc_layout_poll()) ++polls;
    r = bc_layout_result();
    ++*p_total;
    if (r.valid) {
        char got = (char)('A' + r.letter);
        const char *mark = want == '?' ? "" : (got == want ? " OK" : " WRONG");
        if (want != '?') {
            if (got == want) ++*p_ok; else ++*p_wrong;
        }
        printf("%-24s #%-2d want=%c got=%c%-6s v=%u err=%.2f mod=%.1f "
               "gap=%.1f%% runner=%c bars=%u polls=%u\n", path, pass, want,
               got, mark, r.variant, r.err_centi / 100.0,
               r.module_x10 / 10.0, r.gap_permille / 10.0,
               r.runner < 26 ? 'A' + r.runner : '-', r.bars, polls);
    } else {
        printf("%-24s #%-2d want=%c got=-  (%s%s%c bars=%u err=%.2f) "
               "polls=%u\n", path, pass, want,
               bc_layout_reason_text(r.reason),
               r.other < 26 ? " vs " : "", r.other < 26 ? 'A' + r.other : ' ',
               r.bars, r.err_centi / 100.0, polls);
    }
}

int main(int argc, char **argv)
{
    int ok = 0, wrong = 0, total = 0;
    for (int f = 1; f < argc; ++f) {
        FILE *fp = fopen(argv[f], "r");
        char line[512];
        int pass = 0, has_left = 0, in_pass = 0;
        unsigned u0 = 0, prev_left = 0;
        if (!fp) { perror(argv[f]); return 1; }
        g_rows = 0;
        while (fgets(line, sizeof line, fp)) {
            char *raw = strstr(line, "[BC WAVE RAW] ");
            if (strstr(line, "[BC WAVE START]")) {
                if (in_pass && g_rows) decode(argv[f], pass, has_left,
                                              &ok, &wrong, &total);
                ++pass; in_pass = 1; g_rows = 0;
            }
            if (raw && in_pass && g_rows < MAX_ROWS) {
                unsigned v[9]; int n = 0; char *p = raw + 14;
                while (n < 9 && *p >= '0' && *p <= '9') {
                    v[n++] = (unsigned)strtoul(p, &p, 10);
                    if (*p == ',') ++p; else break;
                }
                if (n == 8 || n == 9) {
                    if (g_rows == 0) { u0 = v[3]; prev_left = n == 9 ? v[8] : 0; }
                    has_left = (n == 9);
                    g_units[g_rows] = v[3] - u0;
                    {
                        unsigned step = has_left ?
                            ((v[8] - prev_left) & 0xFFFFu) : 0u;
                        g_step[g_rows] = step > 15u ? 15u : step;
                        prev_left = has_left ? v[8] : 0u;
                    }
                    g_signal[g_rows] = (int)v[5] - (int)v[6];
                    ++g_rows;
                }
            }
            if (strstr(line, "[BC WAVE RAW END]") && in_pass) {
                decode(argv[f], pass, has_left, &ok, &wrong, &total);
                in_pass = 0; g_rows = 0;
            }
        }
        if (in_pass && g_rows) decode(argv[f], pass, has_left,
                                      &ok, &wrong, &total);
        fclose(fp);
    }
    printf("\ncorrect %d, wrong %d, passes %d\n", ok, wrong, total);
    return wrong != 0;
}
