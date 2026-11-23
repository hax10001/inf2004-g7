/* V10 host regression: minimum-width field semantics and bounded SPSC queue. */
#include "app/nav_core.h"
#include "modules/barcode_scanning/bc_fastq.h"
#include "modules/barcode_scanning/bc_scan.h"
#include "modules/barcode_scanning/bc_code.h"
#include "modules/barcode_scanning/bc_match.h"
#include "modules/barcode_scanning/bc_layout.h"
#include "modules/telemetry/buddy1.h"
#include <assert.h>
#include <stdint.h>
#include <stdio.h>

int main(void)
{
    nav_sample_t sample = {0};
    nav_controller_t nav = {0};
    nav_result_t result = {0};
    bc_fastq_sample_t entry = {0};
    bc_fastq_sample_t out = {0};
    bc_code_result_t code = {0};
    bc_scan_t scanner = {0};
    uint32_t widths[BC_CODE_ELEMENTS] = {0};
    uint8_t failure = 0U;
    uint32_t stamp = 0U;

    sample.left_black = 1U;
    sample.right_black = 1U;
    sample.barcode_capturing = 1U;
    sample.barcode_decoding = 1U;
    sample.both_interrupted = 1U;
    assert(sample.left_black && sample.right_black &&
           sample.barcode_capturing && sample.barcode_decoding &&
           sample.both_interrupted);
    nav.state = NAV_CENTRE; /* 9th state requires 4 bits, not 3. */
    nav.pending = 1U;
    nav.letter = 25U;
    nav.action = NAV_UTURN;
    nav.fault_reason = 5U;
    nav.held_letter = 25U;
    assert(nav.state == NAV_CENTRE && nav.pending == 1U);
    assert(nav.letter == 25U && nav.held_letter == 25U);
    assert(nav.action == NAV_UTURN && nav.fault_reason == 5U);
    result.drive = NAV_OUTPUT_UTURN;
    result.event = NAV_EVENT_DECODE_WAIT;
    result.value = 5U;
    assert(result.drive == NAV_OUTPUT_UTURN &&
           result.event == NAV_EVENT_DECODE_WAIT && result.value == 5U);
    code.letter = BC_CODE_LETTER_INVALID;
    code.quality = 3U;
    code.reverse = 1U;
    code.valid = 0U;
    assert(code.letter == 31U && code.quality == 3U && code.reverse);
    scanner.state = BC_SCAN_INVALID;
    scanner.run_count = BC_CODE_ELEMENTS;
    scanner.last_black = 1U;
    scanner.error = BC_SCAN_ERROR_BUSY;
    assert(scanner.state == BC_SCAN_INVALID &&
           scanner.run_count == 29U && scanner.error == 8U);
    assert(bc_scan_take(&scanner, widths, &failure, &stamp) ==
           BC_SCAN_EVENT_REJECTED);
    assert(failure == BC_SCAN_ERROR_BUSY && scanner.state == BC_SCAN_IDLE);

    /* The producer publishes whole records and then a release-ordered
     * queue index. A consumer does not write packed fields in the ring. */
    bc_fastq_reset();
    entry.time_usec = 12345U;
    entry.left_count = -300;
    entry.right_count = 450;
    entry.left_edge_usec = 11100U;
    entry.right_edge_usec = 11900U;
    entry.adc_raw = 4095U;
    entry.gpio_level = 1U;
    entry.adc_valid = 1U;
    entry.valid = 1U;
    assert(bc_fastq_push(&entry) == 1U);
    assert(bc_fastq_pop(&out) == 1U);
    assert(out.adc_raw == 4095U && out.gpio_level == 1U);
    assert(out.adc_valid == 1U && out.valid == 1U);
    assert(out.left_count == -300 && out.right_count == 450);
    assert(bc_fastq_pop(&out) == 0U);
    bc_fastq_reset();
    for (uint32_t i = 0U; i < BC_FASTQ_CAPACITY; ++i)
    {
        entry.time_usec = i;
        entry.adc_raw = (i * 37U) & 4095U;
        assert(bc_fastq_push(&entry) == 1U);
    }
    assert(bc_fastq_push(&entry) == 0U); /* Full queue fails closed. */
    assert(bc_fastq_pop(&out) == 1U);
    assert(out.valid == 0U); /* Overflow invalidates the next sample. */
    for (uint32_t i = 1U; i < BC_FASTQ_CAPACITY; ++i)
    {
        assert(bc_fastq_pop(&out) == 1U);
        assert(out.adc_raw == ((i * 37U) & 4095U));
        assert(out.valid == 1U);
    }
    assert(bc_fastq_pop(&out) == 0U);
    printf("V10_SIZES nav_sample=%zu nav_controller=%zu nav_result=%zu "
           "code_result=%zu scan=%zu fastq_sample=%zu ring=%zu "
           "layout=%zu match=%zu telemetry_store=%zu\n",
           sizeof(nav_sample_t), sizeof(nav_controller_t),
           sizeof(nav_result_t), sizeof(bc_code_result_t),
           sizeof(bc_scan_t), sizeof(bc_fastq_sample_t),
           sizeof(bc_fastq_sample_t) * BC_FASTQ_CAPACITY,
           sizeof(bc_layout_result_t), sizeof(bc_match_result_t),
           sizeof(B1_Store));
    puts("PASS V10 boundaries, packed states, barcode SPSC and overflow");
    return 0;
}
