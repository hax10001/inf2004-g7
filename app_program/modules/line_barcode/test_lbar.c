/** @file test_lbar.c
 * @brief Non-mutating on-target checks of the implemented IR subsystem.
 */
#include "test_lbar.h"
#include "line_follow.h"
#include "ir_shared.h"
#include "tests/test_report.h"

void test_lbar_run (void)
{
    test_report_check("IR white/white",
                      LINE_FORWARD == line_follow_decide(0U, 0U));
    test_report_check("IR black/white",
                      LINE_CORRECT_LEFT == line_follow_decide(1U, 0U));
    test_report_check("IR white/black",
                      LINE_CORRECT_RIGHT == line_follow_decide(0U, 1U));
    test_report_check("IR black/black",
                      LINE_UNCERTAIN == line_follow_decide(1U, 1U));

    ir_shared_snapshot_t reading = {0};

    test_report_check("IR snapshot mutex accessible",
                      E_OK == ir_shared_get_latest(&reading));
    test_report_check("IR null output rejected",
                      E_PAR == ir_shared_get_latest(NULL));
}

/*** end of file ***/

