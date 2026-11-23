/** @file test_report.c
 * @brief Report on-target checks through the existing monitor console.
 */
#include "test_report.h"
#include <tm/tmonitor.h>

void test_report_check (char const * p_name, bool b_passed)
{
    /* UB views read-only string bytes; the monitor does not modify them. */
    tm_printf((UB *)"[TEST] %s: %s\n", p_name, b_passed ? "PASS" : "FAIL");
}

/*** end of file ***/

