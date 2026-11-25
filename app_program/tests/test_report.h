/** @file test_report.h
 * @brief On-target test result reporter.
 */
#ifndef TEST_REPORT_H
#define TEST_REPORT_H

#include <stdbool.h>

/** @brief Print PASS or FAIL for one non-mutating subsystem check. */
void test_report_check (char const * p_name, bool b_passed);

#endif /* TEST_REPORT_H */

/*** end of file ***/

