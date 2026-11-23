/** @file test_ter.c
 * @brief Pure arithmetic checks; no change to the dormant IMU task.
 */
#include "test_ter.h"
#include "terrain.h"
#include "tests/test_report.h"

void test_ter_run (void)
{
    test_report_check("IMU zero magnitude", 0 == terrain_magnitude(0, 0, 0));
    test_report_check("IMU signed single axis",
                      1000 == terrain_magnitude(-1000, 0, 0));
    test_report_check("IMU 12-bit negative extrema",
                      3547 == terrain_magnitude(-2048, -2048, -2048));
}

/*** end of file ***/

