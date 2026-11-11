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
    test_report_check("IMU jerk", 5000 == terrain_jerk(1100, 1000, 20U));
    test_report_check("IMU zero elapsed time",
                      0 == terrain_jerk(1100, 1000, 0U));

    /* Equal acceleration lengths can still have nonzero vector jerk. */
    terrain_jerk_vector jerk = terrain_vector_jerk(
        600, 0, 800, 0, 600, 800, 20U);
    test_report_check("IMU unchanged magnitude rate",
                      0 == terrain_jerk(1000, 1000, 20U));
    test_report_check("IMU vector keeps direction",
                      (30000 == jerk.x) && (-30000 == jerk.y) && (0 == jerk.z));

    jerk = terrain_vector_jerk(600, 0, 800, 600, 0, 800, 20U);
    test_report_check("IMU steady vector",
                      (0 == jerk.x) && (0 == jerk.y) && (0 == jerk.z));
    jerk = terrain_vector_jerk(600, 0, 800, 0, 600, 800, 0U);
    test_report_check("IMU vector zero elapsed",
                      (0 == jerk.x) && (0 == jerk.y) && (0 == jerk.z));
    jerk = terrain_vector_jerk(
        INT16_MAX, INT16_MIN, 0, INT16_MIN, INT16_MAX, 0, 1U);
    test_report_check("IMU vector signed bounds",
                      (65535000 == jerk.x) && (-65535000 == jerk.y) &&
                      (0 == jerk.z));
    jerk = terrain_vector_jerk(600, 0, 800, 0, 600, 800, UINT32_MAX);
    test_report_check("IMU vector long elapsed",
                      (0 == jerk.x) && (0 == jerk.y) && (0 == jerk.z));
}

/*** end of file ***/

