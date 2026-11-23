/** @file g7_odometry_config.h
 * @brief Odometry scale for reporting only; never used for motor control.
 * 0=disabled, 1=nominal/provisional, 2=measured wheel configuration.
 * To set 2, physically measure wheel diameter and encoder counts for one
 * FULL wheel revolution; confirm both wheel forward signs separately.
 * The map is dead reckoning, NOT a real-time positioning or SLAM system.
 */
#ifndef G7_ODOMETRY_CONFIG_H
#define G7_ODOMETRY_CONFIG_H
#include "board/board_config.h"
#define G7_ODOMETRY_MODE 1
/* Share board values with bypass geometry; still a provisional scale. */
#define G7_ENCODER_COUNTS_PER_REV BOARD_ENC_COUNTS_PER_REV
#define G7_WHEEL_DIAMETER_MM BOARD_WHEEL_DIAMETER_MM
#if G7_ODOMETRY_MODE > 2 || G7_ODOMETRY_MODE < 0
#error "G7_ODOMETRY_MODE must be 0, 1, or 2"
#endif
#if G7_ODOMETRY_MODE != 0 && (G7_ENCODER_COUNTS_PER_REV == 0)
#error "Odometry needs nonzero encoder counts per wheel revolution"
#endif
#endif
/*** end of file ***/
