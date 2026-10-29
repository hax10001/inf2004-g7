/** @file lsm303.h
 * @brief LSM303DLHC register operations through the micro T-Kernel I2C device.
 */
#ifndef LSM303_H
#define LSM303_H

#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>

/** @brief Configure the accelerometer and magnetometer on an open device. */
bool lsm303_init (ID h_device);
/** @brief Read signed 12-bit accelerometer axes in the original mg scale. */
bool lsm303_read_accel (ID h_device, int16_t * p_x, int16_t * p_y,
                        int16_t * p_z);
/** @brief Read signed magnetometer axes in X-Z-Y register order. */
bool lsm303_read_mag (ID h_device, int16_t * p_x, int16_t * p_y, int16_t * p_z);

#endif /* LSM303_H */

/*** end of file ***/

