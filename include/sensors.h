#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <config.h>
#include <pinout.h>
#include <utils.h>

/**
 * @brief Number of sensors.
 *
 */
#define SENSORS_COUNT 16

/**
 * @brief Maximum and minimum value.
 *
 */
#define SENSORS_MAX 4095
#define SENSORS_MIN 0


/**
 * @brief Maximum value of the position on the line.
* The range of values ​​will be from -SENSORS_POSITION_MAX to SENSORS_POSITION_MAX, with 0 as the center point.
 *
 */
#define SENSORS_POSITION_MAX 255

void calibrate_sensors();
int get_sensor_raw(int sensor);
int get_sensor_calibrated(int sensor);
int get_sensor_position(int last_position);
long get_last_line_detected_ms();

#endif // SENSORS_H