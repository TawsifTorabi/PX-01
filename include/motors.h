#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>
#include <control.h>
#include <pinout.h>

/**
 * @brief Configuration of the traction motors.
 * Channels: 0 to 3
 * Frequency: 1 kHz
 * Resolution: 10 bits
 * Range: 0 to 1023
 *
 */
#define PWM_MOTOR_RIGHT_A 0
#define PWM_MOTOR_RIGHT_B 1
#define PWM_MOTOR_LEFT_A 2
#define PWM_MOTOR_LEFT_B 3
#define PWM_MOTORS_HZ 1000
#define PWM_MOTORS_RESOLUTION 10
#define PWM_MOTORS_MAX 1023
#define PWM_MOTORS_MIN 0

void init_motors();
void set_motors_speed(float velI, float velD);

#endif // MOTORS_H