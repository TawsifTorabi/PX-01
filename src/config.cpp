#include <config.h>

/**
 * @brief Initializes the robot components.
 *
 */
void init_components() {
  Serial.begin(115200);

  pinMode(NEOPIXEL, OUTPUT);
  pinMode(MOD_START, INPUT_PULLDOWN);
  pinMode(BTN_1, INPUT_PULLDOWN);

  pinMode(SENSOR_1, OUTPUT);
  pinMode(SENSOR_2, OUTPUT);
  pinMode(MUX_C, OUTPUT);
  pinMode(MUX_B, OUTPUT);
  pinMode(MUX_A, OUTPUT);

  pinMode(MOTOR_RIGHT_A, OUTPUT);
  pinMode(MOTOR_RIGHT_B, OUTPUT);
  pinMode(MOTOR_LEFT_A, OUTPUT);
  pinMode(MOTOR_LEFT_B, OUTPUT);
  digitalWrite(MOTOR_RIGHT_A, HIGH);
  digitalWrite(MOTOR_RIGHT_B, HIGH);
  digitalWrite(MOTOR_LEFT_A, HIGH);
  digitalWrite(MOTOR_LEFT_B, HIGH);

  init_motors();
}

/**
 * @brief Gets the wait time to start the race after pressing the button.
 *
 * @return long Wait time in ms.
 */
long get_ms_start() {
#ifdef MODE_RACE
  return START_RACE_MS;
#else
  return START_TEST_MS;
#endif
}