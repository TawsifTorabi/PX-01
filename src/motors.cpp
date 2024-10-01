#include <motors.h>

/**
 * @brief Inicializa el timer controlador de los motores
 *
 */
void init_motors() {
  // Configuración de los canales PWM del Timer
  ledcSetup(PWM_MOTOR_RIGHT_A, PWM_MOTORS_HZ, PWM_MOTORS_RESOLUTION);
  ledcSetup(PWM_MOTOR_RIGHT_B, PWM_MOTORS_HZ, PWM_MOTORS_RESOLUTION);
  ledcSetup(PWM_MOTOR_LEFT_A, PWM_MOTORS_HZ, PWM_MOTORS_RESOLUTION);
  ledcSetup(PWM_MOTOR_LEFT_B, PWM_MOTORS_HZ, PWM_MOTORS_RESOLUTION);

  // Asignación de los pines a los canales PWM
  ledcAttachPin(MOTOR_RIGHT_A, PWM_MOTOR_RIGHT_A);
  ledcAttachPin(MOTOR_RIGHT_B, PWM_MOTOR_RIGHT_B);
  ledcAttachPin(MOTOR_LEFT_A, PWM_MOTOR_LEFT_A);
  ledcAttachPin(MOTOR_LEFT_B, PWM_MOTOR_LEFT_B);

  // Establece el valor inicial de los canales PWM
  ledcWrite(PWM_MOTOR_RIGHT_A, PWM_MOTORS_MIN);
  ledcWrite(PWM_MOTOR_RIGHT_B, PWM_MOTORS_MIN);
  ledcWrite(PWM_MOTOR_LEFT_A, PWM_MOTORS_MIN);
  ledcWrite(PWM_MOTOR_LEFT_B, PWM_MOTORS_MIN);
}

/**
 * @brief Establece la velocidad de los motores
 *
 * @param velI Velocidad del motor izquierdo 0-100%
 * @param velD Velocidad del motor derecho 0-100%
 */
void set_motors_speed(float velI, float velD) {
  if (velI > 100) {
    velI = 100;
  } else if (velI < -100) {
    velI = -100;
  }

  if (is_race_started() || millis() - get_race_stopped_ms() < 1000) {
    if (velI > 0) {
      ledcWrite(PWM_MOTOR_LEFT_A, PWM_MOTORS_MAX);
      ledcWrite(PWM_MOTOR_LEFT_B, PWM_MOTORS_MAX - (PWM_MOTORS_MAX * velI / 100));
    } else {
      ledcWrite(PWM_MOTOR_LEFT_A, PWM_MOTORS_MAX - (PWM_MOTORS_MAX * abs(velI) / 100));
      ledcWrite(PWM_MOTOR_LEFT_B, PWM_MOTORS_MAX);
    }
  } else {
    ledcWrite(PWM_MOTOR_LEFT_A, PWM_MOTORS_MIN);
    ledcWrite(PWM_MOTOR_LEFT_B, PWM_MOTORS_MIN);
  }

  if (velD > 100) {
    velD = 100;
  } else if (velD < -100) {
    velD = -100;
  }

  if (is_race_started() || millis() - get_race_stopped_ms() < 1000) {
    if (velD > 0) {
      ledcWrite(PWM_MOTOR_RIGHT_A, PWM_MOTORS_MAX);
      ledcWrite(PWM_MOTOR_RIGHT_B, PWM_MOTORS_MAX - (PWM_MOTORS_MAX * velD / 100));
    } else {
      ledcWrite(PWM_MOTOR_RIGHT_A, PWM_MOTORS_MAX - (PWM_MOTORS_MAX * abs(velD) / 100));
      ledcWrite(PWM_MOTOR_RIGHT_B, PWM_MOTORS_MAX);
    }
  } else {
    ledcWrite(PWM_MOTOR_RIGHT_A, PWM_MOTORS_MIN);
    ledcWrite(PWM_MOTOR_RIGHT_B, PWM_MOTORS_MIN);
  }
}