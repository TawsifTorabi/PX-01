#include <sensors.h>

static int sensors_raw[SENSORS_COUNT];
static long sensors_refresh_mc = 0;

static int sensors_max[SENSORS_COUNT] = {SENSORS_MIN, SENSORS_MIN, SENSORS_MIN, SENSORS_MIN, SENSORS_MIN, SENSORS_MIN, SENSORS_MIN, SENSORS_MIN, SENSORS_MIN, SENSORS_MIN, SENSORS_MIN, SENSORS_MIN, SENSORS_MIN, SENSORS_MIN, SENSORS_MIN, SENSORS_MIN};
static int sensors_min[SENSORS_COUNT] = {SENSORS_MAX, SENSORS_MAX, SENSORS_MAX, SENSORS_MAX, SENSORS_MAX, SENSORS_MAX, SENSORS_MAX, SENSORS_MAX, SENSORS_MAX, SENSORS_MAX, SENSORS_MAX, SENSORS_MAX, SENSORS_MAX, SENSORS_MAX, SENSORS_MAX, SENSORS_MAX};
static int sensors_umb[SENSORS_COUNT] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

static long last_line_detected_ms = 0;

/**
 * @brief Updates sensor values ​​every 1ms.
 * Values ​​are read two at a time for each multiplexer state.
 * Sensor values ​​are also inverted if LINE_WHITE is selected in the configuration.
 *
 */
static void refresh_sensors() {
  if (micros() - sensors_refresh_mc >= 1000 || micros() < sensors_refresh_mc) {

#ifdef LINE_BLACK
    sensors_raw[7] = analogRead(SENSOR_1);
    sensors_raw[8] = analogRead(SENSOR_2);
    digitalWrite(MUX_A, HIGH); // 0 0 1
    sensors_raw[6] = analogRead(SENSOR_1);
    sensors_raw[9] = analogRead(SENSOR_2);
    digitalWrite(MUX_B, HIGH); // 0 1 1
    sensors_raw[4] = analogRead(SENSOR_1);
    sensors_raw[11] = analogRead(SENSOR_2);
    digitalWrite(MUX_A, LOW); // 0 1 0
    sensors_raw[5] = analogRead(SENSOR_1);
    sensors_raw[10] = analogRead(SENSOR_2);
    digitalWrite(MUX_C, HIGH); // 1 1 0
    sensors_raw[1] = analogRead(SENSOR_1);
    sensors_raw[14] = analogRead(SENSOR_2);
    digitalWrite(MUX_A, HIGH); // 1 1 1
    sensors_raw[0] = analogRead(SENSOR_1);
    sensors_raw[15] = analogRead(SENSOR_2);
    digitalWrite(MUX_B, LOW); // 1 0 1
    sensors_raw[2] = analogRead(SENSOR_1);
    sensors_raw[13] = analogRead(SENSOR_2);
    digitalWrite(MUX_A, LOW); // 1 0 0
    sensors_raw[3] = analogRead(SENSOR_1);
    sensors_raw[12] = analogRead(SENSOR_2);
    digitalWrite(MUX_C, LOW); // 0 0 0
#else
    sensors_raw[7] = SENSORS_MAX - analogRead(SENSOR_1);
    sensors_raw[8] = SENSORS_MAX - analogRead(SENSOR_2);
    digitalWrite(MUX_A, HIGH); // 0 0 1
    sensors_raw[6] = SENSORS_MAX - analogRead(SENSOR_1);
    sensors_raw[9] = SENSORS_MAX - analogRead(SENSOR_2);
    digitalWrite(MUX_B, HIGH); // 0 1 1
    sensors_raw[4] = SENSORS_MAX - analogRead(SENSOR_1);
    sensors_raw[11] = SENSORS_MAX - analogRead(SENSOR_2);
    digitalWrite(MUX_A, LOW); // 0 1 0
    sensors_raw[5] = SENSORS_MAX - analogRead(SENSOR_1);
    sensors_raw[10] = SENSORS_MAX - analogRead(SENSOR_2);
    digitalWrite(MUX_C, HIGH); // 1 1 0
    sensors_raw[1] = SENSORS_MAX - analogRead(SENSOR_1);
    sensors_raw[14] = SENSORS_MAX - analogRead(SENSOR_2);
    digitalWrite(MUX_A, HIGH); // 1 1 1
    sensors_raw[0] = SENSORS_MAX - analogRead(SENSOR_1);
    sensors_raw[15] = SENSORS_MAX - analogRead(SENSOR_2);
    digitalWrite(MUX_B, LOW); // 1 0 1
    sensors_raw[2] = SENSORS_MAX - analogRead(SENSOR_1);
    sensors_raw[13] = SENSORS_MAX - analogRead(SENSOR_2);
    digitalWrite(MUX_A, LOW); // 1 0 0
    sensors_raw[3] = SENSORS_MAX - analogRead(SENSOR_1);
    sensors_raw[12] = SENSORS_MAX - analogRead(SENSOR_2);
    digitalWrite(MUX_C, LOW); // 0 0 0
#endif

    sensors_refresh_mc = micros();
  }
}

/**
 * @brief Prints sensor calibration values
 *
 */
static void print_calibrations() {
  Serial.println("Calibrations: ");
  for (int sensor = 0; sensor < SENSORS_COUNT; sensor++) {
    Serial.print(sensors_min[sensor]);
    Serial.print(" ");
    Serial.print(sensors_max[sensor]);
    Serial.print(" ");
    Serial.println(sensors_umb[sensor]);
  }
}

/**
 * @brief Calibrate the sensors by obtaining the maximum and minimum values ​​to calculate the threshold from which a sensor is considered to be detecting the line.
 * The threshold is calculated as 2/3 of the range of values ​​between the maximum and minimum to further saturate the reading and filter out possible track imperfections.
 *
 */
void calibrate_sensors() {
  set_led(RGB_LEFT, 80, 20, 0);
  set_led(RGB_TOP, 80, 20, 0);
  set_led(RGB_RIGHT, 80, 20, 0);
  while (get_btn_pressed_state() != BTN_PRESSED) {
    blink_led(RGB_TOP, 80, 20, 0, 350);
  }
  clear_led(RGB_LEFT);
  clear_led(RGB_TOP);
  clear_led(RGB_RIGHT);
  int calibration_start_ms = millis();
  int count_ok = 0;
  do {
    rainbow_led(RGB_TOP);
    count_ok = 0;
    for (int sensor = 0; sensor < SENSORS_COUNT; sensor++) {
      int sensor_value = get_sensor_raw(sensor);
      if (sensor_value > sensors_max[sensor]) {
        sensors_max[sensor] = sensor_value;
      }
      if (sensor_value < sensors_min[sensor]) {
        sensors_min[sensor] = sensor_value;
      }
      sensors_umb[sensor] = sensors_min[sensor] + ((sensors_max[sensor] - sensors_min[sensor]) * 2 / 3);
      if (abs(sensors_max[sensor] - sensors_min[sensor]) >= 1000) {
        count_ok++;
      }
    }
  } while (millis() - calibration_start_ms < SENSORS_CALIBRATION_MS);
  print_calibrations();

  clear_led(RGB_LEFT);
  clear_led(RGB_TOP);
  clear_led(RGB_RIGHT);
  if (count_ok == SENSORS_COUNT) {
    set_led(RGB_TOP, 0, 50, 0);
    delay(1000);
  } else {
    while (get_btn_pressed_state() != BTN_LONG_PRESSED) {
      blink_led(RGB_TOP, 50, 0, 0, 75);
    }
  }
  clear_led(RGB_TOP);
  delay(500);
}

/**
 * @brief Gets the raw value from a sensor
 *
 * @param sensor Sensor to read
 * @return int Raw value of sensor
 */
int get_sensor_raw(int sensor) {
  if (sensor >= 0 && sensor < SENSORS_COUNT) {
    refresh_sensors();
    return sensors_raw[sensor];
  } else {
    return -1;
  }
}

/**
 * @brief Gets the calibrated value of a sensor
 *
 * @param sensor Sensor to read
 * @return int Calibrated sensor value
 */
int get_sensor_calibrated(int sensor) {
  if (sensor >= 0 && sensor < SENSORS_COUNT) {
    refresh_sensors();
    return sensors_raw[sensor] >= sensors_umb[sensor] ? SENSORS_MAX : SENSORS_MIN;
  } else {
    return -1;
  }
}

/**
 * @brief Gets the position of the robot on the track
 *
 * @param last_position Last known position of the robot
 * @return int Position of the robot on the track
 */
int get_sensor_position(int last_position) {
  long sum_sensors_weight = 0;
  long sum_sensors = 0;
  int count_sensors_detecting = 0;
  for (int sensor = 0; sensor < SENSORS_COUNT; sensor++) {
    int sensor_value = get_sensor_calibrated(sensor);
    if (sensor_value >= sensors_umb[sensor]) {
      count_sensors_detecting++;
    }
    sum_sensors_weight += (sensor + 1) * sensor_value * 1000;
    sum_sensors += sensor_value;
  }

  int position_max = ((1000 * (SENSORS_COUNT + 1)) / 2);
  int position = 0;
  if (count_sensors_detecting > 0 && count_sensors_detecting < SENSORS_COUNT) {
    position = (sum_sensors_weight / sum_sensors) - position_max;
    last_line_detected_ms = millis();
  } else {
    position = last_position >= 0 ? position_max : -position_max;
  }
  return map(position, -position_max, position_max, -SENSORS_POSITION_MAX, SENSORS_POSITION_MAX);
}

/**
 * @brief Gets the time in ms since the last time the line was detected.
 * Comparing this data with the result of millis() allows you to calculate the time elapsed since the last time the line was detected.
 * This way you can safely stop the robot in case of line loss.
 *
 * @return long Time in ms since the last line detection.
 */
long get_last_line_detected_ms() {
  return last_line_detected_ms;
}
