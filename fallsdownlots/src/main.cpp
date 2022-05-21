/**
 * Robot that falls down lots
 */

// We need both I2C devices.
#include "snprinth.h"

#include "Arduino.h"

#include "LEDBlinker.hpp"
#include "IMU.hpp"
#include "Display.hpp"
#include "Motor.hpp"

#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif

LEDBlinker *blinker;
IMU *imu;
Display *display;

StepperMotor *motor;
const uint8_t EN_A = 6;
const uint8_t FWD_A = 7;
const uint8_t REV_A = 8;
const uint8_t EN_B = 9;
const uint8_t FWD_B = 10;
const uint8_t REV_B = 11;

const uint8_t POT_PIN = 14;

void setup()
{
  Serial.begin(115200); // actually starts at some crazy USB rate

  pinMode(POT_PIN, INPUT);

  display = new Display();
  blinker = new LEDBlinker(LED_BUILTIN);
  imu = new IMU();
  motor = new StepperMotor(EN_A, FWD_A, REV_A, EN_B, FWD_B, REV_B);
}

uint32_t last_update_t = 0;
uint32_t last_update_t_us = 0;
uint32_t last_publish_t_us = 0;
char print_buf[256];

float err_int = 0.;
void loop()
{
  auto t = millis();

  // Update all modules.
  blinker->update(t);
  imu->update(t);
  display->update(t);
  motor->update(t);

  // Positive angle is corrected by positive speed.

  float angle_target_const = 2; // -5. + 10. * (float)analogRead(POT_PIN) / 1024;

  float angle_deg = imu->angle() * 180. / PI;
  float dangle_deg = imu->dangle() * 180. / PI;
  float speed = 0.;
  float AVG_SPEED_ALPHA = 0.99;
  float avg_speed = 0.;

  //
  float kp = 0.05;
  float kd = 0.001;
  float ki = 0.;
  float kid = 000.; // avg speed of 0.001 -> 1 deg correction.

  float angle_target = angle_target_const + ki * err_int - kid * avg_speed;

  if (abs(angle_deg - angle_target) <= 30.)
  {
    speed = (angle_target - angle_deg) * kp + (0. - dangle_deg) * kd;
  }

  motor->set_speed(speed);
  avg_speed = avg_speed * AVG_SPEED_ALPHA + (1. - AVG_SPEED_ALPHA) * speed;

  // Step odometry
  auto t_us = micros();
  uint32_t dt = t_us - last_update_t_us;
  last_update_t_us = t_us;
  err_int += speed * ((float)dt) / 1E6;
  err_int = min(max(err_int, -1.), 1.);

  if (t - last_update_t > 100)
  {
    display->draw_text("A: %.1f\nR: %.4f", angle_deg, avg_speed);
    last_update_t = t;
  }

  // Publish
  if (t_us - last_publish_t_us > 1000)
  {
    last_publish_t_us = t_us;
    snprintf(print_buf, 256, "%0.3f, %0.3f, %0.3f, %0.3f\n", ((float)t_us) / 1E6, angle_deg, dangle_deg, speed);
    Serial.print(print_buf);
  }
}