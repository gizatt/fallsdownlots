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
const uint8_t EN_A = 7;
const uint8_t FWD_A = 6; // PWM
const uint8_t REV_A = 9; // PWM
const uint8_t EN_B = 8;
const uint8_t FWD_B = 10; // PWM
const uint8_t REV_B = 16; // PWM

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

// State
float pos_est = 0.;
float speed_est = 0.;
float speed_est_slow = 0.;
float ang_est = 0.;
float dang_est = 0.;
float ang_target = 2.5;
float SPEED_EST_BW = 0.05;
float SPEED_EST_SLOW_BW = 0.25;

// Feedback gains
float K_pos = -0.25;
float K_speed = -2.;
float K_ang = 0.2;
float K_dang = 0.0;

// To activate: put very close to balanced for
// a few seconds.
bool active = false;
uint32_t active_start_timer = 0;

void loop()
{
  auto t = millis();

  // Update all modules.
  blinker->update(t);
  imu->update(t);
  display->update(t);
  motor->update(t);

  // Positive angle is corrected by positive speed.

  ang_est = imu->angle() * 180. / PI;
  dang_est = imu->dangle() * 180. / PI;

  float speed_set = 0.;

  // Decide to turn on if we're in the "good" region for long enough.
  if (!active && abs(ang_est - ang_target) <= 5. && t - active_start_timer > 1.0)
  {
    active = true;
  }
  else
  {
    active_start_timer = t;
  }

  if (active && abs(ang_est - ang_target) <= 15.)
  {
    float pos_err = (0. - pos_est);
    // clamp pos error contribution to control.
    pos_err = min(max(pos_err, -1.), 1.);
    speed_set = (ang_target - ang_est) * K_ang + (0. - K_dang) * K_dang + pos_err * K_pos + (0. - speed_est_slow) * K_speed;
  }
  else
  {
    active = false;
    speed_set = 0.;
    pos_est = 0.;
    speed_est = 0.;
    speed_est_slow = 0.;
  }
  speed_set = min(max(speed_set, -1.), 1.);
  motor->set_speed(speed_set);

  // Step odometry
  auto t_us = micros();
  uint32_t dt = t_us - last_update_t_us;
  if (dt >= 100)
  {
    last_update_t_us = t_us;
    float dt_f = ((float)dt) / 1E6;

    // run smooth and less smooth speed estimators for control and odometry respectively
    float speed_est_alpha = 1. - dt_f / (SPEED_EST_BW + dt_f);
    float speed_est_slow_alpha = 1. - dt_f / (SPEED_EST_SLOW_BW + dt_f);
    speed_est = speed_est * speed_est_alpha + (1. - speed_est_alpha) * speed_set;
    speed_est_slow = speed_est_slow * speed_est_slow_alpha + (1. - speed_est_slow_alpha) * speed_set;
    pos_est += speed_est * dt_f;
  }
  if (t - last_update_t > 100)
  {
    display->draw_text("x%7.02f dx%7.02f\na%7.02f da%7.02f\nActive: %d", pos_est, speed_est, ang_est, dang_est, active);
    last_update_t = t;
  }

  // Publish
  if (t_us - last_publish_t_us > 1000)
  {
    last_publish_t_us = t_us;
    snprintf(print_buf, 256, "%0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f", ((float)t_us) / 1E6, pos_est, speed_est_slow, ang_est, dang_est, speed_set);
    Serial.print(print_buf);
  }
}