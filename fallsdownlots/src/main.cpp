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
  Serial.begin(9800);

  pinMode(POT_PIN, INPUT);

  display = new Display(&Serial);
  blinker = new LEDBlinker(LED_BUILTIN);
  imu = new IMU(&Serial);
  motor = new StepperMotor(EN_A, FWD_A, REV_A, EN_B, FWD_B, REV_B);
}

float err_int = 0.;
uint32_t last_update_t = 0;
void loop()
{
  auto t = millis();

  { // Update all modules.
    blinker->update(t);
    imu->update(t);
    display->update(t);
    motor->update(t);

    float speed = -1. + 2. * (float)analogRead(POT_PIN) / 1024;

    motor->set_speed(speed);
    display->draw_text("A: %.1f\nS: %u", imu->angle() * 180. / PI, motor->m_step_period_us);
  }
}