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

Motor *motor;
const uint8_t MOTOR_PWM_PIN = 6;
const uint8_t MOTOR_FWD_PIN = 7;
const uint8_t MOTOR_REV_PIN = 8;
const uint8_t POT_PIN = 14;

void setup()
{
  Serial.begin(9800);

  pinMode(POT_PIN, INPUT);

  display = new Display(&Serial);
  blinker = new LEDBlinker(LED_BUILTIN);
  imu = new IMU(&Serial);
  motor = new Motor(MOTOR_PWM_PIN, MOTOR_FWD_PIN, MOTOR_REV_PIN);
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

    // float period = 10.0; // seconds
    // float speed = 0.5 + 0.5 * sin(((float)t) / 1000. * (2. * PI / period));
    // float speed = ((float)analogRead(POT_PIN)) / 1024.;
    float angle_zero = -5. + 10. * ((float)analogRead(POT_PIN)) / 1024;
    float kp = 0.01;
    float kd = 0.0;
    float ki = 0.0; //
    // motor->DEADBAND = ;
    float angle = imu->angle() * 180. / PI;                    // deg
    float dangle = -imu->rotational_velocity()[1] * 180. / PI; // deg/s
    float speed = (angle - angle_zero) * kp + dangle * kd + err_int * ki;

    // Integrate error
    float dt = max((float)(micros() - last_update_t) / 1E6, 0.);
    last_update_t = micros();
    err_int += (angle - angle_zero) * dt;
    // Anti-windup
    err_int = max(min(err_int, 1.), -1.);

    if (t < 1000 || abs(angle) > 45) // Don't spin motor on startup or if fallen over.
    {
      speed = 0.;
      // Reset integrator
      err_int = 0;
    }

    motor->set_speed(speed);
    display->draw_text("A: %.1f\n(%.4f)", angle, angle_zero);
  }
}