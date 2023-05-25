/**
 * A robot that falls down lots.
 */
#include "SPI.h"
#include <SimpleFOC.h>
#include "IMU.hpp"
#include "BLEManager.hpp"

MagneticSensorI2C as5600_r = MagneticSensorI2C(AS5600_I2C);
TwoWire &Wire_r = Wire;
BLDCMotor motor_r = BLDCMotor(7, 12.0, 450); // Nominally 250, but this value passes the 0-torque-makes-it-feel-smooth test.
BLDCDriver3PWM driver_r = BLDCDriver3PWM(6, 9, 10, 5);
const int MOTOR_R_THERMISTOR_PIN = A4;
const float MOTOR_R_THERMISTOR_R_DIVIDER = 1000; // ohms

MagneticSensorI2C as5600_l = MagneticSensorI2C(AS5600_I2C);
TwoWire Wire_l(NRF_TWIM1, NRF_TWIS1, SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQn, 24, 25);
BLDCMotor motor_l = BLDCMotor(7, 12.0, 450); // Nominally 250, but this value passes the 0-torque-makes-it-feel-smooth test.
BLDCDriver3PWM driver_l = BLDCDriver3PWM(12, 13, 26, 11);
const int MOTOR_L_THERMISTOR_PIN = A1;
const float MOTOR_L_THERMISTOR_R_DIVIDER = 914; // ohms

// THERMISTOR STUFF
// 3.3V -> [parallel 1.6kohm leakage from somewhere reg with Motor Thermistor] -> [VSENSE] -> [divider_R~1k] resistor -> GND
// LUT starts at 0 and each entry is 5*C hotter.
const float thermistor_lut_temp_spacing = 5.;
const float thermistor_lut_resistances[] = {32.96, 25.58, 20.00, 15.76, 12.51, 10.0, 8.048, 6.518, 5.312, 4.354, 3.588, 2.974, 2.476, 2.072, 1.743, 1.437, 1.250, 1.065, 0.9110, 0.7824, 0.6744, 0.5836, 0.5066};
const int thermistor_lut_entries = 23;

IMU mpu(Wire_l, &Serial);
// TODO: low-pass this value, it's pretty noisy as-is.
float get_motor_temperature(int thermistor_pin, float divider_r)
{
  float v_sense = min(3.3 - 1E-3, 3.3 * float(analogRead(thermistor_pin)) / 1023.);
  // v_sense = 3.3 * R_div / (R_div + R_therm)
  // v_sense * (R_div + R_therm) = 3.3 * R_div
  // v_Sense * R_div + v_sense * R_therm = 3.3 * R_div
  // R_therm = R_div * (3.3 - v_sense) / v_sense
  // final units in kohm
  float thermistor_and_regulator_resistance = (divider_r * (3.3 - v_sense) / v_sense) / 1000.;
  float R_leak = 1.6; // not sure where this leak is coming from. leakage of regulator, maybe?
  // rtot = 1 / (1/R_div + 1/R_leak))
  // rtot / rdiv + rtot / rleak = 1
  // rtot / rdiv = 1. - rtot / rleak
  // rdiv = rtot / (1. - rtot / rleak)
  // math error somewhere, need a sign flip... ugh
  float thermistor_resistance = -thermistor_and_regulator_resistance / (1. - thermistor_and_regulator_resistance / R_leak);

  // See where we fall in the lut.
  float temperature = 1234.0; // obviously uninitialized value
  if (thermistor_resistance > thermistor_lut_resistances[0])
  {
    temperature = 0.0;
  }
  else if (thermistor_resistance <= thermistor_lut_resistances[thermistor_lut_entries - 1])
  {
    temperature = 110.0;
  }
  else
  {
    for (int i = 0; i < thermistor_lut_entries - 1; i++)
    {
      float r_lower = thermistor_lut_resistances[i + 1];
      float r_upper = thermistor_lut_resistances[i];
      if (thermistor_resistance >= r_lower && thermistor_resistance < r_upper)
      {
        // Ratio -> 0 when resistance = r_lower, -> 1 when = r_upper.
        float ratio = (thermistor_resistance - r_lower) / (r_upper - r_lower);
        temperature = ratio * (i * 5.) + (1. - ratio) * (i + 1) * 5.;
        break;
      }
    }
  }
  return temperature;
}

void setup()
{
  analogReadResolution(12);

  // monitoring port
  Serial.begin(115200);


  Wire_l.setClock(800000);
  Wire_r.setClock(800000);
  Wire_l.begin();
  Wire_r.begin();

  // Talk to IMU
  while (!mpu.try_connect()){
    Serial.println("Failed to find MPU6050 chip");
    delay(500);
  }
  Serial.println("MPU6050 Found!");

  while (!begin_ble("fallsdownlots")){
    Serial.println("Failed to set up BLE.");
    delay(1000);
  }

  // Initialize magnetic encoders.
  as5600_r.init(&Wire_r);
  as5600_l.init(&Wire_l);

  Serial.println("AS5600 ready");
  _delay(1000);

  // initialize motor
  // driver config
  // power supply voltage [V]
  driver_r.voltage_power_supply = 7.4;
  driver_r.init();
  motor_r.linkDriver(&driver_r);
  motor_r.linkSensor(&as5600_r);

  // set control loop type to be used
  motor_r.torque_controller = TorqueControlType::voltage;
  motor_r.PID_velocity.output_ramp = 1000;
  motor_r.controller = MotionControlType::torque;
  // limiting voltage
  motor_r.voltage_limit = 7.4; // Volts
  // or current  - if phase resistance provided
  motor_r.current_limit = 1.0; // Amps
  motor_r.PID_velocity.P = 0.1;
  motor_r.PID_velocity.D = 0.001;
  motor_r.PID_velocity.I = 10;
  motor_r.LPF_velocity.Tf = 0.1;

  // initialise motor
  motor_r.useMonitoring(Serial);
  motor_r.init();
  // align encoder and start FOC
  motor_r.initFOC();
  // set the inital target value
  motor_r.target = 0;
  motor_r.velocity_limit = 1000;
  Serial.println("Motor R ready");

  // initialize motor
  // driver config
  // power supply voltage [V]
  driver_l.voltage_power_supply = 7.4;
  driver_l.init();
  motor_l.linkDriver(&driver_l);
  motor_l.linkSensor(&as5600_l);

  // set control loop type to be used
  motor_l.torque_controller = TorqueControlType::voltage;
  motor_l.PID_velocity.output_ramp = 1000;
  motor_l.controller = MotionControlType::torque;
  // limiting voltage
  motor_l.voltage_limit = 7.4; // Volts
  // or current  - if phase resistance provided
  motor_l.current_limit = 1.0; // Amps
  motor_l.PID_velocity.P = 0.1;
  motor_l.PID_velocity.D = 0.001;
  motor_l.PID_velocity.I = 10;
  motor_l.LPF_velocity.Tf = 0.1;

  // initialise motor
  motor_l.useMonitoring(Serial);
  motor_l.init();
  // align encoder and start FOC
  motor_l.initFOC();
  // set the inital target value
  motor_l.target = 0;
  motor_l.velocity_limit = 1000;
  Serial.println("Motor L ready");
}

float low_pass_x_acc = 0.;

long unsigned int last_printed_t = 0;
long unsigned int last_command_t = 0;
void loop()
{
  // Loop management to prioritize good state estimation (targeting 1khz)
  // then good 
  mpu.update();

  long unsigned int t = millis();
  if (t - last_command_t > 3){
    as5600_r.update();
    as5600_l.update();
    motor_l.loopFOC();
    motor_r.loopFOC();

    //float target_torque = sin((2. * PI / 5.) * ((float)t) / 1E3) * 0.1;
    const float KP = 1.0;
    const float KD = 0.01;
    const float BASIN_OF_ATTRACTION = 0.5;
    float target_torque;
    if (fabs(mpu.angle()) < BASIN_OF_ATTRACTION){
      target_torque =  KP * mpu.angle() - KD * mpu.dangle();
    }

    motor_l.move(target_torque);
    motor_r.move(target_torque);
    last_command_t = t;
  }

  if (t - last_printed_t > 100)
  {
    // display the angle and the angular velocity to the terminal
    float l_angle = as5600_l.getAngle();
    float l_temp = get_motor_temperature(MOTOR_L_THERMISTOR_PIN, MOTOR_L_THERMISTOR_R_DIVIDER);
    float r_angle = as5600_r.getAngle();
    float r_temp = get_motor_temperature(MOTOR_R_THERMISTOR_PIN, MOTOR_R_THERMISTOR_R_DIVIDER);


    Serial.printf("L[%08.4f rad, %04.1fC] R[%08.4f, %04.1fC] IMU[%03.1f %03.1f %03.1fms]\n", l_angle, l_temp, r_angle, r_temp, mpu.angle(), mpu.dangle(), 1000.*mpu.avg_update_dt());
    last_printed_t = t;

    //motor_l.move(max(-1., min(1., -low_pass_x_acc * 10.)));
    //motor_r.move(max(-1., min(1., -low_pass_x_acc * 10.)));
  }
}