/**
 * A robot that falls down lots.
 */
#include "SPI.h"
#include <SimpleFOC.h>
#include "IMU.hpp"
#include "BLEManager.hpp"
#include "ThermistorManager.hpp"

MagneticSensorI2C as5600_r = MagneticSensorI2C(AS5600_I2C);
TwoWire &Wire_r = Wire;
BLDCMotor motor_r = BLDCMotor(7, 12.0, 450); // Nominally 250, but this value passes the 0-torque-makes-it-feel-smooth test.
BLDCDriver3PWM driver_r = BLDCDriver3PWM(6, 9, 10, 5);
const int MOTOR_R_THERMISTOR_READ_PIN = A3;
const int MOTOR_R_THERMISTOR_WRITE_PIN = A4;
const float MOTOR_R_THERMISTOR_R_DIVIDER = 1000; // ohms
const float MOTOR_THERMISTOR_LOWPASS_RC = 0.5;

MagneticSensorI2C as5600_l = MagneticSensorI2C(AS5600_I2C);
TwoWire Wire_l(NRF_TWIM1, NRF_TWIS1, SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQn, 24, 25);
BLDCMotor motor_l = BLDCMotor(7, 12.0, 450); // Nominally 250, but this value passes the 0-torque-makes-it-feel-smooth test.
BLDCDriver3PWM driver_l = BLDCDriver3PWM(12, 13, 26, 11);
const int MOTOR_L_THERMISTOR_READ_PIN = A2;
const int MOTOR_L_THERMISTOR_WRITE_PIN = A1;
const float MOTOR_L_THERMISTOR_R_DIVIDER = 1000; // ohms

IMU mpu(Wire_l, &Serial);
ThermistorManager thermistor_l(MOTOR_L_THERMISTOR_READ_PIN, MOTOR_L_THERMISTOR_WRITE_PIN, MOTOR_L_THERMISTOR_R_DIVIDER, MOTOR_THERMISTOR_LOWPASS_RC);
ThermistorManager thermistor_r(MOTOR_R_THERMISTOR_READ_PIN, MOTOR_R_THERMISTOR_WRITE_PIN, MOTOR_R_THERMISTOR_R_DIVIDER, MOTOR_THERMISTOR_LOWPASS_RC);

// Control params. Override by passing float buffer in this order.
struct ControlParams {
  float KP = 1.0;
  float KD = -0.05;
} control_params;


// Non-overrideable float params.
const float BASIN_OF_ATTRACTION = 0.5;

void onBLEPacketReceived(const uint8_t *buffer, size_t size)
{
  Serial.println("Got packet");
  if (size == sizeof(ControlParams)){
    memcpy(&control_params, (const ControlParams*) buffer, sizeof(ControlParams));
  } 
  maybe_send_ble_uart((const uint8_t *)&control_params, sizeof(ControlParams));
}

void setup()
{
  analogReadResolution(12);

  // monitoring port
  Serial.begin(115200);

  Wire_l.setClock(TWIM_FREQUENCY_FREQUENCY_K400);
  Wire_r.setClock(TWIM_FREQUENCY_FREQUENCY_K400);
  Wire_l.begin();
  Wire_r.begin();

  // Talk to IMU
  while (!mpu.try_connect())
  {
    Serial.println("Failed to find MPU6050 chip");
    delay(500);
  }
  Serial.println("MPU6050 Found!");

  while (!begin_ble("fallsdownlots"))
  {
    Serial.println("Failed to set up BLE.");
    delay(1000);
  }
  blueart_packet_serial.setPacketHandler(&onBLEPacketReceived);

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

void do_control()
{
  float target_torque = 0.0;
  if (fabs(mpu.angle()) < BASIN_OF_ATTRACTION)
  {
    target_torque = control_params.KP * mpu.angle() - control_params.KD * mpu.dangle();
  }

  motor_l.move(target_torque);
  motor_r.move(target_torque);
}

float low_pass_x_acc = 0.;

long unsigned int last_printed_t = 0;
long unsigned int last_command_t = 0;
long unsigned int last_thermistor_read_t = 0;
long unsigned int last_ble_uart_update = 0;
float ble_send_buffer[7];
void loop()
{
  long unsigned int t = millis();

  // Loop management to prioritize good state estimation (targeting 1khz)
  // We're very limited by I2C comm rate, which is set to its max (400khz).
  mpu.update();

  if (t - last_command_t > 1)
  {
    last_command_t = t;
    // TODO(gizatt) I'm pretty sure the source of slowness here is I2C comms
    // with both magnetic sensors. I suspect it'd be pretty straightforward
    // to make a new MagneticSensorI2C class that reads the sensor at a slow
    // rate (like 30hz), estimates velocity from the signals, and returns
    // projected rotation estimates at higher rates.
    as5600_r.update();
    as5600_l.update();
    motor_l.loopFOC();
    motor_r.loopFOC();
    do_control();
  }

  if (t - last_thermistor_read_t > 50)
  {
    last_thermistor_read_t = t;
    // Don't need tons of accuracy out of these, so don't update super frequently.
    thermistor_l.update();
    thermistor_r.update();
  }

  if (t - last_ble_uart_update > 10)
  {
    last_ble_uart_update = t;
    update_ble_uart();
  }

  if (t - last_printed_t > 250)
  {
    last_printed_t = t;
    // display the angle and the angular velocity to the terminal
    float l_angle = as5600_l.getAngle();
    float l_temp = thermistor_l.get_temperature();
    float r_angle = as5600_r.getAngle();
    float r_temp = thermistor_r.get_temperature();

    Serial.printf("L[%08.4f rad, %04.1fC] R[%08.4f, %04.1fC] IMU[%03.1f %03.1f %03.1fms]\n", l_angle, l_temp, r_angle, r_temp, mpu.angle(), mpu.dangle(), 1000. * mpu.avg_update_dt());

    // Send current state as a simple float buffer
    ble_send_buffer[0] = mpu.angle();
    ble_send_buffer[1] = 1000. * mpu.avg_update_dt();
    ble_send_buffer[2] = r_angle;
    ble_send_buffer[3] = max(l_temp, r_temp);
    maybe_send_ble_uart((uint8_t *)ble_send_buffer, sizeof(float) * 4);
  }
}