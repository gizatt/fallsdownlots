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

const float MAX_TEMP = 70.0; // Celsius, max temp for motors before thermal shutoff disables control.

IMU mpu(Wire_l, &Serial);
ThermistorManager thermistor_l(MOTOR_L_THERMISTOR_READ_PIN, MOTOR_L_THERMISTOR_WRITE_PIN, MOTOR_L_THERMISTOR_R_DIVIDER, MOTOR_THERMISTOR_LOWPASS_RC);
ThermistorManager thermistor_r(MOTOR_R_THERMISTOR_READ_PIN, MOTOR_R_THERMISTOR_WRITE_PIN, MOTOR_R_THERMISTOR_R_DIVIDER, MOTOR_THERMISTOR_LOWPASS_RC);

// Control params.
struct NamedControlParam {
  // Actual value of the param.
  float value;
  // String name by which this param will be referred in the serial protocol. Keep this short.
  const char * name;
};

struct ControlParams {
  NamedControlParam angle_p = {1.0, "ANG_P"};
  NamedControlParam angle_d = {0.06, "ANG_D"};
  NamedControlParam angle_target_rc = {10.0, "ANG_RC"};
  NamedControlParam angle_error_max = {1.0, "ANG_TM"};
  NamedControlParam vel_p = {0.02, "VEL_P"};
  NamedControlParam vel_target = {0.0, "VEL_T"};
  NamedControlParam vel_i = {0.01, "VEL_I"};
  NamedControlParam vel_i_max = {10.0, "VEL_IM"}; // More than enough. Probably getting a little windup here.
  NamedControlParam dyaw_p = {0.05, "DYAW_P"};
  NamedControlParam dyaw_target = {0.00, "DYAW_T"};
  NamedControlParam dyaw_i = {0.00, "DYAW_I"};
  NamedControlParam dyaw_i_max = {1.0, "DYAW_IM"};
  NamedControlParam target_decay_rc = {1.0, "DEC_RC"};
  NamedControlParam target_decay_timeout = {1.0, "DEC_TM"};
  NamedControlParam send_state = {0.0, "SEND_STATE"}; // Must set to >= 0.5 to enable sending state messages.
} control_params;

// Error integrator state
float integrated_velocity_error = 0.0;
float integrated_dyaw_error = 0.0;
float average_angle = 0.0;


uint8_t send_buf[256];
uint32_t last_received_ble_packet_millis;
void onBLEPacketReceived(const uint8_t *buffer, size_t size)
{
  // Get as many chars off the front that are valid ascii chars to get a name.
  size_t name_len = 0;
  for (name_len = 0; name_len < size && buffer[name_len] >= 32 && buffer[name_len] <= 127 ;  name_len++) { ; }
  if (name_len == 0) {
    Serial.printf("Couldn't parse packet, bad name len %u\n", name_len);
    return;
  }
  int val_len = size - name_len - 1;
  if (val_len != 0 && val_len != 4){
    Serial.printf("Got invalid val len in packet: %u\n", val_len);
    return;
  }
  // Check the name against our parameters and see if any match.
  bool found_matching_param = false;
  for (uint8_t param_k = 0; param_k < (sizeof(control_params) / sizeof(NamedControlParam)); param_k++){
    // Hacky technique for iterating over our named control params...
    auto * control_param = (NamedControlParam *)(&control_params) + param_k;
    if (strncmp(control_param->name, (const char *)buffer, name_len) == 0){
      found_matching_param = true;
      if (val_len == 4){
        // Update param.
        control_param->value = * (float *) (buffer + name_len + 1);
      }

      memcpy(send_buf, buffer, name_len + 1);
      memcpy(send_buf + name_len + 1, &(control_param->value), sizeof(float));
      maybe_send_ble_uart(send_buf, name_len + 1 + sizeof(float));
      last_received_ble_packet_millis = millis();
      break;
    }
  }
  if (!found_matching_param){
    Serial.printf("Couldn't find param with name %.*s to update\n", name_len, buffer);
  }
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
    delay(1000);
  }
  Serial.println("MPU6050 Found!");

  while (!begin_ble("fallsdownlots"))
  {
    Serial.println("Failed to set up BLE.");
    delay(1000);
  }
  blueart_packet_serial.setPacketHandler(&onBLEPacketReceived);
  last_received_ble_packet_millis = millis();

  // Initialize magnetic encoders.
  as5600_r.init(&Wire_r);
  as5600_l.init(&Wire_l);

  Serial.println("AS5600 ready");
  _delay(100);

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

float clamp(float x, float lb, float ub){
  return min(max(x, lb), ub);
}

float average_velocity(){
 return 0.5 * motor_l.shaftVelocity() + 0.5 * motor_r.shaftVelocity();
}

uint32_t last_control_update_t = 0;
void do_control()
{
  uint32_t t = micros();
  float dt = ((float)(t - last_control_update_t)) / 1E6;
  last_control_update_t = t;

  if (millis() - last_received_ble_packet_millis >= control_params.target_decay_timeout.value * 1000){
    // Decay our "target" inputs back to zero if they're older than our timeout value
    float target_decay_alpha = 1. - dt / (control_params.target_decay_rc.value + dt);
    control_params.vel_target.value *= target_decay_alpha;
    control_params.dyaw_target.value *= target_decay_alpha;
  }

  float target_torque_l = 0.0;
  float target_torque_r = 0.0;
  if (fabs(mpu.pitch()) < control_params.angle_error_max.value && thermistor_l.get_temperature() <= MAX_TEMP && thermistor_r.get_temperature() <= MAX_TEMP)
  {  
    // Update estimated "upright" angle by averaging our angle measurements.
    float alpha = 1.0 - dt / (control_params.angle_target_rc.value + dt);
    average_angle = average_angle * alpha + (1.0 - alpha) * mpu.pitch();

    float pitch_error = mpu.pitch() - average_angle;
    float velocity_error = average_velocity() - control_params.vel_target.value;
    integrated_velocity_error += dt * velocity_error;
    integrated_velocity_error = clamp(integrated_velocity_error, -control_params.vel_i_max.value, control_params.vel_i_max.value);

    float dyaw_error = mpu.dyaw() - control_params.dyaw_target.value;
    integrated_dyaw_error += dt * dyaw_error;
    integrated_dyaw_error = clamp(integrated_dyaw_error, -control_params.dyaw_i_max.value, control_params.dyaw_i_max.value);
    
    float pitch_component = control_params.angle_p.value * pitch_error + control_params.angle_d.value * mpu.dpitch();
    float odometry_component = control_params.vel_p.value * velocity_error + control_params.vel_i.value * integrated_velocity_error;
    float dyaw_component = control_params.dyaw_p.value * dyaw_error + control_params.dyaw_i.value * integrated_dyaw_error;
    target_torque_l = pitch_component + odometry_component - dyaw_component;
    target_torque_r = pitch_component + odometry_component + dyaw_component;
  } else {
    integrated_velocity_error = 0.0;
    integrated_dyaw_error = 0.0;
    average_angle = 0.0;
  }

  motor_l.move(target_torque_l);
  motor_r.move(target_torque_r);
}

float low_pass_x_acc = 0.;

long unsigned int last_printed_status_t = 0;
long unsigned int last_sent_state_est_t = 0;
long unsigned int last_command_t = 0;
long unsigned int last_thermistor_read_t = 0;
long unsigned int last_ble_uart_update = 0;

float ble_state_send_buffer[12]; // Will be populated with pitch, dpitch, dyaw, integrated_velocity_error, integrated_yaw_error, control_l, angle_l, vel_l, control_r, angle_r, vel_r, max_motor_temp
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

  if (t - last_ble_uart_update > 1)
  {
    last_ble_uart_update = t;
    update_ble_uart();
  }

  if (t - last_sent_state_est_t > 50 && control_params.send_state.value >= 0.5)
  {
    last_sent_state_est_t = t;
    // display the angle and the angular velocity to the terminal
    float l_temp = thermistor_l.get_temperature();
    float r_temp = thermistor_r.get_temperature();

    // Send current state as a simple float buffer
    ble_state_send_buffer[0] = mpu.pitch();
    ble_state_send_buffer[1] = mpu.dpitch();
    ble_state_send_buffer[2] = mpu.dyaw();
    ble_state_send_buffer[3] = integrated_velocity_error;
    ble_state_send_buffer[4] = integrated_dyaw_error;
    ble_state_send_buffer[5] = motor_l.target;
    ble_state_send_buffer[6] = as5600_l.getAngle();
    ble_state_send_buffer[7] = motor_l.shaftVelocity();
    ble_state_send_buffer[8] = motor_r.target;
    ble_state_send_buffer[9] = as5600_r.getAngle();
    ble_state_send_buffer[10] = motor_r.shaftVelocity();
    ble_state_send_buffer[11] = max(l_temp, r_temp);
    maybe_send_ble_uart((uint8_t *)ble_state_send_buffer, sizeof(float) * 12);
  }

  if (t - last_printed_status_t > 250)
  {
    // Display detailed stats to the terminal.
    last_printed_status_t = t;
    float l_temp = thermistor_l.get_temperature();
    float r_temp = thermistor_r.get_temperature();

    Serial.printf("L[%04.2fr %04.2frs %04.2ft %04.2fC] R[%04.2fr %04.2frs  %04.2ft %04.2fC] Int[%04.2f %04.2f] IMU[%04.2f %04.2f %04.2fms avg, %04.2fms worst]\n",
      as5600_l.getAngle(), motor_l.shaftVelocity(), motor_l.target, l_temp,
      as5600_r.getAngle(), motor_r.shaftVelocity(), motor_r.target, r_temp,
      integrated_velocity_error, integrated_dyaw_error,
      mpu.pitch(), mpu.dpitch(), 1000. * mpu.avg_update_dt(), 1000. * mpu.worst_update_dt());
    mpu.reset_worst_update_dt();
  }
  
}