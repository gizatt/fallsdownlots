/**
 * Comprehensive BLDC motor control example using Hall sensor
 *
 * Using serial terminal user can send motor commands and configure the motor and FOC in real-time:
 * - configure PID controller constants
 * - change motion control loops
 * - monitor motor variabels
 * - set target values
 * - check all the configuration values
 *
 * See more info in docs.simplefoc.com/commander_interface
 */
#include <SimpleFOC.h>

#define V1_PIN A1

char print_buf[256];

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(7, 12.0, 450); // Nominally 250, but this value passes the 0-torque-makes-it-feel-smooth test.
// BLDCMotor motor = BLDCMotor(7, 1.0, 350); // Nominally 250, but this value passes the 0-torque-makes-it-feel-smooth test.
BLDCDriver3PWM driver = BLDCDriver3PWM(5, 6, 9, 10);
 MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
//MagneticSensorAnalog sensor = MagneticSensorAnalog(A1, 5, 3800);
// Stepper motor & driver instance
// StepperMotor motor = StepperMotor(50);
// StepperDriver4PWM driver = StepperDriver4PWM(9, 5, 10, 6,  8);

// // hall sensor instance
// HallSensor sensor = HallSensor(2, 3, 4, 11);

// commander interface
Commander command = Commander(Serial);
void doLimitCurrent(char *cmd) { command.scalar(&motor.current_limit, cmd); }
// target variable
bool do_sinusoid = false;
float target_velocity = 1.;
float sinusoid_amplitude = 1.;
float sinusoid_period = 1.;
void doTarget(char *cmd)
{
  command.scalar(&target_velocity, cmd);
  do_sinusoid = false;
}
void doTargetSinusoid(char *cmd)
{
  command.scalar(&sinusoid_amplitude, cmd);
  do_sinusoid = true;
}
void doTargetSinusoidPeriod(char *cmd)
{
  command.scalar(&sinusoid_period, cmd);
}

void doKv(char *cmd)
{
  command.scalar(&motor.KV_rating, cmd);
}
void doPID_P(char *cmd)
{
  command.scalar(&motor.PID_velocity.P, cmd);
  Serial.println("PID:");
  Serial.println(motor.PID_velocity.P);
  Serial.println(motor.PID_velocity.I);
  Serial.println(motor.PID_velocity.D);
}
void doPID_D(char *cmd)
{
  command.scalar(&motor.PID_velocity.D, cmd);
  Serial.println("PID:");
  Serial.println(motor.PID_velocity.P);
  Serial.println(motor.PID_velocity.I);
  Serial.println(motor.PID_velocity.D);
}
void doPID_I(char *cmd)
{
  command.scalar(&motor.PID_velocity.I, cmd);
  Serial.println("PID:");
  Serial.println(motor.PID_velocity.P);
  Serial.println(motor.PID_velocity.I);
  Serial.println(motor.PID_velocity.D);
}
void doPID_Tf(char *cmd)
{
  command.scalar(&motor.LPF_velocity.Tf, cmd);
}

void doLimit(char *cmd) { command.scalar(&motor.voltage_limit, cmd); }

// 3.3V -> Motor Thermistor -> 1K resistor -> GND
#define MOTOR_THERMISTOR_PIN A5
// LUT starts at 0 and each entry is 5*C hotter.
const float thermistor_lut_temp_spacing = 5.;
const float thermistor_lut_resistances[] = {32.96, 25.58, 20.00, 15.76, 12.51, 10.0, 8.048, 6.518, 5.312, 4.354, 3.588, 2.974, 2.476, 2.072, 1.743, 1.437, 1.250, 1.065, 0.9110, 0.7824, 0.6744, 0.5836, 0.5066};
const int thermistor_lut_entries = 23;

float get_motor_temperature()
{
  float sensed_voltage = min(3.299, 3.3 * float(analogRead(MOTOR_THERMISTOR_PIN)) / 1023.);
  // sensed_voltage = 3.3 * 1k / (1k + motor thermistor)
  // motor_thermistor = 3.3 * 1K / (sensed_voltage) - 1k
  // Units in Kohm
  float thermistor_resistance = ((3.3 / sensed_voltage) * 1000. - 1000.) / 1000.0;

  // See where we fall in the lut.
  float temperature;
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
  snprintf(print_buf, 100, "V %0.2f R %0.2f T %0.2f\n", sensed_voltage, thermistor_resistance, temperature);
  Serial.print(print_buf);
}

void setup()
{
  analogReadResolution(12);
  sensor.init();

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12.;
  // driver.pwm_frequency = 128000;
  driver.init();
  // link driver
  motor.linkDriver(&driver);
  motor.linkSensor(&sensor);

  // set control loop type to be used
  // sensor.L
  //  open loop control config
  // motor.controller = MotionControlType::velocity_openloop;
  motor.torque_controller = TorqueControlType::voltage;
  // motor.controller = MotionControlType::velocity;
  motor.PID_velocity.output_ramp = 1000;
  motor.controller = MotionControlType::torque;

  // choose FOC modulation (optional) - SinePWM or SpaceVectorPWM
  // motor.foc_modulation = FOCModulationType::SinePWM;

  // limiting voltage
  motor.voltage_limit = 7.4; // Volts
  // or current  - if phase resistance provided
  motor.current_limit = 1.1; // Amps
  motor.PID_velocity.P = 0.25;
  motor.PID_velocity.D = 0.001;
  motor.PID_velocity.I = 10;
  motor.LPF_velocity.Tf = 0.1;
  // motor.LPF_angle.Tf = 0.01;
  // motor.LPF_current_d.Tf = 0.01;
  // motor.LPF_current_q.Tf = 0.01;

  // use monitoring with serial for motor init
  // monitoring port
  Serial.begin(115200);
  // comment out if not needed
  motor.useMonitoring(Serial);
  // initialise motor
  motor.init();
  // align encoder and start FOC
  motor.initFOC();

  // set the inital target value
  motor.target = 2;
  motor.velocity_limit = 1000;

  // add target command T
  command.add('T', doTarget, "target velocity");
  command.add('S', doTargetSinusoid, "target sinusoid amplitude");
  command.add('P', doTargetSinusoidPeriod, "target sinusoid period");
  command.add('L', doLimit, "voltage limit");
  command.add('p', doPID_P, "PID P");
  command.add('i', doPID_I, "PID I");
  command.add('d', doPID_D, "PID D");
  command.add('t', doPID_Tf, "PID TF");
  command.add('C', doLimitCurrent, "Current limit");
  command.add('K', doKv, "Kv");

  // Run user commands to configure and the motor (find the full command list in docs.simplefoc.com)
  Serial.println("Motor ready!");
  Serial.println("Set target velocity [rad/s]");

  _delay(1000);
}
float v1 = 0.;
int k = 0;

uint32_t last_temp_check_t = 0;
void loop()
{

  sensor.update();
  motor.loopFOC();

  uint32_t t = micros();

  if (do_sinusoid)
  {
    target_velocity = sin((2. * PI / sinusoid_period) * ((float)t) / 1E6) * sinusoid_amplitude;
  }
  motor.move(target_velocity);

  float r1 = 10000.;
  float r2 = 4700.;
  float v1_new = (3.3 * ((float)analogRead(V1_PIN)) / 1024.) * (r1 + r2) / (r2);
  v1 = v1_new * 0.1 + v1 * 0.9;

  command.run();

  if (t - last_temp_check_t > 1000 * 100)
  {
    get_motor_temperature();
    last_temp_check_t = t;
  }
}
