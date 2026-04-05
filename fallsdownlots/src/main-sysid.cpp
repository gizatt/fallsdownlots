/**
 * SysID firmware - single motor, serial streaming.
 *
 * Uses right motor hardware (Wire/AS5600, driver pins 6/9/10/5).
 *
 * Serial protocol (115200 baud, newline-terminated ASCII):
 *
 *   Python -> Board:
 *     T<float>               Set constant target. Volts in torque mode, rad/s in
 *                            velocity mode. Cancels any running chirp.
 *     M0                     Switch to torque (voltage) control mode.
 *     M1                     Switch to velocity control mode.
 *     P<float>               Velocity PID P gain.
 *     I<float>               Velocity PID I gain.
 *     D<float>               Velocity PID D gain.
 *     F<float>               Velocity LPF time constant (seconds).
 *     C<amp>,<f0>,<f1>,<dur> Start log-chirp. Firmware generates the signal.
 *                            amp in same units as T. f0/f1 in Hz. dur in seconds.
 *
 *   Board -> Python (CSV at loop rate, ~500-1000 Hz depending on I2C):
 *     <t_us>,<angle_rad>,<vel_rad_s>,<cmd>\n
 *
 *   Comment lines starting with '#' are status/error messages, not data.
 */

#include <SimpleFOC.h>

// Right motor hardware (matches main-fallsdownlots.cpp).
MagneticSensorI2C as5600 = MagneticSensorI2C(AS5600_I2C);
BLDCMotor motor = BLDCMotor(7, 12.0, 450);
BLDCDriver3PWM driver = BLDCDriver3PWM(6, 9, 10, 5);

// Control state.
enum Mode { TORQUE, VELOCITY };
Mode mode = TORQUE;
float target = 0.0;

// Back-EMF feedforward gain (V·s/rad).
// Derived from motor KV: Kb = 60 / (2π × KV_rpm_per_volt).
// Set to 0 to disable.
float Kb = 60.0f / (TWO_PI * 450.0f);  // ≈ 0.0212 V·s/rad

// Chirp state.
bool chirp_active = false;
float chirp_amp = 0.0;
float chirp_f0 = 0.0;
float chirp_f1 = 0.0;
float chirp_duration = 0.0;
float chirp_phase = 0.0;
float chirp_f_current = 0.0;
uint32_t chirp_start_us = 0;

void start_chirp(float amp, float f0, float f1, float dur) {
  chirp_amp = amp;
  chirp_f0 = f0;
  chirp_f1 = f1;
  chirp_duration = dur;
  chirp_phase = 0.0;
  chirp_f_current = f0;
  chirp_start_us = micros();
  chirp_active = true;
}

void set_mode(Mode new_mode) {
  mode = new_mode;
  target = 0.0;
  chirp_active = false;
  if (mode == VELOCITY) {
    motor.controller = MotionControlType::velocity;
    motor.PID_velocity.reset();
  } else {
    motor.controller = MotionControlType::torque;
  }
  Serial.printf("# mode=%s\n", mode == TORQUE ? "torque" : "velocity");
}

// Parse one command line (without the trailing newline).
void handle_command(const char *line) {
  if (line[0] == 'T') {
    target = atof(line + 1);
    chirp_active = false;
  } else if (line[0] == 'M') {
    set_mode(line[1] == '1' ? VELOCITY : TORQUE);
  } else if (line[0] == 'P') {
    motor.PID_velocity.P = atof(line + 1);
    Serial.printf("# PID P=%.4f\n", motor.PID_velocity.P);
  } else if (line[0] == 'I') {
    motor.PID_velocity.I = atof(line + 1);
    Serial.printf("# PID I=%.4f\n", motor.PID_velocity.I);
  } else if (line[0] == 'D') {
    motor.PID_velocity.D = atof(line + 1);
    Serial.printf("# PID D=%.4f\n", motor.PID_velocity.D);
  } else if (line[0] == 'F') {
    motor.LPF_velocity.Tf = atof(line + 1);
    Serial.printf("# LPF Tf=%.4f\n", motor.LPF_velocity.Tf);
  } else if (line[0] == 'C') {
    // sscanf %f is unreliable on ARM Arduino; parse with strtof instead.
    char *p = (char *)(line + 1);
    char *end;
    float amp = strtof(p, &end);
    if (end == p || *end != ',') { Serial.println("# bad chirp args"); return; }
    p = end + 1;
    float f0 = strtof(p, &end);
    if (end == p || *end != ',') { Serial.println("# bad chirp args"); return; }
    p = end + 1;
    float f1 = strtof(p, &end);
    if (end == p || *end != ',') { Serial.println("# bad chirp args"); return; }
    p = end + 1;
    float dur = strtof(p, &end);
    if (end == p)                { Serial.println("# bad chirp args"); return; }
    start_chirp(amp, f0, f1, dur);
    Serial.printf("# chirp amp=%.2f f0=%.2f f1=%.2f dur=%.2f\n", amp, f0, f1, dur);
  } else if (line[0] == 'B') {
    Kb = atof(line + 1);
    Serial.printf("# Kb=%.5f V·s/rad (set to 0 to disable feedforward)\n", Kb);
  } else {
    Serial.printf("# unknown command: %s\n", line);
  }
}

// Serial line buffer.
static char line_buf[64];
static int line_len = 0;

void read_serial_commands() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (line_len > 0) {
        line_buf[line_len] = '\0';
        handle_command(line_buf);
        line_len = 0;
      }
    } else if (line_len < (int)sizeof(line_buf) - 1) {
      line_buf[line_len++] = c;
    }
  }
}

void setup() {
  Serial.begin(115200);

  Wire.setClock(400000);
  Wire.begin();

  as5600.init(&Wire);

  driver.voltage_power_supply = 7.4;
  driver.init();
  motor.linkDriver(&driver);
  motor.linkSensor(&as5600);

  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::torque;
  motor.voltage_limit = 7.4;
  motor.current_limit = 1.0;

  // Velocity PID defaults — reasonable starting point, tune via P/I/D/F commands.
  motor.PID_velocity.P = 0.25;
  motor.PID_velocity.I = 5.0;
  motor.PID_velocity.D = 0.001;
  motor.PID_velocity.output_ramp = 1000;
  motor.LPF_velocity.Tf = 0.005;  // 5ms (~32 Hz corner) — reduced from 20ms to cut observation lag.

  motor.velocity_limit = 1000;
  motor.target = 0;

  motor.init();
  motor.initFOC();

  Serial.println("# sysid ready. Commands: T<val> M<0|1> P I D F B<Kb> C<amp,f0,f1,dur>");
}

uint32_t last_loop_us = 0;

void loop() {
  uint32_t t = micros();
  float dt = (t - last_loop_us) * 1e-6f;
  last_loop_us = t;

  read_serial_commands();

  as5600.update();
  motor.loopFOC();

  // Update chirp.
  if (chirp_active) {
    float elapsed = (t - chirp_start_us) * 1e-6f;
    if (elapsed >= chirp_duration) {
      chirp_active = false;
      target = 0.0;
    } else if (chirp_f0 > 0 && chirp_f1 > chirp_f0) {
      // Log-chirp: frequency advances exponentially.
      float k = logf(chirp_f1 / chirp_f0) / chirp_duration;
      chirp_phase += TWO_PI * chirp_f_current * dt;
      chirp_f_current = chirp_f0 * expf(k * elapsed);
      target = chirp_amp * sinf(chirp_phase);
    }
  }

  // Back-EMF feedforward in torque mode: adds Kb*ω to compensate for
  // speed-dependent voltage drop, making voltage→torque more linear.
  float v_applied = target;
  if (mode == TORQUE) {
    v_applied += Kb * motor.shaftVelocity();
  }
  motor.move(v_applied);

  // Stream the raw target (not v_applied) so the sysID analysis sees the
  // commanded "torque proxy" as the input, not the FF-compensated voltage.
  Serial.printf("%lu,%.5f,%.4f,%.4f\n",
    t,
    as5600.getAngle(),
    motor.shaftVelocity(),
    target);
}
