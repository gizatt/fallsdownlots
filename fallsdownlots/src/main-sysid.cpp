/**
 * SysID firmware — single motor, serial streaming.
 *
 * Flash with:
 *   pio run -e sysid_r -t upload   (right motor: Wire, pins 6/9/10/5)
 *   pio run -e sysid_l -t upload   (left motor:  Wire_l pins 24/25, pins 12/13/26/11)
 *
 * Serial protocol (115200 baud, newline-terminated ASCII):
 *
 *   Python -> Board:
 *     T<float>               Set constant target. Volts in torque mode, rad/s
 *                            in velocity mode. Cancels any running chirp.
 *     M0                     Switch to torque (voltage) control mode.
 *     M1                     Switch to velocity control mode.
 *     P<float>               Velocity PID P gain.
 *     I<float>               Velocity PID I gain.
 *     D<float>               Velocity PID D gain.
 *     F<float>               Velocity LPF time constant (seconds).
 *     C<amp>,<f0>,<f1>,<dur> Start log-chirp. amp in same units as T.
 *                            f0/f1 in Hz. dur in seconds.
 *     K<float>               Start encoder calibration at the given D-axis
 *                            voltage (volts). Steps through one full
 *                            mechanical rotation forward then backward.
 *
 *   Board -> Python during normal operation (CSV at loop rate ~500 Hz):
 *     <t_us>,<angle_rad>,<vel_rad_s>,<cmd>\n
 *     (angle_rad is LUT-corrected)
 *
 *   Board -> Python during encoder calibration:
 *     E,<F|R>,<ref_elec_rad>,<raw_mech_rad>\n   (raw = no LUT applied)
 *     # CAL_START ...
 *     # CAL_REVERSE
 *     # CAL_DONE
 *
 *   Comment lines starting with '#' are status/error messages, not data.
 */

#include <SimpleFOC.h>
#include "encoder_lut_r.h"
#include "encoder_lut_l.h"
#include "cogging_lut_r.h"
#include "cogging_lut_l.h"

static constexpr int N_POLE_PAIRS = 7;

// Interpolate a 128-point LUT keyed by electrical angle in [0, 2pi).
static float lut_interp(float elec_angle, const float *lut, int n) {
  float pos = fmodf(elec_angle, TWO_PI) * n / TWO_PI;
  if (pos < 0.0f) pos += n;
  int i0 = (int)pos % n;
  int i1 = (i0 + 1) % n;
  float f = pos - (int)pos;
  return lut[i0] + f * (lut[i1] - lut[i0]);
}

// -----------------------------------------------------------------------
// AS5600 with 128-point LUT correction.
// getRawSensorAngle() bypasses the LUT — used during calibration so we
// record the true uncorrected sensor reading.
// -----------------------------------------------------------------------
struct LutAS5600 : public MagneticSensorI2C {
  const float *lut;
  int lut_n;
  LutAS5600(const float *lut, int lut_n)
    : MagneticSensorI2C(AS5600_I2C), lut(lut), lut_n(lut_n) {}

  float getSensorAngle() override {
    float raw = MagneticSensorI2C::getSensorAngle();
    float pos = raw * lut_n / TWO_PI;
    int   i0  = (int)pos % lut_n;
    int   i1  = (i0 + 1) % lut_n;
    float f   = pos - (int)pos;
    return raw - (lut[i0] + f * (lut[i1] - lut[i0]));
  }

  float getRawSensorAngle() {
    return MagneticSensorI2C::getSensorAngle();
  }
};

// -----------------------------------------------------------------------
// Hardware selection via build flag: -DMOTOR_LEFT or -DMOTOR_RIGHT
// -----------------------------------------------------------------------
#if defined(MOTOR_LEFT)
  TwoWire Wire_motor(NRF_TWIM1, NRF_TWIS1,
                     SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQn, 24, 25);
  LutAS5600        as5600(ENCODER_LUT_L, 128);
  BLDCDriver3PWM   driver(12, 13, 26, 11);
  static const float *COGGING_LUT = COGGING_LUT_L;
#else  // MOTOR_RIGHT (default)
  TwoWire         &Wire_motor = Wire;
  LutAS5600        as5600(ENCODER_LUT_R, 128);
  BLDCDriver3PWM   driver(6, 9, 10, 5);
  static const float *COGGING_LUT = COGGING_LUT_R;
#endif

BLDCMotor motor(N_POLE_PAIRS, 12.0, 450);

// -----------------------------------------------------------------------
// Normal sysid state
// -----------------------------------------------------------------------

enum Mode { TORQUE, VELOCITY };
Mode  mode   = TORQUE;
float target = 0.0f;

bool     chirp_active = false;
float    chirp_amp, chirp_f0, chirp_f1, chirp_duration;
float    chirp_phase, chirp_f_current;
uint32_t chirp_start_us;

void start_chirp(float amp, float f0, float f1, float dur) {
  chirp_amp       = amp;
  chirp_f0        = f0;
  chirp_f1        = f1;
  chirp_duration  = dur;
  chirp_phase     = 0.0f;
  chirp_f_current = f0;
  chirp_start_us  = micros();
  chirp_active    = true;
}

void set_mode(Mode new_mode) {
  mode   = new_mode;
  target = 0.0f;
  chirp_active = false;
  motor.controller = (mode == VELOCITY)
    ? MotionControlType::velocity
    : MotionControlType::torque;
  if (mode == VELOCITY) motor.PID_velocity.reset();
  Serial.printf("# mode=%s\n", mode == TORQUE ? "torque" : "velocity");
}

// -----------------------------------------------------------------------
// Encoder calibration state machine
// -----------------------------------------------------------------------
//
// Steps the D-axis (Ud=voltage, Uq=0) through one full mechanical rotation
// forward then backward. The rotor follows the reference regardless of
// encoder errors, giving clean (ref_elec, raw_mech) pairs.
// loopFOC/move are bypassed; raw (uncorrected) angle is streamed.
//
// Phase ordering: if raw_mech decreases while ref_elec increases, two
// motor phase wires are swapped. Swap them and recalibrate.

static constexpr int      CAL_STEPS     = 1000;
static constexpr float    CAL_STEP_ELEC = TWO_PI * N_POLE_PAIRS / (float)CAL_STEPS;
static constexpr uint32_t CAL_SETTLE_US = 5000;  // 5 ms/step → ~10 s total

enum CalState { CAL_IDLE, CAL_FWD, CAL_BWD };
CalState cal_state    = CAL_IDLE;
float    cal_voltage  = 3.0f;
float    cal_ref_elec = 0.0f;
int      cal_step     = 0;
uint32_t cal_last_us  = 0;

void start_cal(float voltage) {
  cal_voltage  = voltage;
  cal_ref_elec = 0.0f;
  cal_step     = 0;
  cal_state    = CAL_FWD;
  motor.setPhaseVoltage(0.0f, cal_voltage, cal_ref_elec);
  cal_last_us  = micros();
  Serial.printf("# CAL_START voltage=%.2f steps=%d settle_us=%lu\n",
                cal_voltage, CAL_STEPS, (unsigned long)CAL_SETTLE_US);
}

void update_cal() {
  if (micros() - cal_last_us < CAL_SETTLE_US) return;
  cal_last_us = micros();

  // Motor has settled at cal_ref_elec — read raw (uncorrected) angle.
  char dir = (cal_state == CAL_FWD) ? 'F' : 'R';
  Serial.printf("E,%c,%.5f,%.5f\n", dir, cal_ref_elec, as5600.getRawSensorAngle());

  cal_ref_elec += (cal_state == CAL_FWD) ? CAL_STEP_ELEC : -CAL_STEP_ELEC;

  if (++cal_step >= CAL_STEPS) {
    cal_step = 0;
    if (cal_state == CAL_FWD) {
      cal_state = CAL_BWD;
      Serial.println("# CAL_REVERSE");
    } else {
      cal_state = CAL_IDLE;
      motor.setPhaseVoltage(0.0f, 0.0f, 0.0f);
      Serial.println("# CAL_DONE");
      return;
    }
  }

  motor.setPhaseVoltage(0.0f, cal_voltage, cal_ref_elec);
}

// -----------------------------------------------------------------------
// Serial command parser
// -----------------------------------------------------------------------

void handle_command(const char *line) {
  if (line[0] == 'K') {
    float v = atof(line + 1);
    if (v <= 0.0f || v > 7.4f) {
      Serial.printf("# bad cal voltage %.2f (0 < V <= 7.4)\n", v);
      return;
    }
    target = 0.0f;
    chirp_active = false;
    motor.controller = MotionControlType::torque;
    start_cal(v);
  } else if (line[0] == 'T') {
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
    char *p = (char *)(line + 1), *end;
    float amp = strtof(p, &end); if (end == p || *end != ',') { Serial.println("# bad chirp args"); return; } p = end + 1;
    float f0  = strtof(p, &end); if (end == p || *end != ',') { Serial.println("# bad chirp args"); return; } p = end + 1;
    float f1  = strtof(p, &end); if (end == p || *end != ',') { Serial.println("# bad chirp args"); return; } p = end + 1;
    float dur = strtof(p, &end); if (end == p)                { Serial.println("# bad chirp args"); return; }
    start_chirp(amp, f0, f1, dur);
    Serial.printf("# chirp amp=%.2f f0=%.2f f1=%.2f dur=%.2f\n", amp, f0, f1, dur);
  } else {
    Serial.printf("# unknown command: %s\n", line);
  }
}

static char line_buf[64];
static int  line_len = 0;

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

// -----------------------------------------------------------------------
// Setup / loop
// -----------------------------------------------------------------------

void setup() {
  Serial.begin(115200);

  Wire_motor.setClock(400000);
  Wire_motor.begin();
  as5600.init(&Wire_motor);

  driver.voltage_power_supply = 7.4;
  driver.init();
  motor.linkDriver(&driver);
  motor.linkSensor(&as5600);

  motor.torque_controller        = TorqueControlType::voltage;
  motor.controller               = MotionControlType::torque;
  motor.voltage_limit            = 7.4;
  motor.current_limit            = 1.0;

  motor.PID_velocity.P           = 0.03;
  motor.PID_velocity.I           = 1.0;
  motor.PID_velocity.D           = 0.0;
  motor.PID_velocity.output_ramp = 1000;
  motor.LPF_velocity.Tf          = 0.01;

  motor.velocity_limit = 1000;
  motor.target         = 0;

  motor.init();
  motor.initFOC();

#if defined(MOTOR_LEFT)
  Serial.println("# sysid ready (LEFT motor).");
#else
  Serial.println("# sysid ready (RIGHT motor).");
#endif
  Serial.println("# Commands: T<val> M<0|1> P I D F C<amp,f0,f1,dur> K<voltage>");
}

uint32_t last_loop_us = 0;

void loop() {
  uint32_t t  = micros();
  float    dt = (t - last_loop_us) * 1e-6f;
  last_loop_us = t;

  read_serial_commands();
  as5600.update();

  // During calibration we drive the motor directly and skip FOC entirely.
  if (cal_state != CAL_IDLE) {
    update_cal();
    return;
  }

  motor.loopFOC();

  // Update chirp.
  if (chirp_active) {
    float elapsed = (t - chirp_start_us) * 1e-6f;
    if (elapsed >= chirp_duration) {
      chirp_active = false;
      target = 0.0f;
    } else if (chirp_f0 > 0 && chirp_f1 > chirp_f0) {
      float k = logf(chirp_f1 / chirp_f0) / chirp_duration;
      chirp_phase += TWO_PI * chirp_f_current * dt;
      chirp_f_current = chirp_f0 * expf(k * elapsed);
      target = chirp_amp * sinf(chirp_phase);
    }
  }

  motor.move(target);

  // Apply cogging feedforward. Added to voltage.q here so loopFOC picks it
  // up on the next iteration (1-sample delay, negligible at 500 Hz).
  motor.voltage.q += lut_interp(motor.electricalAngle(), COGGING_LUT, 128);

  // Stream LUT-corrected angle; include voltage.q for cogging sweep analysis.
  Serial.printf("%lu,%.5f,%.4f,%.4f,%.4f\n",
    t,
    as5600.getAngle(),
    motor.shaftVelocity(),
    target,
    motor.voltage.q);
}
