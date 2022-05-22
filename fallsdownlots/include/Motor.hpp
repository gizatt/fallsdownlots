#pragma once

#include "Arduino.h"

// Phase chart

/*
const uint32_t N_PHASES = 4;
const uint8_t stepper_phase_chart[16] = {
    1, 0, 1, 0,
    0, 1, 1, 0,
    0, 1, 0, 1,
    1, 0, 0, 1};
*/
const uint32_t PWM_PERIOD_US = 1000;
/*
const uint32_t N_PHASES = 16;
// clang-format off
const uint32_t stepper_phase_chart[N_PHASES * 4] = {
        PWM_PERIOD_US*1.00, PWM_PERIOD_US*0.00, PWM_PERIOD_US*0.50, PWM_PERIOD_US*0.50,
        PWM_PERIOD_US*0.96, PWM_PERIOD_US*0.04, PWM_PERIOD_US*0.69, PWM_PERIOD_US*0.31,
        PWM_PERIOD_US*0.85, PWM_PERIOD_US*0.15, PWM_PERIOD_US*0.85, PWM_PERIOD_US*0.15,
        PWM_PERIOD_US*0.69, PWM_PERIOD_US*0.31, PWM_PERIOD_US*0.96, PWM_PERIOD_US*0.04,
        PWM_PERIOD_US*0.50, PWM_PERIOD_US*0.50, PWM_PERIOD_US*1.00, PWM_PERIOD_US*0.00,
        PWM_PERIOD_US*0.31, PWM_PERIOD_US*0.69, PWM_PERIOD_US*0.96, PWM_PERIOD_US*0.04,
        PWM_PERIOD_US*0.15, PWM_PERIOD_US*0.85, PWM_PERIOD_US*0.85, PWM_PERIOD_US*0.15,
        PWM_PERIOD_US*0.04, PWM_PERIOD_US*0.96, PWM_PERIOD_US*0.69, PWM_PERIOD_US*0.31,
        PWM_PERIOD_US*0.00, PWM_PERIOD_US*1.00, PWM_PERIOD_US*0.50, PWM_PERIOD_US*0.50,
        PWM_PERIOD_US*0.04, PWM_PERIOD_US*0.96, PWM_PERIOD_US*0.31, PWM_PERIOD_US*0.69,
        PWM_PERIOD_US*0.15, PWM_PERIOD_US*0.85, PWM_PERIOD_US*0.15, PWM_PERIOD_US*0.85,
        PWM_PERIOD_US*0.31, PWM_PERIOD_US*0.69, PWM_PERIOD_US*0.04, PWM_PERIOD_US*0.96,
        PWM_PERIOD_US*0.50, PWM_PERIOD_US*0.50, PWM_PERIOD_US*0.00, PWM_PERIOD_US*1.00,
        PWM_PERIOD_US*0.69, PWM_PERIOD_US*0.31, PWM_PERIOD_US*0.04, PWM_PERIOD_US*0.96,
        PWM_PERIOD_US*0.85, PWM_PERIOD_US*0.15, PWM_PERIOD_US*0.15, PWM_PERIOD_US*0.85,
        PWM_PERIOD_US*0.96, PWM_PERIOD_US*0.04, PWM_PERIOD_US*0.31, PWM_PERIOD_US*0.69,
};
//clang-format on
*/
const uint32_t N_PHASES = 32;
// clang-format off
const uint32_t stepper_phase_chart[N_PHASES * 4] = {
        PWM_PERIOD_US*1.00, PWM_PERIOD_US*0.00, PWM_PERIOD_US*0.50, PWM_PERIOD_US*0.50,
        PWM_PERIOD_US*0.99, PWM_PERIOD_US*0.01, PWM_PERIOD_US*0.60, PWM_PERIOD_US*0.40,
        PWM_PERIOD_US*0.96, PWM_PERIOD_US*0.04, PWM_PERIOD_US*0.69, PWM_PERIOD_US*0.31,
        PWM_PERIOD_US*0.92, PWM_PERIOD_US*0.08, PWM_PERIOD_US*0.78, PWM_PERIOD_US*0.22,
        PWM_PERIOD_US*0.85, PWM_PERIOD_US*0.15, PWM_PERIOD_US*0.85, PWM_PERIOD_US*0.15,
        PWM_PERIOD_US*0.78, PWM_PERIOD_US*0.22, PWM_PERIOD_US*0.92, PWM_PERIOD_US*0.08,
        PWM_PERIOD_US*0.69, PWM_PERIOD_US*0.31, PWM_PERIOD_US*0.96, PWM_PERIOD_US*0.04,
        PWM_PERIOD_US*0.60, PWM_PERIOD_US*0.40, PWM_PERIOD_US*0.99, PWM_PERIOD_US*0.01,
        PWM_PERIOD_US*0.50, PWM_PERIOD_US*0.50, PWM_PERIOD_US*1.00, PWM_PERIOD_US*0.00,
        PWM_PERIOD_US*0.40, PWM_PERIOD_US*0.60, PWM_PERIOD_US*0.99, PWM_PERIOD_US*0.01,
        PWM_PERIOD_US*0.31, PWM_PERIOD_US*0.69, PWM_PERIOD_US*0.96, PWM_PERIOD_US*0.04,
        PWM_PERIOD_US*0.22, PWM_PERIOD_US*0.78, PWM_PERIOD_US*0.92, PWM_PERIOD_US*0.08,
        PWM_PERIOD_US*0.15, PWM_PERIOD_US*0.85, PWM_PERIOD_US*0.85, PWM_PERIOD_US*0.15,
        PWM_PERIOD_US*0.08, PWM_PERIOD_US*0.92, PWM_PERIOD_US*0.78, PWM_PERIOD_US*0.22,
        PWM_PERIOD_US*0.04, PWM_PERIOD_US*0.96, PWM_PERIOD_US*0.69, PWM_PERIOD_US*0.31,
        PWM_PERIOD_US*0.01, PWM_PERIOD_US*0.99, PWM_PERIOD_US*0.60, PWM_PERIOD_US*0.40,
        PWM_PERIOD_US*0.00, PWM_PERIOD_US*1.00, PWM_PERIOD_US*0.50, PWM_PERIOD_US*0.50,
        PWM_PERIOD_US*0.01, PWM_PERIOD_US*0.99, PWM_PERIOD_US*0.40, PWM_PERIOD_US*0.60,
        PWM_PERIOD_US*0.04, PWM_PERIOD_US*0.96, PWM_PERIOD_US*0.31, PWM_PERIOD_US*0.69,
        PWM_PERIOD_US*0.08, PWM_PERIOD_US*0.92, PWM_PERIOD_US*0.22, PWM_PERIOD_US*0.78,
        PWM_PERIOD_US*0.15, PWM_PERIOD_US*0.85, PWM_PERIOD_US*0.15, PWM_PERIOD_US*0.85,
        PWM_PERIOD_US*0.22, PWM_PERIOD_US*0.78, PWM_PERIOD_US*0.08, PWM_PERIOD_US*0.92,
        PWM_PERIOD_US*0.31, PWM_PERIOD_US*0.69, PWM_PERIOD_US*0.04, PWM_PERIOD_US*0.96,
        PWM_PERIOD_US*0.40, PWM_PERIOD_US*0.60, PWM_PERIOD_US*0.01, PWM_PERIOD_US*0.99,
        PWM_PERIOD_US*0.50, PWM_PERIOD_US*0.50, PWM_PERIOD_US*0.00, PWM_PERIOD_US*1.00,
        PWM_PERIOD_US*0.60, PWM_PERIOD_US*0.40, PWM_PERIOD_US*0.01, PWM_PERIOD_US*0.99,
        PWM_PERIOD_US*0.69, PWM_PERIOD_US*0.31, PWM_PERIOD_US*0.04, PWM_PERIOD_US*0.96,
        PWM_PERIOD_US*0.78, PWM_PERIOD_US*0.22, PWM_PERIOD_US*0.08, PWM_PERIOD_US*0.92,
        PWM_PERIOD_US*0.85, PWM_PERIOD_US*0.15, PWM_PERIOD_US*0.15, PWM_PERIOD_US*0.85,
        PWM_PERIOD_US*0.92, PWM_PERIOD_US*0.08, PWM_PERIOD_US*0.22, PWM_PERIOD_US*0.78,
        PWM_PERIOD_US*0.96, PWM_PERIOD_US*0.04, PWM_PERIOD_US*0.31, PWM_PERIOD_US*0.69,
        PWM_PERIOD_US*0.99, PWM_PERIOD_US*0.01, PWM_PERIOD_US*0.40, PWM_PERIOD_US*0.60,
};
// clang-format on

// Timer interrupt stepping for *all* stepper motors.
static IntervalTimer stepper_update_timer;
static bool stepper_update_timer_started;
static const uint32_t stepper_update_timer_period_us = 50;
static volatile int n_stepper_motors;
class StepperMotor;
static StepperMotor *steppers[2];

class StepperMotor
{
public:
    uint8_t m_en_A;
    uint8_t m_fwd_A;
    uint8_t m_rev_A;
    uint8_t m_en_B;
    uint8_t m_fwd_B;
    uint8_t m_rev_B;

    volatile uint8_t m_phase;
    volatile float m_speed;
    volatile uint32_t m_stepper_update_timer_last_update;
    volatile uint32_t m_step_period_us;

    static void do_timer_setup()
    {
        n_stepper_motors = 0;
        stepper_update_timer.begin(advance_steppers, stepper_update_timer_period_us);
    }
    static void advance_steppers()
    {
        uint32_t t = micros();
        for (int i = 0; i < n_stepper_motors; i++)
        {
            // PWM is implemented by breaking time (from micros()) into PWM_PERIOD_US
            // chunks, and being "on" for whatever period of that is requested. When
            // the update period gets sufficiently big, aliasing will begin corrupting this
            // signal; but since we update an order of magnitude faster than we're able to
            // actually spin the motor, that winds up being OK.
            StepperMotor *stepper = steppers[i];
            uint32_t pwm_frac = t % PWM_PERIOD_US;
            digitalWrite(stepper->m_fwd_A, stepper_phase_chart[stepper->m_phase * 4 + 0] < pwm_frac);
            digitalWrite(stepper->m_rev_A, stepper_phase_chart[stepper->m_phase * 4 + 1] < pwm_frac);
            digitalWrite(stepper->m_fwd_B, stepper_phase_chart[stepper->m_phase * 4 + 2] < pwm_frac);
            digitalWrite(stepper->m_rev_B, stepper_phase_chart[stepper->m_phase * 4 + 3] < pwm_frac);

            uint32_t dt = t - stepper->m_stepper_update_timer_last_update;
            if (dt > stepper->m_step_period_us)
            {
                if (stepper->m_speed > 0.)
                {
                    stepper->m_phase = (stepper->m_phase + 1) % N_PHASES;
                }
                else
                {
                    // Equivalent to stepping backwards
                    stepper->m_phase = (stepper->m_phase + (N_PHASES - 1)) % N_PHASES;
                }
                stepper->m_stepper_update_timer_last_update = t;
            }
        }
    }

    const uint32_t UPDATE_PERIOD = 100; // ms
    const float MAX_SPEED = 500;        // updates / sec

    StepperMotor(uint8_t en_A, uint8_t fwd_A, uint8_t rev_A,
                 uint8_t en_B, uint8_t fwd_B, uint8_t rev_B) : m_en_A(en_A), m_fwd_A(fwd_A), m_rev_A(rev_A),
                                                               m_en_B(en_B), m_fwd_B(fwd_B), m_rev_B(rev_B),
                                                               m_phase(0), m_speed(0.0),
                                                               m_stepper_update_timer_last_update(micros()), m_step_period_us(1E6),
                                                               m_enable(1), m_last_update_t(millis())
    {
        pinMode(m_en_A, OUTPUT);
        pinMode(m_en_B, OUTPUT);
        pinMode(m_fwd_A, OUTPUT);
        pinMode(m_fwd_B, OUTPUT);
        pinMode(m_rev_A, OUTPUT);
        pinMode(m_rev_B, OUTPUT);

        if (!stepper_update_timer_started)
        {
            do_timer_setup();
        }

        steppers[n_stepper_motors] = this;
        n_stepper_motors += 1;
    }

    /*
     Clips to range -1 to 1.
    */
    void set_speed(float speed)
    {
        noInterrupts();
        m_speed = max(min(speed, 1.0), -1.0);
        m_enable = (abs(m_speed) >= 1E-5);
        // Account for microstepping in speed setting.
        m_step_period_us = (uint32_t)max(1., 1E6 / (MAX_SPEED * abs(m_speed))) / (N_PHASES / 4);
        interrupts();
    }

    void update(uint32_t t)
    {
        if (t - m_last_update_t > UPDATE_PERIOD)
        {
            m_last_update_t = t;

            digitalWrite(m_en_A, m_enable);
            digitalWrite(m_en_B, m_enable);
        }
    }

private:
    bool m_enable;
    uint32_t m_last_update_t;
};

class DCMotor
{
public:
    const uint32_t UPDATE_PERIOD = 1; // ms
    uint8_t ANALOG_WRITE_RESOLUTION_BITS = 12;
    uint32_t ANALOG_INPUT_MAX = (1 << ANALOG_WRITE_RESOLUTION_BITS) - 1;
    // Input -DEADBAND -> DEADBAND = 0 input
    // (i.e. remap 0-to-1 to DEADBAND-to-1.)
    float DEADBAND = 0.3;

    DCMotor(uint8_t en_pin, uint8_t fwd_pin, uint8_t rev_pin) : m_en_pin(en_pin), m_fwd_pin(fwd_pin), m_rev_pin(rev_pin),
                                                                m_speed(0.0), m_last_update_t(millis())
    {
        pinMode(m_en_pin, OUTPUT);
        analogWriteResolution(ANALOG_WRITE_RESOLUTION_BITS);
        // analogWriteFrequency(en_pin, 375000);
        pinMode(m_fwd_pin, OUTPUT);
        pinMode(m_rev_pin, OUTPUT);
        step_pins();
    }

    void step_pins()
    {
        analogWrite(m_en_pin, (int)(abs(m_speed) * ANALOG_INPUT_MAX));
        if (m_speed > 1E-6)
        {
            digitalWrite(m_fwd_pin, 1);
            digitalWrite(m_rev_pin, 0);
        }
        else if (m_speed < -1E-6)
        {
            digitalWrite(m_fwd_pin, 0);
            digitalWrite(m_rev_pin, 1);
        }
        else
        {
            digitalWrite(m_fwd_pin, 0);
            digitalWrite(m_rev_pin, 0);
        }
    }

    /*
     Clips to range -1 to 1.
    */
    void set_speed(float speed)
    {
        m_speed = max(min(speed, 1.0), -1.0);
        // Apply deadband remapping:
        // 0-1 to DEADBAND-to-1.
        // (abs(commanded) - DEADBAND) / (1. - DEADBAND) = desired / 1.
        // abs(command) = desired * (1. - DEADBAND) + DEADBAND
        if (m_speed > 1E-5)
        {
            m_speed = m_speed * (1. - DEADBAND) + DEADBAND;
        }
        else if (m_speed < -1E-5)
        {
            m_speed = m_speed * (1. - DEADBAND) - DEADBAND;
        }
        else
        {
            m_speed = 0.;
        }
    }

    void update(uint32_t t)
    {
        if (t - m_last_update_t > UPDATE_PERIOD)
        {
            m_last_update_t = t;
            step_pins();
        }
    }

private:
    uint8_t m_en_pin;
    uint8_t m_fwd_pin;
    uint8_t m_rev_pin;
    float m_speed;
    uint32_t m_last_update_t;
};