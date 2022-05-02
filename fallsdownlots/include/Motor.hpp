#pragma once

#include "Arduino.h"

class Motor
{
public:
    uint32_t update_period = 1; // ms
    uint8_t ANALOG_WRITE_RESOLUTION_BITS = 12;
    uint32_t ANALOG_INPUT_MAX = (1 << ANALOG_WRITE_RESOLUTION_BITS) - 1;
    // Input -DEADBAND -> DEADBAND = 0 input
    // (i.e. remap 0-to-1 to DEADBAND-to-1.)
    float DEADBAND = 0.3;

    Motor(uint8_t pwm_pin, uint8_t fwd_pin, uint8_t rev_pin) : m_pwm_pin(pwm_pin), m_fwd_pin(fwd_pin), m_rev_pin(rev_pin),
                                                               m_speed(0.0), m_last_update_t(millis())
    {
        pinMode(m_pwm_pin, OUTPUT);
        analogWriteResolution(ANALOG_WRITE_RESOLUTION_BITS);
        // analogWriteFrequency(pwm_pin, 375000);
        pinMode(m_fwd_pin, OUTPUT);
        pinMode(m_rev_pin, OUTPUT);
        write_pins();
    }

    void write_pins()
    {
        analogWrite(m_pwm_pin, (int)(abs(m_speed) * ANALOG_INPUT_MAX));
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
        if (t - m_last_update_t > update_period)
        {
            m_last_update_t = t;
            write_pins();
        }
    }

private:
    uint8_t m_pwm_pin;
    uint8_t m_fwd_pin;
    uint8_t m_rev_pin;
    float m_speed;
    bool m_led_state;
    uint32_t m_last_update_t;
};