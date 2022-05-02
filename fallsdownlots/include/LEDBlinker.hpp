#pragma once

#include "Arduino.h"

class LEDBlinker
{
public:
    uint32_t blink_period = 250; // ms
    LEDBlinker(int led_pin) : m_led_pin(led_pin), m_led_state(false), m_last_update_t(millis())
    {
        pinMode(m_led_pin, OUTPUT);
        update_led();
    }

    void update_led()
    {
        if (m_led_state)
        {
            digitalWrite(m_led_pin, HIGH);
        }
        else
        {
            digitalWrite(m_led_pin, LOW);
        }
    }

    void update(uint32_t t)
    {
        if (t - m_last_update_t > blink_period)
        {
            m_last_update_t = t;
            m_led_state = !m_led_state;
            update_led();
        }
    }

private:
    int m_led_pin;
    bool m_led_state;
    uint32_t m_last_update_t;
};