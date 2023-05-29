#pragma once

#include "Arduino.h"

// THERMISTOR STUFF
// 3.3V -> thermistor -> [VSENSE] -> [divider_R~1k] resistor -> GND
// LUT starts at 0 and each entry is 5*C hotter.
const float thermistor_lut_temp_spacing = 5.;
const float thermistor_lut_resistances[] = {32.96, 25.58, 20.00, 15.76, 12.51, 10.0, 8.048, 6.518, 5.312, 4.354, 3.588, 2.974, 2.476, 2.072, 1.743, 1.437, 1.250, 1.065, 0.9110, 0.7824, 0.6744, 0.5836, 0.5066};
const int thermistor_lut_entries = 23;

class ThermistorManager
{
public:
    ThermistorManager(int read_pin, int write_pin, float divider_r, float lowpass_rc) : m_read_pin(read_pin), m_write_pin(write_pin), m_divider_r(divider_r), m_lowpass_bw(lowpass_rc), m_temperature(-1.0), m_last_update_t(micros()){
        // Use this "write" pin as a 3.3V driver that's hopefully isolated from the rest of the system.
        pinMode(m_write_pin, OUTPUT);
        digitalWrite(m_write_pin, true);
    }
    
    // TODO: low-pass this value, it's pretty noisy as-is.
    void update()
    {
        // Assume we've externally set analogReadResolution to 12, project-wide.
        float v_sense = min(3.3 - 1E-3, 3.3 * ((float)analogRead(m_read_pin)) / (1<<12));
        // v_sense = 3.3 * R_div / (R_div + R_therm)
        // v_sense * (R_div + R_therm) = 3.3 * R_div
        // v_Sense * R_div + v_sense * R_therm = 3.3 * R_div
        // R_therm = R_div * (3.3 - v_sense) / v_sense
        // final units in kohm
        float thermistor_resistance = (m_divider_r * (3.3 - v_sense) / v_sense) / 1000.;
        
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

        // low-pass update
        uint32_t t = micros();
        float dt_s = ((float)(t - m_last_update_t)) / 1E6;
        m_last_update_t = t;
        float alpha = 1. - dt_s / (m_lowpass_bw + dt_s);
        m_temperature = m_temperature * alpha + (1. - alpha) * temperature;
    }

    float get_temperature(){
        return m_temperature;
    }


private:
    int m_read_pin;
    int m_write_pin;
    float m_divider_r;
    float m_lowpass_bw;
    float m_temperature;
    uint32_t m_last_update_t;
};