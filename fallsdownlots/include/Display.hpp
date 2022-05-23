#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

class Display
{
public:
    const uint8_t SCREEN_WIDTH = 128;       // OLED display width, in pixels
    const uint8_t SCREEN_HEIGHT = 32;       // OLED display height, in pixels
    const uint8_t SCREEN_ADDRESS = 0x3C;    ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
    const uint8_t OLED_RESET = 4;           // Reset pin # (or -1 if sharing Arduino reset pin)
    const static uint8_t BUF_LEN = 64;      // String buffer for format string printing
    const uint32_t UPDATE_PERIOD = 100;     // ms
    const uint32_t RECONNECT_PERIOD = 1000; // ms

    Display(usb_serial_class *serial = nullptr) : m_display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET),
                                                  m_serial(serial), m_have_screen(false), m_last_update_t(millis())
    {
        m_buf[0] = 0; // Null terminate buffer.
        try_connect();
    }

    bool connected()
    {
        return m_have_screen;
    }

    bool try_connect()
    {
        m_have_screen = m_display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
        if (!m_have_screen && m_serial)
        {
            m_serial->printf("Could not connect to screen.");
        }
        else
        {
            m_display.display();
            // Clear the buffer
            m_display.clearDisplay();
            // Draw a single pixel in white
            m_display.drawPixel(10, 10, SSD1306_WHITE);
            m_display.display();
        }
        return m_have_screen;
    }

    template <typename... Args>
    bool draw_text(Args &&...args)
    {
        if (!m_have_screen)
            return false;

        snprintf(m_buf, BUF_LEN, args...);
        return true;
    }
    void update(uint32_t t)
    {
        uint32_t dt = t - m_last_update_t;

        if (m_have_screen && dt >= UPDATE_PERIOD)
        {
            m_display.clearDisplay();
            m_display.setTextSize(1);              // Normal 1:1 pixel scale
            m_display.setTextColor(SSD1306_WHITE); // Draw white text
            m_display.setCursor(0, 0);             // Start at top-left corner
            m_display.cp437(true);                 // Use full 256 char 'Code Page 437' font
            for (uint8_t i = 0; (i < BUF_LEN && m_buf[i]); i++)
            {
                m_display.write(m_buf[i]);
            }
            m_display.display();
            m_last_update_t = t;

            if (m_serial)
            {
                m_serial->println(m_buf);
            }
        }
        else if (!m_have_screen && dt >= RECONNECT_PERIOD)
        {
            try_connect();
            m_last_update_t = t;
        }
    }

private:
    Adafruit_SSD1306 m_display;
    usb_serial_class *m_serial = nullptr;
    bool m_have_screen;
    uint32_t m_last_update_t;
    // Buffer in Acc xyz, Gyro xyz, temp order.
    float m_acc_gyro_temp[7];
    // Buffers needed to get data out of the IMU library.
    sensors_event_t m_accel;
    sensors_event_t m_gyro;
    sensors_event_t m_temp;

    char m_buf[BUF_LEN];
};
