#pragma once

#include <Arduino.h>

#include <I2Cdev.h>
#if (I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE) && !defined(PARTICLE)
#include "Wire.h"
#endif
#include <MPU6050_6Axis_MotionApps20.h>

volatile static bool m_mpu_interrupt; // indicates whether MPU interrupt pin has gone high
static void isr_dmp_data_ready()
{
    m_mpu_interrupt = true;
}

class IMU
{
public:
    const float UPDATE_PERIOD = 0.0001; // Seconds
    const float RECONNECT_PERIOD = 1.0; // Seconds
    const float COMPLEMENTARY_FILTER_ALPHA = 0.995;
    const float DANGLE_LOW_PASS_ALPHA = 0.5;
    const uint8_t INTERRUPT_PIN = 15;
    // Keyed by MPU6050_GYRO_FS_*
    const float GYRO_SCALING[4] = {
        1. / 131., 1 / 65.5, 1 / 32.8, 1 / 16.4};

    // This class maybe shouldn't exist, as it's inevitably a singleton?
    IMU(usb_serial_class *serial = nullptr) : m_mpu(0x68), m_serial(serial), m_have_imu(false), m_dmp_ready(false), m_packet_size(-1), m_last_update_t(micros()), m_avg_update_dt(0.)
    {
        Wire.begin();
        Wire.setClock(400000);
        try_connect();
    }

    bool connected()
    {
        return m_have_imu;
    }

    bool try_connect()
    {
        m_mpu.initialize();
        m_have_imu = m_mpu.testConnection();
        if (!m_have_imu && m_serial)
        {
            m_serial->printf("Could not connect to IMU.");
        }
        else
        {
            uint8_t dev_status = m_mpu.dmpInitialize();
            /*
            // These are copied from library example. What are mine?
            m_mpu.setXGyroOffset(220);
            m_mpu.setYGyroOffset(76);
            m_mpu.setZGyroOffset(-85);
            m_mpu.setZAccelOffset(1788);
            */

            if (dev_status != 0)
            {
                m_have_imu = false;
                m_serial->printf("DMP initialization failed with code %d.\n", dev_status);
                return false;
            }
            // m_mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
            // m_mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
            m_mpu.setDLPFMode(MPU6050_DLPF_BW_98); // gyro lpf in approximate bandwidth hz (gyro samples).
            m_mpu.setRate(4);                      // Update every low-passed gyro sample.
            m_mpu.setDMPEnabled(true);

            // Attach data-ready interrupt.
            attachInterrupt(INTERRUPT_PIN, isr_dmp_data_ready, RISING);
            m_mpu_interrupt = false;
            m_mpu.getIntStatus();

            // All done, get ready to roll.
            m_packet_size = m_mpu.dmpGetFIFOPacketSize();
            m_dmp_ready = true;
        }
        return m_have_imu;
    }

    void update(uint32_t t)
    {
        // uint32_t dt = t - m_last_update_t;
        //  Handle our own timekeeping. TODO: Update whole system to float seconds since
        //  startup using micros() or something...
        t = micros();
        uint32_t dt_uint32 = t - m_last_update_t;
        float dt = ((float)dt_uint32) / 1E6; // Convert to seconds.

        if (m_dmp_ready && m_mpu_interrupt && dt >= UPDATE_PERIOD)
        {
            // Data is available to read.
            m_mpu_interrupt = false;
            uint8_t mpu_int_status = m_mpu.getIntStatus();

            // See how much data is ready.
            uint16_t fifo_count = m_mpu.getFIFOCount();
            if ((mpu_int_status & 0x10) || fifo_count == 1024)
            {
                // reset so we can continue cleanly
                m_mpu.resetFIFO();
                m_serial->printf("IMU FIFO overflow!");
                fifo_count = 0;
            }
            else if ((mpu_int_status & 0x02) > 0)
            {
                while (fifo_count > 0)
                {
                    // wait for correct available data length, should be a VERY short wait
                    while (fifo_count < m_packet_size)
                    {
                        fifo_count = m_mpu.getFIFOCount();
                    }
                    m_mpu.getFIFOBytes(m_fifo_buffer, m_packet_size);
                    fifo_count -= m_packet_size;

                    // Parse out data.
                    m_mpu.dmpGetQuaternion(&m_q, m_fifo_buffer);
                    m_mpu.dmpGetGravity(&m_gravity, &m_q);
                    m_mpu.dmpGetGyro(m_gyro, m_fifo_buffer);
                }
            }
            m_avg_update_dt = m_avg_update_dt * 0.99 + dt * 0.01;
            m_last_update_t = t;
        }
        else if (!m_have_imu && dt >= RECONNECT_PERIOD)
        {
            try_connect();
            m_last_update_t = t;
        }
    }

    /*
     * Return the estimated angle (0 = upright) in radians.
     */
    float angle()
    {
        // Calc from gravity
        float xz_norm = sqrt(m_gravity.x * m_gravity.x + m_gravity.z * m_gravity.z);
        float normalized_x = m_gravity.x / xz_norm;
        float normalized_z = m_gravity.z / xz_norm;
        return atan2(-normalized_z, -normalized_x);
    }

    float dangle()
    {
        uint8_t range_ind = m_mpu.getFullScaleGyroRange();
        if (range_ind > 4)
        {
            m_serial->printf("Got invalid range ind from m_mpu.");
            return 0.;
        }
        float scale = GYRO_SCALING[range_ind];
        // Remap from full int16_t range to whatever the selected range is.
        // return scale * PI / 180. * (float)m_gyro[1];
        return scale * PI / 180. * (float)m_mpu.getRotationY();
    }

    float avg_update_dt()
    {
        return m_avg_update_dt;
    }

private:
    MPU6050 m_mpu;
    usb_serial_class *m_serial = nullptr;
    bool m_have_imu;

    // DMP details.
    bool m_dmp_ready;
    uint16_t m_packet_size;

    uint32_t m_last_update_t;

    // Buffers needed to get data.
    int16_t m_gyro[3];
    VectorFloat m_gravity;
    float m_avg_update_dt;
    Quaternion m_q;
    // FIFO storage buffer.
    uint8_t m_fifo_buffer[64];
};
