#pragma once

#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

class IMU
{
public:
    const float UPDATE_PERIOD = 0.0001; // Seconds
    const float RECONNECT_PERIOD = 1.0; // Seconds
    const float COMPLEMENTARY_FILTER_ALPHA = 0.995;
    const float DANGLE_LOW_PASS_ALPHA = 0.5;

    IMU(usb_serial_class *serial = nullptr) : m_serial(serial), m_have_imu(false), m_last_update_t(micros()), m_angle(0.), m_dangle(0.0), m_avg_update_dt(0.)
    {
        try_connect();
    }

    bool connected()
    {
        return m_have_imu;
    }

    bool try_connect()
    {
        m_have_imu = m_mpu.begin(MPU6050_I2CADDR_DEFAULT, &Wire);
        if (!m_have_imu && m_serial)
        {
            m_serial->printf("Could not connect to IMU.");
        }
        else
        {
            Wire.setClock(400000);
            m_mpu.setAccelerometerRange(mpu6050_accel_range_t::MPU6050_RANGE_2_G);
            m_mpu.setGyroRange(mpu6050_gyro_range_t::MPU6050_RANGE_1000_DEG);
            m_dangle_calib = 0.;
            for (int i = 0; i < 25; i++)
            {
                // Quick and dirty gyro zero calib.
                m_mpu.getEvent(&m_accel, &m_gyro, &m_temp);
                m_dangle_calib += m_gyro.gyro.y;
                delay(10);
            }
            // m_dangle_calib /= 25.;
            m_dangle_calib = 0.0;
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
        if (t < m_last_update_t)
        {
            // Wraparound. TODO(gizatt) Is this right?
            dt_uint32 += UINT32_MAX;
        }
        float dt = ((float)dt_uint32) / 1E6; // Convert to seconds.

        if (m_have_imu && dt >= UPDATE_PERIOD)
        {
            m_mpu.getEvent(&m_accel, &m_gyro, &m_temp);
            m_acc_gyro_temp[0] = m_accel.acceleration.x;
            m_acc_gyro_temp[1] = m_accel.acceleration.y;
            m_acc_gyro_temp[2] = m_accel.acceleration.z;
            m_acc_gyro_temp[3] = m_gyro.gyro.x;
            m_acc_gyro_temp[4] = m_gyro.gyro.y;
            m_acc_gyro_temp[5] = m_gyro.gyro.z;
            m_acc_gyro_temp[6] = m_temp.temperature;

            // Update angle estimate.
            // +z axis is forward (up "out" of the IMU), -x axis is up,
            // rotation is around y axis. So at
            // --> 0 deg = z = 0, x = 9.8
            // --> +90 deg = z = 9.8, x = 0.
            float xz_norm = sqrt(acceleration()[0] * acceleration()[0] + acceleration()[2] * acceleration()[2]);
            float normalized_x = acceleration()[0] / xz_norm;
            float normalized_z = acceleration()[2] / xz_norm;
            float angle_from_imu = atan2(-normalized_z, -normalized_x);
            m_dangle = m_dangle * DANGLE_LOW_PASS_ALPHA + (rotational_velocity()[1] - m_dangle_calib) * (1. - DANGLE_LOW_PASS_ALPHA);
            float angle_from_odometry = m_angle + dt * m_dangle;
            m_angle = angle_from_odometry * COMPLEMENTARY_FILTER_ALPHA + angle_from_imu * (1. - COMPLEMENTARY_FILTER_ALPHA);
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
     * m/s/s
     */
    const float *acceleration()
    {
        return m_acc_gyro_temp + 0;
    }
    /*
     *   Rad / sec
     */
    const float *rotational_velocity()
    {
        return m_acc_gyro_temp + 3;
    }
    float temperature()
    {
        return m_acc_gyro_temp[6];
    }
    /*
     * Return the estimated angle (0 = upright) in radians.
     */
    float angle()
    {
        return m_angle;
    }

    float dangle()
    {
        return m_dangle;
    }

    float avg_update_dt()
    {
        return m_avg_update_dt;
    }

private:
    Adafruit_MPU6050 m_mpu;
    usb_serial_class *m_serial = nullptr;
    bool m_have_imu;
    uint32_t m_last_update_t;
    // Buffer in Acc xyz, Gyro xyz, temp order.
    float m_acc_gyro_temp[7];
    float m_angle;
    float m_dangle;
    float m_dangle_calib;
    float m_avg_update_dt;
    // Buffers needed to get data out of the IMU library.
    sensors_event_t m_accel;
    sensors_event_t m_gyro;
    sensors_event_t m_temp;
};
