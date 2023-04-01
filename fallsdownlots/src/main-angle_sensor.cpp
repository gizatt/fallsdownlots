#include <Arduino.h>
#include <SimpleFOC.h>

MagneticSensorI2C as5600 = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C as5600_2 = MagneticSensorI2C(AS5600_I2C);
// MagneticSensorAnalog as5600 = MagneticSensorAnalog(A1, 2, 3803);
int minraw = 10000;
int maxraw = 0;

// TwoWire Wire0(NRF_TWIM0, NRF_TWIS0, SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQn, 5, 6);
TwoWire Wire1(NRF_TWIM1, NRF_TWIS1, SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQn, 11, 12);

void setup()
{
    analogReadResolution(12);

    // monitoring port
    Serial.begin(115200);

    // init magnetic sensor hardware
    as5600.init(&Wire);
    as5600_2.init(&Wire1);

    Serial.println("AS5600 ready");
    _delay(1000);
}

void loop()
{
    // IMPORTANT - call as frequently as possible
    // update the sensor values
    as5600.update();
    as5600_2.update();
    /*
    if (as5600.raw_count > 0 && as5600.raw_count < 5000)
    {
        minraw = min(as5600.raw_count, minraw);
        maxraw = max(as5600.raw_count, maxraw);
    }
    */
    // display the angle and the angular velocity to the terminal
    Serial.print(as5600.getAngle());
    Serial.print(" ");
    Serial.println(as5600_2.getAngle());
}