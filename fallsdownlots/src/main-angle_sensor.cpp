#include <SimpleFOC.h>

MagneticSensorI2C as5600 = MagneticSensorI2C(AS5600_I2C);

void setup()
{
    // monitoring port
    Serial.begin(115200);

    // init magnetic sensor hardware
    as5600.init();

    Serial.println("AS5600 ready");
    _delay(1000);
}

void loop()
{
    // IMPORTANT - call as frequently as possible
    // update the sensor values
    as5600.update();
    // display the angle and the angular velocity to the terminal
    Serial.print(as5600.getAngle());
    Serial.print("\t");
    Serial.println(as5600.getVelocity());
}