#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "AS5600.h"

#define TCAADDR 0x70

Adafruit_BNO055 bno055_1 = Adafruit_BNO055(55, 0x29);
Adafruit_BNO055 bno055_2 = Adafruit_BNO055(55, 0x29);

AS5600 as5600;

void tcaSelect(uint8_t i)
{
  if (i > 7)
    return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

// standard Arduino setup()
void setup()
{
  Wire.begin();

  Serial.begin(115200);
  Serial.println("\nTCAScanner ready!");

  tcaSelect(0);
  if (!bno055_1.begin())
  {
    Serial.print("No BNO055 detected on TCA9548A channel 0");
    while (1)
      ;
  }

  tcaSelect(1); // Select channel 1
  if (!bno055_2.begin())
  {
    Serial.print("No BNO055 detected on TCA9548A channel 1");
    while (1)
      ;
  }
  tcaSelect(2);
  as5600.isConnected();

  delay(1000);
  tcaSelect(0);
  bno055_1.setExtCrystalUse(true);
  tcaSelect(1);
  bno055_2.setExtCrystalUse(true);

  Serial.println("BNO055 sensors initialized.");
}

void loop()
{
  tcaSelect(0);
  imu::Quaternion quat_1 = bno055_1.getQuat();
  Serial.print("Sensor 1 - Qx: ");
  Serial.print(quat_1.x(), 4);
  Serial.print(" Qy: ");
  Serial.print(quat_1.y(), 4);
  Serial.print(" Qz: ");
  Serial.print(quat_1.z(), 4);
  Serial.print(" Qw: ");
  Serial.print(quat_1.w(), 4);
  Serial.println("");

  tcaSelect(1);
  imu::Quaternion quat_2 = bno055_2.getQuat();
  Serial.print("Sensor 2 - Qx: ");
  Serial.print(quat_2.x(), 4);
  Serial.print(" Qy: ");
  Serial.print(quat_2.y(), 4);
  Serial.print(" Qz: ");
  Serial.print(quat_2.z(), 4);
  Serial.print(" Qw: ");
  Serial.print(quat_2.w(), 4);
  Serial.println("");

  tcaSelect(2);
  Serial.println(as5600.rawAngle() * AS5600_RAW_TO_DEGREES);
  delay(100);
}
