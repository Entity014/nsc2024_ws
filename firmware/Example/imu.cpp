#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);
imu::Vector<3> euler_temp, euler;

bool isStartX = false;
bool isStartY = false;
bool isStartZ = false;

void setup(void)
{
  Serial.begin(115200);
  Serial.println("Orientation Sensor Test");
  Serial.println("");
  //  Wire.begin(22,27); // SDA pin 21, SCL pin 22
  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }

  delay(1000);

  bno.setExtCrystalUse(true);
}

void loop(void)
{
  imu::Vector<3> e = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  if ((euler_temp.x() != e.x()) && (!isStartX))
  {
    euler_temp.x() = e.x();
    isStartX = true;
  }
  if ((euler_temp.y() != e.y()) && (!isStartY))
  {
    euler_temp.y() = e.y();
    isStartY = true;
  }
  if ((euler_temp.z() != e.z()) && (!isStartZ))
  {
    euler_temp.z() = e.z();
    isStartZ = true;
  }
  euler.x() = euler_temp.x() - e.x();
  euler.y() = euler_temp.y() - e.y();
  euler.z() = euler_temp.z() - e.z();
  /* Display the floating point data */
  Serial.print("X: ");
  Serial.print(e.x());
  Serial.print(" Y: ");
  Serial.print(e.y());
  Serial.print(" Z: ");
  Serial.print(e.z());
  Serial.println("");

  delay(100);
}