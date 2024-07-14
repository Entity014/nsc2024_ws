#include <Arduino.h>
#include "AS5600.h"

AS5600 as5600; //  use default Wire

void setup()
{
  Serial.begin(115200);
  Serial.println(__FILE__);
  Serial.print("AS5600_LIB_VERSION: ");
  Serial.println(AS5600_LIB_VERSION);

  Wire.begin();
  Serial.println(as5600.getAddress());

  //  as5600.setAddress(0x40);  //  AS5600L only

  int b = as5600.isConnected();
  Serial.print("Connect: ");
  Serial.println(b);

  delay(1000);
}

void loop()
{
  Serial.println(as5600.rawAngle() * AS5600_RAW_TO_DEGREES);

  // delay(1000);
}