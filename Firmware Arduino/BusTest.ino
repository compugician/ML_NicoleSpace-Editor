// Stress test for SoftwareWire library.
// Tested with an Arduino Uno connected to an Arduino Uno.
// This is the sketch for the Master Arduino using the software i2c.

// Use define to switch between the Arduino Wire and the SoftwareWire.
#define TEST_SOFTWAREWIRE


#ifdef TEST_SOFTWAREWIRE

#include "SoftwareWire.h"

// SoftwareWire constructor.
// Parameters:
//   (1) pin for the software sda
//   (2) pin for the software scl
//   (3) use internal pullup resistors. Default true. Set to false to disable them.
//   (4) allow the Slave to stretch the clock pulse. Default true. Set to false for faster code.

// This stress test uses A4 and A5, that makes it easy to switch between the (hardware) Wire
// and the (software) SoftwareWire libraries.
// myWire: sda = A4, scl = A5, turn on internal pullups, allow clock stretching by Slave
SoftwareWire myWire( A4, A5);

#else

// Make code work with normal Wire library.
#include <Arduino.h>
#include <Wire.h>
#define myWire Wire         // use the real Arduino Wire library instead of the SoftwareWire.

#endif


void setup()
{
  Serial.begin(9600);      // start serial port
  Serial.println(("\nMaster"));

  myWire.begin();          // join i2c bus as master
  myWire.setClock(5000);
}

void setPosition(uint8_t addr, uint8_t pos) {
  myWire.beginTransmission(addr);
  myWire.write( pos );
  if( myWire.endTransmission() != 0)
  {
    Serial.println("Error!");
  }
}

void loop()
{
  int a,b;
  while(Serial.available()) {
    a = Serial.parseInt();// read the incoming data as string
    b = Serial.parseInt();// read the incoming data as string
    Serial.print(a);
    Serial.print(":");
    Serial.println(b);
    if (a>=0 && a<256) {
      setPosition(a,b);
    }
  }
}
