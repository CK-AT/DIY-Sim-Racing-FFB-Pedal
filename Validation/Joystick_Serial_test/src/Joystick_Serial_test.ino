// Simple example application that shows how to read four Arduino
// digital pins and map them to the USB Joystick library.
//
// Ground digital pins 9, 10, 11, and 12 to press the joystick 
// buttons 0, 1, 2, and 3.
//
// NOTE: This sketch file is for use with Arduino Leonardo and
//       Arduino Micro only.
//
// by Matthew Heironimus
// 2015-11-20
//--------------------------------------------------------------------

#include <Joystick_ESP32S2.h>
//USBCDC USBSerial;

Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_GAMEPAD,
                   0, 0,                 // Button Count, Hat Switch Count
                   false, false, false,  // X and Y, but no Z Axis
                   false, false, false,  // No Rx, Ry, or Rz
                   false, false,         // No rudder or throttle
                   false, true, false);  // No accelerator, brake, or steering

void setup() {

  Serial.setTxTimeoutMs(500);
  
  
  Joystick.setBrakeRange(0, 1000);

  // Initialize Joystick Library
  Joystick.begin();
}


uint64_t counter = 0;
void loop() {

  delay(5);

  // increment joystick output
  counter+=100;
  if (counter > 1000)
  {
    counter = 0;
  }

  Serial.println(counter);

  // write large packet
  uint8_t tmp[100];
  Serial.write((char*)&tmp[0], sizeof(uint8_t) * 100 );

  delay(5); // <-- This helped a lot!

  // send USB HID output
  Joystick.setBrake(counter);
}

