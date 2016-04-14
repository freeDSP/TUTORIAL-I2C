/*
   volume_control.ino

    Created on: 15.01.2016
        Author: Fabian Geißler (fabian.geissler[at]tu-dresden.de)

    This file is part of the FreeDSP I2C communication guide.
    It will work without modification on an Arduino Nano which
    is stacked onto the FreeDSP Classic. For any other configuration
    you may need to modify this sketch.

    This sketch has the following functions:
     1) write e2prom.hex file from arduino flash to eeprom memory chip
     2) reset the ADAU1701
     3) Listen on the serialport and change the DSP parameters on-the-fly



    FreeDSP I2C communication guide - PART 1

    In part 1 we have a simple schematic, which routes the input
    through a double volume control to the output. The volume slider
    value will be adjusted to match a parameter recieved via the
    serial port. A 100 on the serial port represents a volume level
    of 1.0 or 0dB, so the signal is not affected. A value of 0 will 
	mute the output.

    NOTE: The volume will be adjusted linear, not in decibels.

*/

// Include libraries for I2C and EEPROM
#include "i2clib/EEPROM24.h"
#include "i2clib/I2CMaster.h"
#include "i2clib/SoftI2C_mod.h"
#include "i2clib/ADAU1701.h"

// Defines for DSP communication.
#define DSP_SDA       9
#define DSP_SCL       10
#define DSP_WP        11
#define DSP_RES       12
#define DSP_ADDRESS   0

#define VOLUME0_ADDR  0x0000
#define VOLUME1_ADDR  0x0001

// Defines for serial communication.
#define COM_BAUD    9600
#define COM_BUFSIZE 25

// Library class instantiation.
SoftI2C   mI2C      (DSP_SDA, DSP_SCL);       // Init software I2C library.
EEPROM24  mEeprom   (mI2C, EEPROM_24LC256);   // Init EEPROM controller library.
ADAU1701  mDSP      (mI2C, DSP_ADDRESS);      // Init ADAU1701 library.

// Here follows the content of the E2PROM.hex file (collapse the code block for better readability).
#define E2PROM_HEX_SIZE 544
const uint8_t e2prom_hex[E2PROM_HEX_SIZE] = { 0x01 , 0x00 , 0x05 , 0x00 , 0x08 , 0x1C , 0x00 , 0x58 ,
                                              0x03 , 0x03 , 0x03 , 0x03 , 0x03 , 0x03 , 0x03 , 0x03 ,
                                              0x03 , 0x03 , 0x03 , 0x03 , 0x03 , 0x03 , 0x03 , 0x03 ,
                                              0x03 , 0x03 , 0x01 , 0x00 , 0x23 , 0x00 , 0x08 , 0x00 ,
                                              0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 ,
                                              0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 ,
                                              0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 ,
                                              0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 ,
                                              0x01 , 0x00 , 0x0B , 0x00 , 0x00 , 0x00 , 0x00 , 0x80 ,
                                              0x00 , 0x00 , 0x00 , 0x80 , 0x00 , 0x00 , 0x01 , 0x01 ,
                                              0x43 , 0x00 , 0x04 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 ,
                                              0x01 , 0x00 , 0x00 , 0x00 , 0xE8 , 0x01 , 0x00 , 0x00 ,
                                              0x00 , 0x00 , 0x01 , 0x00 , 0x08 , 0x00 , 0xE8 , 0x01 ,
                                              0x00 , 0x02 , 0x00 , 0x20 , 0x01 , 0x00 , 0x10 , 0x00 ,
                                              0xE2 , 0x01 , 0x00 , 0x00 , 0x00 , 0x00 , 0x01 , 0x00 ,
                                              0x0A , 0x01 , 0x20 , 0x01 , 0x00 , 0x18 , 0x00 , 0xE2 ,
                                              0x01 , 0x00 , 0x00 , 0x00 , 0x00 , 0x01 , 0x00 , 0x11 ,
                                              0x08 , 0x20 , 0x01 , 0xFF , 0x68 , 0x00 , 0x02 , 0x01 ,
                                              0x00 , 0x19 , 0x08 , 0x20 , 0x01 , 0xFF , 0x70 , 0x00 ,
                                              0x02 , 0x01 , 0x00 , 0x00 , 0x00 , 0x00 , 0x01 , 0x00 ,
                                              0x00 , 0x00 , 0x00 , 0x01 , 0x00 , 0x00 , 0x00 , 0x00 ,
                                              0x01 , 0x00 , 0x00 , 0x00 , 0x00 , 0x01 , 0x00 , 0x00 ,
                                              0x00 , 0x00 , 0x01 , 0x00 , 0x00 , 0x00 , 0x00 , 0x01 ,
                                              0x00 , 0x00 , 0x00 , 0x00 , 0x01 , 0x00 , 0x00 , 0x00 ,
                                              0x00 , 0x01 , 0x00 , 0x00 , 0x00 , 0x00 , 0x01 , 0x00 ,
                                              0x00 , 0x00 , 0x00 , 0x01 , 0x00 , 0x00 , 0x00 , 0x00 ,
                                              0x01 , 0x00 , 0x00 , 0x00 , 0x00 , 0x01 , 0x00 , 0x00 ,
                                              0x00 , 0x00 , 0x01 , 0x00 , 0x00 , 0x00 , 0x00 , 0x01 ,
                                              0x00 , 0x00 , 0x00 , 0x00 , 0x01 , 0x00 , 0x00 , 0x00 ,
                                              0x00 , 0x01 , 0x00 , 0x00 , 0x00 , 0x00 , 0x01 , 0x00 ,
                                              0x00 , 0x00 , 0x00 , 0x01 , 0x00 , 0x00 , 0x00 , 0x00 ,
                                              0x01 , 0x00 , 0x00 , 0x00 , 0x00 , 0x01 , 0x00 , 0x00 ,
                                              0x00 , 0x00 , 0x01 , 0x00 , 0x00 , 0x00 , 0x00 , 0x01 ,
                                              0x00 , 0x00 , 0x00 , 0x00 , 0x01 , 0x00 , 0x00 , 0x00 ,
                                              0x00 , 0x01 , 0x00 , 0x00 , 0x00 , 0x00 , 0x01 , 0x00 ,
                                              0x00 , 0x00 , 0x00 , 0x01 , 0x00 , 0x00 , 0x00 , 0x00 ,
                                              0x01 , 0x00 , 0x00 , 0x00 , 0x00 , 0x01 , 0x00 , 0x00 ,
                                              0x00 , 0x00 , 0x01 , 0x00 , 0x00 , 0x00 , 0x00 , 0x01 ,
                                              0x00 , 0x00 , 0x00 , 0x00 , 0x01 , 0x00 , 0x00 , 0x00 ,
                                              0x00 , 0x01 , 0x00 , 0x00 , 0x00 , 0x00 , 0x01 , 0x00 ,
                                              0x00 , 0x00 , 0x00 , 0x01 , 0x00 , 0x00 , 0x00 , 0x00 ,
                                              0x01 , 0x00 , 0x00 , 0x00 , 0x00 , 0x01 , 0x00 , 0x00 ,
                                              0x00 , 0x00 , 0x01 , 0x00 , 0x00 , 0x00 , 0x00 , 0x01 ,
                                              0x00 , 0x00 , 0x00 , 0x00 , 0x01 , 0x00 , 0x00 , 0x00 ,
                                              0x00 , 0x01 , 0x00 , 0x00 , 0x00 , 0x00 , 0x01 , 0x00 ,
                                              0x00 , 0x00 , 0x00 , 0x01 , 0x00 , 0x00 , 0x00 , 0x00 ,
                                              0x01 , 0x00 , 0x00 , 0x00 , 0x00 , 0x01 , 0x00 , 0x00 ,
                                              0x00 , 0x00 , 0x01 , 0x00 , 0x00 , 0x00 , 0x00 , 0x01 ,
                                              0x00 , 0x00 , 0x00 , 0x00 , 0x01 , 0x00 , 0x00 , 0x00 ,
                                              0x00 , 0x01 , 0x00 , 0x00 , 0x00 , 0x00 , 0x01 , 0x00 ,
                                              0x00 , 0x00 , 0x00 , 0x01 , 0x01 , 0x00 , 0x05 , 0x00 ,
                                              0x08 , 0x1C , 0x00 , 0x1C , 0x01 , 0x00 , 0x04 , 0x00 ,
                                              0x08 , 0x1D , 0x08 , 0x01 , 0x00 , 0x05 , 0x00 , 0x08 ,
                                              0x1E , 0x00 , 0x00 , 0x01 , 0x00 , 0x04 , 0x00 , 0x08 ,
                                              0x1F , 0x00 , 0x01 , 0x00 , 0x06 , 0x00 , 0x08 , 0x20 ,
                                              0x00 , 0x00 , 0x00 , 0x01 , 0x00 , 0x06 , 0x00 , 0x08 ,
                                              0x21 , 0x00 , 0x00 , 0x00 , 0x01 , 0x00 , 0x05 , 0x00 ,
                                              0x08 , 0x22 , 0x00 , 0x00 , 0x01 , 0x00 , 0x05 , 0x00 ,
                                              0x08 , 0x23 , 0x00 , 0x00 , 0x01 , 0x00 , 0x05 , 0x00 ,
                                              0x08 , 0x24 , 0x80 , 0x00 , 0x01 , 0x00 , 0x05 , 0x00 ,
                                              0x08 , 0x25 , 0x00 , 0x00 , 0x01 , 0x00 , 0x05 , 0x00 ,
                                              0x08 , 0x26 , 0x00 , 0x00 , 0x01 , 0x00 , 0x05 , 0x00 ,
                                              0x08 , 0x27 , 0x00 , 0x01 , 0x01 , 0x00 , 0x05 , 0x00 ,
                                              0x08 , 0x1C , 0x00 , 0x1C , 0x06 , 0x00 , 0x00 , 0x00 ,
                                              0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 ,
                                              0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 ,
                                              0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 ,
                                              0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00
                                            };

void setup()
{
  // Set all pins to input and deactivate internal pullup
  // as they are all open drain and not used yet.
  pinMode(DSP_SDA, INPUT);
  digitalWrite(DSP_SDA, LOW);
  pinMode(DSP_SCL, INPUT);
  digitalWrite(DSP_SCL, LOW);
  pinMode(DSP_WP, INPUT);         // For open drain output switch between input and output
  digitalWrite(DSP_WP, LOW);      // while "output" value is set to LOW all the time.
  pinMode(DSP_RES, INPUT);
  digitalWrite(DSP_RES, LOW);

  // Wait for DSP to startup and load data from eeprom.
  delay(22);
  while (digitalRead(DSP_WP) == LOW);

  // Init serial port.
  Serial.begin(COM_BAUD);

  // Wait for serial port to connect.
  while (!Serial);

  Serial.println("FreeDSP I2C Guide - PART 1");

  // Check bus for EEPROM.
  bool eeprom_found = mEeprom.available();

  // Some delay between addressing different devices.
  delay(10);

  // Check bus for DSP.
  bool dsp_found = mDSP.available();

  // Tell the user.
  Serial.print("eeprom ");
  if (eeprom_found == false) Serial.print("not ");
  Serial.println("found");

  Serial.print("adau1701 ");
  if (dsp_found == false) Serial.print("not ");
  Serial.println("found");

  // Both devices must be connected to the bus for this sketch to work.
  if ((!dsp_found) || (!eeprom_found))
  {
    Serial.println("Please make sure the DSP and the EEPROM are available on the I2C line and reststart!");
    while (1);
  }

  mEeprom.write(0, e2prom_hex, E2PROM_HEX_SIZE);

  // Reset the DSP to reload the EEPROM content
  pinMode(DSP_RES, OUTPUT);
  delay(1000);
  pinMode(DSP_RES, INPUT);

  delay(22); // wait for DSP to startup
  while (digitalRead(DSP_WP) == LOW); // Wait for DSP to startup and load data from eeprom.

  Serial.println("EEPROM written and DSP restarted!");
  Serial.println("Feel free to write values now.");
  Serial.println("Values range from -1500 to +1500.");
  Serial.println("A value of 100 represents a volume level of 1.0 or 0dB.");
}

void waitForLine(char* line, uint8_t len)
{
  // Wait until new data has been sent.
  while (Serial.available() == 0);
  uint8_t idx = Serial.readBytesUntil('\n', line, len - 1);
  line[idx] = 0;
}

void loop() {
  char line[COM_BUFSIZE];

  waitForLine(line, COM_BUFSIZE);

  // Read level from line.
  int32_t level = atoi(line);

  // Now convert our percent value to a 5.23 fixpoint as the DSP uses these.
  int32_t val32 = ((1UL << 23) * level) / 100;

  // Highest byte must be sent first, so reverse byte order.
  uint8_t data[4] = {((uint8_t*)&val32)[3],
                     ((uint8_t*)&val32)[2],
                     ((uint8_t*)&val32)[1],
                     ((uint8_t*)&val32)[0]
                    };

  // Write the value to the DSP param memory.
  mDSP.write(VOLUME0_ADDR, 4, data);
  mDSP.write(VOLUME1_ADDR, 4, data);

  // Do some user interaction.
  Serial.print("Data recieved: ");
  Serial.println(level, DEC);

  Serial.print("Data written: ");

  for (int i = 0; i < 4; ++i)
  {
    if (data[i] < 16)
      Serial.print("0");

    Serial.print(data[i], HEX);
    Serial.print(" ");
  }

  Serial.print("\n");
}
