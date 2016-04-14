/*
   safeload.ino

    Created on: 16.01.2016
        Author: Fabian Geißler (fabian.geissler[at]tu-dresden.de)

    This file is part of the FreeDSP I2C communication guide.
    It will work without modification on an Arduino Nano which
    is stacked onto the FreeDSP Classic. For any other configuration
    you may need to modify this sketch.

    This sketch has the following functions:
     1) write e2prom.hex file from arduino flash to eeprom memory chip
     2) reset the ADAU1701
     3) Listen on the serialport and change the DSP parameters on-the-fly



    FreeDSP I2C communication guide - PART 4

    In the schematic for this part te inputs are wired to a RMS level envelope,
    which is wired to a readback cell. Another readback cell is wired directly 
    to a DC source which generates a fixed reference value. Although one could 
    possibly read from any point in the program flow, the readback cell forces 
    the compiler to output the needed program counter value, at which we can 
    read the values we want, i.e. which are connected to the readback cell.

    The principle of the readback is somehow easy to do but hard to understand.
    First we need to write the mentioned program counter address and two register
    select bits to the capture register. The register select bits specify at
    which stage of the calculation (which ALU register) should be read. After
    this write we have to read three bytes from the exact same register. Those
    bytes are the data of the ALU register.

    IMPORTANT: See ADAU_REGISTERS.h for register defines and the datasheet
               for detailed description.

*/

#include <math.h>

// Include libraries for I2C and EEPROM
#include "i2clib/EEPROM24.h"
#include "i2clib/I2CMaster.h"
#include "i2clib/SoftI2C_mod.h"
#include "i2clib/ADAU1701.h"
#include "i2clib/ADAU1701_REGISTERS.h"

// Defines for DSP communication.
#define DSP_SDA       9
#define DSP_SCL       10
#define DSP_WP        11
#define DSP_RES       12
#define DSP_ADDRESS   0
#define DSP_FIX_ADDR  0

// Serial communication defines
#define COM_BAUD  9600

// Readback addresses.
// These values are taken from the readback.xml file in Sysfiles directory.
// They are a parameter of the readback cells marked as HexArray-type.
// The values already include the register select bits.
#define RB_LEFT 0x0116
#define RB_RIGHT 0x0122
#define RB_FIX 0x010A

// Library class initialisation.
SoftI2C   mI2C      (DSP_SDA, DSP_SCL);       // Init software I2C library.
EEPROM24  mEeprom   (mI2C, EEPROM_24LC256);   // Init EEPROM controller library.
ADAU1701  mDSP      (mI2C, DSP_ADDRESS);      // Init ADAU1701 library.

// Here follows the content of the E2PROM.hex file (collapse the code block for better readability).
#define E2PROM_HEX_SIZE 72*8
const uint8_t e2prom_hex[E2PROM_HEX_SIZE] = { 0x01 , 0x00 , 0x05 , 0x00 , 0x08 , 0x1C , 0x00 , 0x58 ,
                                              0x03 , 0x03 , 0x03 , 0x03 , 0x03 , 0x03 , 0x03 , 0x03 ,
                                              0x03 , 0x03 , 0x03 , 0x03 , 0x03 , 0x03 , 0x03 , 0x03 ,
                                              0x03 , 0x03 , 0x01 , 0x00 , 0x23 , 0x00 , 0x08 , 0x00 ,
                                              0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 ,
                                              0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 ,
                                              0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 ,
                                              0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 ,
                                              0x01 , 0x00 , 0x1F , 0x00 , 0x00 , 0x00 , 0x00 , 0x0C ,
                                              0xCC , 0xCD , 0x00 , 0x00 , 0x12 , 0xDE , 0x00 , 0x00 ,
                                              0x01 , 0xE0 , 0x00 , 0x00 , 0x00 , 0x12 , 0x00 , 0x00 ,
                                              0x12 , 0xDE , 0x00 , 0x00 , 0x01 , 0xE0 , 0x00 , 0x00 ,
                                              0x00 , 0x12 , 0x01 , 0x01 , 0x5C , 0x00 , 0x04 , 0x00 ,
                                              0x00 , 0x00 , 0x00 , 0x00 , 0x01 , 0x00 , 0x00 , 0x00 ,
                                              0xE8 , 0x01 , 0x00 , 0x00 , 0x00 , 0x00 , 0x01 , 0x00 ,
                                              0x08 , 0x00 , 0xE8 , 0x01 , 0xFF , 0xF2 , 0x00 , 0x20 ,
                                              0x01 , 0x00 , 0x10 , 0x00 , 0xE2 , 0x01 , 0x00 , 0x01 ,
                                              0x08 , 0x20 , 0x01 , 0x00 , 0x00 , 0x00 , 0x00 , 0x01 ,
                                              0xFF , 0xE9 , 0x08 , 0x30 , 0x01 , 0x00 , 0x68 , 0x00 ,
                                              0xE2 , 0x01 , 0x00 , 0x21 , 0x08 , 0x20 , 0x01 , 0x00 ,
                                              0x22 , 0x01 , 0x22 , 0x41 , 0x00 , 0x6A , 0x01 , 0x22 ,
                                              0x01 , 0x00 , 0x60 , 0x00 , 0xE2 , 0x01 , 0x00 , 0x00 ,
                                              0x00 , 0x00 , 0x01 , 0x00 , 0x28 , 0x00 , 0xE2 , 0x01 ,
                                              0x00 , 0x38 , 0x00 , 0xF2 , 0x01 , 0x00 , 0x51 , 0x08 ,
                                              0x20 , 0x09 , 0x00 , 0x58 , 0x00 , 0xE2 , 0x01 , 0x00 ,
                                              0x41 , 0x08 , 0x20 , 0x01 , 0xFF , 0xF2 , 0x03 , 0x22 ,
                                              0x67 , 0x00 , 0x48 , 0x00 , 0xE2 , 0x01 , 0x00 , 0x61 ,
                                              0x08 , 0x22 , 0x49 , 0x00 , 0x61 , 0x08 , 0x20 , 0x01 ,
                                              0x00 , 0x48 , 0x00 , 0xE2 , 0x27 , 0xFF , 0xF2 , 0x02 ,
                                              0x20 , 0x01 , 0x00 , 0x58 , 0x00 , 0xE2 , 0x27 , 0x00 ,
                                              0x00 , 0x00 , 0x00 , 0x01 , 0x00 , 0x59 , 0x08 , 0x20 ,
                                              0x09 , 0xFF , 0xF9 , 0x08 , 0x22 , 0x41 , 0x00 , 0x58 ,
                                              0x00 , 0xE2 , 0x26 , 0x00 , 0x49 , 0x08 , 0x20 , 0x01 ,
                                              0x00 , 0x18 , 0x00 , 0xE2 , 0x01 , 0x00 , 0x09 , 0x08 ,
                                              0x20 , 0x01 , 0x00 , 0x00 , 0x00 , 0x00 , 0x01 , 0xFF ,
                                              0xE9 , 0x08 , 0x30 , 0x01 , 0x00 , 0x68 , 0x00 , 0xE2 ,
                                              0x01 , 0x00 , 0x79 , 0x08 , 0x20 , 0x01 , 0x00 , 0x7A ,
                                              0x04 , 0x22 , 0x41 , 0x00 , 0x6A , 0x04 , 0x22 , 0x01 ,
                                              0x00 , 0xB8 , 0x00 , 0xE2 , 0x01 , 0x00 , 0x00 , 0x00 ,
                                              0x00 , 0x01 , 0x00 , 0x80 , 0x00 , 0xE2 , 0x01 , 0x00 ,
                                              0x90 , 0x00 , 0xF2 , 0x01 , 0x00 , 0xA9 , 0x08 , 0x20 ,
                                              0x09 , 0x00 , 0xB0 , 0x00 , 0xE2 , 0x01 , 0x00 , 0x99 ,
                                              0x08 , 0x20 , 0x01 , 0xFF , 0xF2 , 0x06 , 0x22 , 0x67 ,
                                              0x00 , 0xA0 , 0x00 , 0xE2 , 0x01 , 0x00 , 0xB9 , 0x08 ,
                                              0x22 , 0x49 , 0x00 , 0xB9 , 0x08 , 0x20 , 0x01 , 0x00 ,
                                              0xA0 , 0x00 , 0xE2 , 0x27 , 0xFF , 0xF2 , 0x05 , 0x20 ,
                                              0x01 , 0x00 , 0xB0 , 0x00 , 0xE2 , 0x27 , 0x00 , 0x00 ,
                                              0x00 , 0x00 , 0x01 , 0x00 , 0xB1 , 0x08 , 0x20 , 0x09 ,
                                              0xFF , 0xF9 , 0x08 , 0x22 , 0x41 , 0x00 , 0xB0 , 0x00 ,
                                              0xE2 , 0x26 , 0x00 , 0xA1 , 0x08 , 0x20 , 0x01 , 0x00 ,
                                              0x70 , 0x00 , 0xE2 , 0x01 , 0x00 , 0x00 , 0x00 , 0x00 ,
                                              0x01 , 0x00 , 0x11 , 0x08 , 0x20 , 0x01 , 0x00 , 0xC0 ,
                                              0x00 , 0xE2 , 0x01 , 0x00 , 0x00 , 0x00 , 0x00 , 0x01 ,
                                              0x00 , 0x19 , 0x08 , 0x20 , 0x01 , 0x00 , 0xC8 , 0x00 ,
                                              0xE2 , 0x01 , 0x00 , 0x00 , 0x00 , 0x00 , 0x01 , 0x00 ,
                                              0x71 , 0x08 , 0x20 , 0x01 , 0x00 , 0xD0 , 0x00 , 0xE2 ,
                                              0x01 , 0x01 , 0x00 , 0x05 , 0x00 , 0x08 , 0x1C , 0x00 ,
                                              0x1C , 0x01 , 0x00 , 0x04 , 0x00 , 0x08 , 0x1D , 0x08 ,
                                              0x01 , 0x00 , 0x05 , 0x00 , 0x08 , 0x1E , 0x00 , 0x00 ,
                                              0x01 , 0x00 , 0x04 , 0x00 , 0x08 , 0x1F , 0x00 , 0x01 ,
                                              0x00 , 0x06 , 0x00 , 0x08 , 0x20 , 0x00 , 0x00 , 0x00 ,
                                              0x01 , 0x00 , 0x06 , 0x00 , 0x08 , 0x21 , 0x00 , 0x00 ,
                                              0x00 , 0x01 , 0x00 , 0x05 , 0x00 , 0x08 , 0x22 , 0x00 ,
                                              0x00 , 0x01 , 0x00 , 0x05 , 0x00 , 0x08 , 0x23 , 0x00 ,
                                              0x00 , 0x01 , 0x00 , 0x05 , 0x00 , 0x08 , 0x24 , 0x80 ,
                                              0x00 , 0x01 , 0x00 , 0x05 , 0x00 , 0x08 , 0x25 , 0x00 ,
                                              0x00 , 0x01 , 0x00 , 0x05 , 0x00 , 0x08 , 0x26 , 0x00 ,
                                              0x00 , 0x01 , 0x00 , 0x05 , 0x00 , 0x08 , 0x27 , 0x00 ,
                                              0x01 , 0x01 , 0x00 , 0x05 , 0x00 , 0x08 , 0x1C , 0x00 ,
                                              0x1C , 0x06 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 ,
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

  // Print the input options
  Serial.println("EEPROM written and DSP restarted!");
  Serial.println("Level readbacks will show up here:");

  writeFixParameter(-0.5);
}

float readDSPValue(uint16_t pc)
{
  uint8_t dcr[ADAU_DC_WIDTH];

  // Set the program counter and the register select bits for the data capture register.
  dcr[0] = (pc >> 8) & 0xFF;
  dcr[1] = pc & 0xFF;

  // Write the data.
  mDSP.write(ADAU_DC, ADAU_DC_WIDTH, dcr);

  uint8_t data[3];

  // Read three bytes back from the exact same register.
  mDSP.read(ADAU_DC, 3, data);

  // 24bit 5.19 fixpoint value. The overall shift is needed for negative values.
  int32_t value = (((int32_t)data[2] << 8) | ((int32_t)data[1] << 16) | ((int32_t)data[0] << 24)) >> 8;

  // 5.19 fixpoint to float.
  float realval = (float)value / 524288.0f; // x / (1 << 19)

  return realval;
}

void writeFixParameter(float value)
{
  // Convert value to 5.23 fixpoint format.
  int32_t fixpoint = (int32_t)(8388608.0f * value); // (1 << 23) * x

// Reverse byte order and write to array.
  uint8_t bytes[4];
  bytes[0] = ((uint8_t*)(&fixpoint))[3];
  bytes[1] = ((uint8_t*)(&fixpoint))[2];
  bytes[2] = ((uint8_t*)(&fixpoint))[1];
  bytes[3] = ((uint8_t*)(&fixpoint))[0];

// Write directly to parameter memory.
// No audio stram is touched, so safeload is not necessary.
  mDSP.write(DSP_FIX_ADDR, 4, bytes);
}

void loop()
{
  // Write some value for the fixed param
  writeFixParameter(-0.5);
  
  // Read data from DSP.
  float valright = readDSPValue(RB_LEFT);
  float valleft = readDSPValue(RB_RIGHT);
  float valfix = readDSPValue(RB_FIX);

  //Calculate decibils.
  float levelleft = 20 * log10(valleft);
  float levelright = 20 * log10(valright);

  // Output data.
  Serial.print("Measure: left = ");
  Serial.print(levelleft);
  Serial.print("dB, right = ");
  Serial.print(levelright);
  Serial.print("dB, fix =");
  Serial.println(valfix);

  delay(500);
}
