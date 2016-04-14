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



    FreeDSP I2C communication guide - PART 3

    In part 3 our shematic is a simple two-channel biquad filter.

    With this schematic we want to perform safeload writes to the param
    memory of the DSP. If we would perform direct writes we have no control,
    at which time the parameters are changed. Maybe two of five biquad
    parameters are changed when the filter is calculated. This would be
    very bad and lead to transients, high gain, distortion and maybe
    even damaged equipment.

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
#define DSP_SAMPLERATE 48000UL

// Defines for serial communication.
#define COM_BAUD    9600
#define COM_BUFSIZE 25

// Filter adresses:
#define BIQUAD_B0   0
#define BIQUAD_B1   1
#define BIQUAD_B2   2
#define BIQUAD_A1   3
#define BIQUAD_A2   4

// Library class initialisation.
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
                                              0x01 , 0x00 , 0x17 , 0x00 , 0x00 , 0x00 , 0x00 , 0x80 ,
                                              0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 ,
                                              0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 ,
                                              0x00 , 0x00 , 0x01 , 0x01 , 0x43 , 0x00 , 0x04 , 0x00 ,
                                              0x00 , 0x00 , 0x00 , 0x00 , 0x01 , 0x00 , 0x10 , 0x00 ,
                                              0xE8 , 0x01 , 0x00 , 0x00 , 0x00 , 0x00 , 0x01 , 0x00 ,
                                              0x28 , 0x00 , 0xE8 , 0x01 , 0x00 , 0x52 , 0x03 , 0x20 ,
                                              0x01 , 0x00 , 0x4A , 0x04 , 0x22 , 0x01 , 0x00 , 0x3A ,
                                              0x03 , 0x34 , 0x01 , 0x00 , 0x32 , 0x04 , 0x22 , 0x01 ,
                                              0x00 , 0x12 , 0x00 , 0x22 , 0x01 , 0x00 , 0x0A , 0x01 ,
                                              0x22 , 0x01 , 0x00 , 0x02 , 0x02 , 0x22 , 0x01 , 0x00 ,
                                              0x00 , 0x00 , 0x00 , 0x01 , 0x00 , 0x40 , 0x00 , 0xE2 ,
                                              0x01 , 0x00 , 0x58 , 0x00 , 0xF2 , 0x01 , 0x00 , 0x82 ,
                                              0x03 , 0x20 , 0x01 , 0x00 , 0x7A , 0x04 , 0x22 , 0x01 ,
                                              0x00 , 0x6A , 0x03 , 0x34 , 0x01 , 0x00 , 0x62 , 0x04 ,
                                              0x22 , 0x01 , 0x00 , 0x2A , 0x00 , 0x22 , 0x01 , 0x00 ,
                                              0x22 , 0x01 , 0x22 , 0x01 , 0x00 , 0x1A , 0x02 , 0x22 ,
                                              0x01 , 0x00 , 0x00 , 0x00 , 0x00 , 0x01 , 0x00 , 0x70 ,
                                              0x00 , 0xE2 , 0x01 , 0x00 , 0x88 , 0x00 , 0xF2 , 0x01 ,
                                              0x00 , 0x41 , 0x08 , 0x20 , 0x01 , 0xFF , 0x68 , 0x00 ,
                                              0x02 , 0x01 , 0x00 , 0x71 , 0x08 , 0x20 , 0x01 , 0xFF ,
                                              0x70 , 0x00 , 0x02 , 0x01 , 0x00 , 0x00 , 0x00 , 0x00 ,
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
                                              0x01 , 0x00 , 0x05 , 0x00 , 0x08 , 0x1C , 0x00 , 0x1C ,
                                              0x01 , 0x00 , 0x04 , 0x00 , 0x08 , 0x1D , 0x08 , 0x01 ,
                                              0x00 , 0x05 , 0x00 , 0x08 , 0x1E , 0x00 , 0x00 , 0x01 ,
                                              0x00 , 0x04 , 0x00 , 0x08 , 0x1F , 0x00 , 0x01 , 0x00 ,
                                              0x06 , 0x00 , 0x08 , 0x20 , 0x00 , 0x00 , 0x00 , 0x01 ,
                                              0x00 , 0x06 , 0x00 , 0x08 , 0x21 , 0x00 , 0x00 , 0x00 ,
                                              0x01 , 0x00 , 0x05 , 0x00 , 0x08 , 0x22 , 0x00 , 0x00 ,
                                              0x01 , 0x00 , 0x05 , 0x00 , 0x08 , 0x23 , 0x00 , 0x00 ,
                                              0x01 , 0x00 , 0x05 , 0x00 , 0x08 , 0x24 , 0x80 , 0x00 ,
                                              0x01 , 0x00 , 0x05 , 0x00 , 0x08 , 0x25 , 0x00 , 0x00 ,
                                              0x01 , 0x00 , 0x05 , 0x00 , 0x08 , 0x26 , 0x00 , 0x00 ,
                                              0x01 , 0x00 , 0x05 , 0x00 , 0x08 , 0x27 , 0x00 , 0x01 ,
                                              0x01 , 0x00 , 0x05 , 0x00 , 0x08 , 0x1C , 0x00 , 0x1C ,
                                              0x06 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 ,
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
  Serial.println("Feel free to write values now");
  Serial.println("Write any number to set the frequency of the lowpass filter.");
}

void waitForLine(char* line, uint8_t len)
{
  // Wait until new data has been sent.
  while (Serial.available() == 0);
  uint8_t idx = Serial.readBytesUntil('\n', line, len - 1);
  line[idx] = 0;
}

void getHighpass(float frequency, uint32_t samplerate, float qfactor, float *params)
{
  if (qfactor == 0)
    return;

  float omega = 2.0f * PI * frequency / (float)samplerate;
  float cs = cos(omega);
  float sn = sin(omega);
  float alpha = sinh(1 / (2.0f * qfactor)) * sn;
  float a0 = 1 + alpha;

  // order: b0, b1, b2, a1, a2
  params[0] = (float)((1 + cs) / (2.0f * a0));
  params[1] = (float)((-1 - cs) / a0);
  params[2] = (float)((1 + cs) / (2.0f * a0));
  params[3] = -(float)((-2 * cs) / a0);
  params[4] = -(float)((1 - alpha) / a0);

  // IMPORTANT: The a parameters must always be negated,
  // as they are normally substracted but the DSP can only
  // add them up.
}

void getLowpass(float frequency, uint32_t samplerate, float qfactor, float *params)
{
  if (qfactor == 0)
    return;

  float omega = 2.0f * PI * frequency / (float)samplerate;
  float cs = cos(omega);
  float sn = sin(omega);
  float alpha = sinh(1 / (2.0f * qfactor)) * sn;
  float a0 = 1 + alpha;

  // order: b0, b1, b2, a1, a2
  params[0] = (float)((1 - cs) / (2.0f * a0));
  params[1] = (float)((1 - cs) / a0);
  params[2] = (float)((1 - cs) / (2.0f * a0));
  params[3] = -(float)((-2 * cs) / a0);
  params[4] = -(float)((1 - alpha) / a0);

  // IMPORTANT: The a parameters must always be negated,
  // as they are normally substracted but the DSP can only
  // add them up.
}

void loop() {
  char line[COM_BUFSIZE];

  waitForLine(line, COM_BUFSIZE);

  // Read frequency from line.
  int freq = atoi(line);

  // Protect against unstable filter 
  if (freq >= DSP_SAMPLERATE / 2)
  {
    Serial.print("Cannot set cut-off frequency above half samplerate, as the filter will become unstable!");
	return;
  }
  
  Serial.print("Set filter frequency to ");
  Serial.println(freq, DEC);

  float biquad[5];

  // Calculate lowpass or highpass biquad parameters.
  //getHighpass((float)freq, DSP_SAMPLERATE, 0.7f, biquad);
  getLowpass((float)freq, DSP_SAMPLERATE, 0.7f, biquad);

  // First write safeload address registers.
  uint8_t address[2] = {0, 0};

  // Since our addresses are all smaller than 256 we only need to change the lower part.
  address[1] = BIQUAD_B0;
  mDSP.write(ADAU_SLA0, 2, address);

  address[1] = BIQUAD_B1;
  mDSP.write(ADAU_SLA1, 2, address);

  address[1] = BIQUAD_B2;
  mDSP.write(ADAU_SLA2, 2, address);

  address[1] = BIQUAD_A1;
  mDSP.write(ADAU_SLA3, 2, address);

  address[1] = BIQUAD_A2;
  mDSP.write(ADAU_SLA4, 2, address);

  // Byte array for safeload registers.
  // One register is five bytes wide.
  uint8_t bytes[5];
  int32_t val;

  Serial.println("Biquad Values: ");

  for (int i = 0; i < 5; ++i)
  {
    // Convert parameters to 5.23 fixpoint format.
    val = (int32_t)(8388608.0f * biquad[i]); // x * (1 << 23)

    // Reverse byte order and store in array.
    bytes[0] = 0;
    bytes[1] = ((uint8_t*)(&val))[3];
    bytes[2] = ((uint8_t*)(&val))[2];
    bytes[3] = ((uint8_t*)(&val))[1];
    bytes[4] = ((uint8_t*)(&val))[0];

    // Write to safeload register.
    // Here we can see, that we can also iterate through the safeload
    // registers, as their addresses are nicely aligned.
    mDSP.write(ADAU_SLD0 + i, 5, bytes);

    // Make some user interaction.

    for (int j = 0; j < 5; ++j)
    {
      if (bytes[j] < 16)
        Serial.print("0");
      Serial.print(bytes[j], HEX);
      Serial.print(" ");
    }

    Serial.print("\n");
  }

  // Now after all the data is written we need to initiate a safeload transfer.
  // Therefore we need to set the IST bit in the Core Control Register.
  uint8_t ccr[ADAU_CC_WIDTH];

  // Read CCR content.
  mDSP.read(ADAU_CC, ADAU_CC_WIDTH, ccr);

  // Modify CCR.
  ccr[1] |= ADAU_CCL_IST;

  // Write modified CCR back.
  mDSP.write(ADAU_CC, ADAU_CC_WIDTH, ccr);
}
