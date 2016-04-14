/*
   register_settings.ino

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



    FreeDSP I2C communication guide - PART 2

    This time we use a schematic which routes ADC0:1 to DAC0:1 and has a
    oscillator on DAC2:3. In this part we want to dive deeper into the DSP.
    We are going to change the ADAUs internal registers. Based on the input
    via the serial port we will change the state of the DSPs GPIO pins and
    mute the ADCs or DACs.

    IMPORTANT: See ADAU_REGISTERS.h for register defines and the datasheet
               for detailed description.

*/

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


// Defines for serial communication.
#define COM_BAUD   9600

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
                                              0x01 , 0x00 , 0x13 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 ,
                                              0x00 , 0xFF , 0x00 , 0x02 , 0xAA , 0xAE , 0x00 , 0x80 ,
                                              0x00 , 0x00 , 0x00 , 0x20 , 0x26 , 0xF3 , 0x01 , 0x01 ,
                                              0x43 , 0x00 , 0x04 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 ,
                                              0x01 , 0x00 , 0x00 , 0x00 , 0xE8 , 0x01 , 0x00 , 0x00 ,
                                              0x00 , 0x00 , 0x01 , 0x00 , 0x08 , 0x00 , 0xE8 , 0x01 ,
                                              0x00 , 0x29 , 0x1F , 0x20 , 0x01 , 0x00 , 0x19 , 0x08 ,
                                              0x22 , 0x01 , 0x00 , 0x02 , 0x01 , 0xA0 , 0x01 , 0xFF ,
                                              0xE1 , 0x18 , 0x22 , 0x01 , 0x00 , 0x02 , 0x00 , 0xA0 ,
                                              0x01 , 0x00 , 0x20 , 0x00 , 0xEA , 0x01 , 0x00 , 0x30 ,
                                              0x00 , 0xF2 , 0x01 , 0x00 , 0x20 , 0x00 , 0xC0 , 0x01 ,
                                              0x00 , 0x00 , 0x00 , 0x00 , 0x01 , 0xFF , 0xF0 , 0x01 ,
                                              0x21 , 0x01 , 0x00 , 0x00 , 0x00 , 0xA1 , 0x01 , 0xFF ,
                                              0xE1 , 0x08 , 0x22 , 0x41 , 0x00 , 0x38 , 0x00 , 0xE2 ,
                                              0x01 , 0x00 , 0x30 , 0x00 , 0xC0 , 0x01 , 0x00 , 0x3F ,
                                              0xFF , 0x20 , 0x01 , 0xFF , 0xE1 , 0x08 , 0x22 , 0x01 ,
                                              0x00 , 0x40 , 0x00 , 0xE2 , 0x01 , 0x00 , 0x42 , 0x02 ,
                                              0x20 , 0x01 , 0x00 , 0x10 , 0x00 , 0xE2 , 0x01 , 0x00 ,
                                              0x01 , 0x08 , 0x20 , 0x01 , 0xFF , 0x68 , 0x00 , 0x02 ,
                                              0x01 , 0x00 , 0x09 , 0x08 , 0x20 , 0x01 , 0xFF , 0x70 ,
                                              0x00 , 0x02 , 0x01 , 0x00 , 0x12 , 0x03 , 0x20 , 0x01 ,
                                              0x00 , 0x50 , 0x00 , 0xE2 , 0x01 , 0x00 , 0x00 , 0x00 ,
                                              0x00 , 0x01 , 0x00 , 0x51 , 0x08 , 0x20 , 0x01 , 0xFF ,
                                              0x80 , 0x00 , 0x02 , 0x01 , 0x00 , 0x51 , 0x08 , 0x20 ,
                                              0x01 , 0xFF , 0x78 , 0x00 , 0x02 , 0x01 , 0x00 , 0x00 ,
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
                                              0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00
                                            };

void initRegisters()
{
  // Before editing the DSPs GPIO pins we need to tell it, that we want to control them.
  // Therefore we need to set the GPCW bit in the Core Control Register.
  uint8_t ccr[ADAU_CC_WIDTH];

  // Read CCR content.
  mDSP.read(ADAU_CC, ADAU_CC_WIDTH, ccr);

  // Modify CCR.
  ccr[1] |= ADAU_CCL_GPCW;

  // Write modified CCR back.
  mDSP.write(ADAU_CC, ADAU_CC_WIDTH, ccr);

  // Now we will set all the GPIOs to output
  // GPIO function select needs 4 bits per pin,
  // so per byte we can configure only two pins.
  uint8_t mppcr[ADAU_MPPC0_WIDTH];

  mppcr[2] = ADAU_MPPC0L_MP0_OUTPUT | ADAU_MPPC0L_MP1_OUTPUT;
  mppcr[1] = ADAU_MPPC0M_MP2_OUTPUT | ADAU_MPPC0M_MP3_OUTPUT;
  mppcr[0] = ADAU_MPPC0H_MP4_OUTPUT | ADAU_MPPC0H_MP5_OUTPUT;

  // Write MPPC0.
  mDSP.write(ADAU_MPPC0, ADAU_MPPC0_WIDTH, mppcr);

  mppcr[2] = ADAU_MPPC1L_MP6_OUTPUT | ADAU_MPPC1L_MP7_OUTPUT;
  mppcr[1] = ADAU_MPPC1M_MP8_OUTPUT | ADAU_MPPC1M_MP9_OUTPUT;
  mppcr[0] = ADAU_MPPC1H_MP10_OUTPUT | ADAU_MPPC1H_MP11_OUTPUT;

  // Write MPPC1.
  mDSP.write(ADAU_MPPC1, ADAU_MPPC1_WIDTH, mppcr);
}

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
  Serial.println("'0' to 'B' (hex number) toggle the according GPIO pins.");
  Serial.println("'X' mutes the ADCs.");
  Serial.println("'Y' mutes the DACs.");

  initRegisters();
}

void toggleDSPGPIO(uint8_t pin)
{
  if (pin > 11)
    return;

  // ADAU_GPIO is two bytes wide
  uint8_t gpio[ADAU_GPIO_WIDTH];

  // Read GPIO pin setting register.
  mDSP.read(ADAU_GPIO, ADAU_GPIO_WIDTH, gpio);

  uint8_t pin_mask;

  // In the GPIO register each bit represents the corresponding pin.
  // Bits 15:12 are not used and should be zero.
  // Calculate pin mask and toggle pin using XOR
  if (pin > 7)
  {
    pin_mask = 1 << (pin - 8);
    gpio[0] ^= pin_mask;
  }
  else
  {
    pin_mask = 1 << pin;
    gpio[1] ^= pin_mask;
  }

  // Write modified GPIO register back.
  mDSP.write(ADAU_GPIO, ADAU_GPIO_WIDTH, gpio);
}

void toggleADCs()
{
  uint8_t ccr[ADAU_CC_WIDTH];

  // Read CCR content.
  mDSP.read(ADAU_CC, ADAU_CC_WIDTH, ccr);

  // Modify CCR. The ADM bit is low active and mutes the ADCs.
  ccr[1] ^= ADAU_CCL_ADM;

  // Write modified CCR back.
  mDSP.write(ADAU_CC, ADAU_CC_WIDTH, ccr);
}

void toggleDACs()
{
  uint8_t ccr[ADAU_CC_WIDTH];

  // Read CCR content.
  mDSP.read(ADAU_CC, ADAU_CC_WIDTH, ccr);

  // Modify CCR. The DAM bit is low active and mutes the DACs.
  ccr[1] ^= ADAU_CCL_DAM;

  // Write modified CCR back.
  mDSP.write(ADAU_CC, ADAU_CC_WIDTH, ccr);
}

void loop() {
  // Wait until new data has been sent.
  while (Serial.available() == 0);

  // Read one byte.
  char val = Serial.read();

  switch (val)
  {
    case 'x':
    case 'X':
      // Mute/Unmute ADCs.
      toggleADCs();
      Serial.println("ADC state toggled!");
      break;

    case 'y':
    case 'Y':
      // Mute/Unmute DACs.
      toggleDACs();
      Serial.println("DAC state toggled!");
      break;

    case '0':
    case '1':
    case '2':
    case '3':
    case '4':
    case '5':
    case '6':
    case '7':
    case '8':
    case '9':
      toggleDSPGPIO(val - '0');
      Serial.println("GPIO toggled!");
      break;

    case 'a':
    case 'b':
      toggleDSPGPIO(val - 'a' + 10);
      Serial.println("GPIO toggled!");
      break;

    case 'A':
    case 'B':
      toggleDSPGPIO(val - 'A' + 10);
      Serial.println("GPIO toggled!");
      break;
  }
}
