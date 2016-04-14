/*
 * ADAU1701_REGISTERS.h
 *
 *  Created on: 15.01.2016
 *      Author: Fabian Geißler (fabian.geissler[at]tu-dresden.de)
 */

#ifndef ADAU18701_RegisterS_H
#define ADAU18701_RegisterS_H

#define ADAU_I2C_ADDRESS	0x68

// Parameter RAM
#define	ADAU_PARRAM				0		// address od the parameter RAM
#define	ADAU_PARRAM_SIZE		1024	// size of the parameter RAM in words
#define ADAU_PARRAM_WIDTH		4		// width of the RAM in bits

// Program RAM
#define	ADAU_PGRRAM				0x400	// address od the program RAM
#define	ADAU_PGRRAM_SIZE		1024	// size of the program RAM in words
#define ADAU_PGRRAM_WIDTH		5		// width of the RAM in bits

// Interface Registers, only interesting for SelfBoot mode.
// Data the ADAU writes back to the EEPROM and loads back at the next startup.
#define ADAU_IF0			0x800
#define	ADAU_IF_WIDTH		4

#define ADAU_IF1			0x801
#define	ADAU_IF_WIDTH		4

#define ADAU_IF2			0x802
#define	ADAU_IF_WIDTH		4

#define ADAU_IF3			0x803
#define	ADAU_IF_WIDTH		4

#define ADAU_IF4			0x804
#define	ADAU_IF_WIDTH		4

#define ADAU_IF5			0x805
#define	ADAU_IF_WIDTH		4

#define ADAU_IF6			0x806
#define	ADAU_IF_WIDTH		4

#define ADAU_IF7			0x807
#define	ADAU_IF_WIDTH		4

// GPIO Pin Setting Register
#define ADAU_GPIO			0x808
#define ADAU_GPIO_WIDTH		2

// Auxiliary ADC Registers, 1.11 fixpoint format with padding MSB zeros
#define ADAU_AUXADC0		0x809
#define ADAU_AUXADC0_WIDTH	2

#define ADAU_AUXADC1		0x80A
#define ADAU_AUXADC1_WIDTH	2

#define ADAU_AUXADC2		0x80B
#define ADAU_AUXADC2_WIDTH	2

#define ADAU_AUXADC3		0x80C
#define ADAU_AUXADC3_WIDTH	2

// SAFELOAD Data Registers
#define ADAU_SLD0			0x810
#define ADAU_SLD0_WIDTH		5

#define ADAU_SLD1			0x811
#define ADAU_SLD1_WIDTH		5

#define ADAU_SLD2			0x812
#define ADAU_SLD2_WIDTH		5

#define ADAU_SLD3			0x813
#define ADAU_SLD3_WIDTH		5

#define ADAU_SLD4			0x814
#define ADAU_SLD4_WIDTH		5

// SAFELOAD Address Registers
#define ADAU_SLA0			0x815
#define ADAU_SLA0_WIDTH		2

#define ADAU_SLA1			0x816
#define ADAU_SLA1_WIDTH		2

#define ADAU_SLA2			0x817
#define ADAU_SLA2_WIDTH		2

#define ADAU_SLA3			0x818
#define ADAU_SLA3_WIDTH		2

#define ADAU_SLA4			0x819
#define ADAU_SLA4_WIDTH		2

// Data Capture Registers
#define ADAU_DC			0x81A
#define ADAU_DC_WIDTH		2

#define ADAU_DC_RS				0x03			// register select
#define ADAU_DC_RS_MULX			0x00			// 00 - multiplier X
#define ADAU_DC_RS_MULY			0x01			// 01 - multiplier Y
#define ADAU_DC_RS_MUL_AC_OUT	0x02			// 10 - multiplier-accumulator output
#define ADAU_DC_RS_AC_FB		0x03			// 11 - accumulator feedback

#define ADAU_DC_PC				0x04			// 10 bit program counter

// Core Control Register
#define ADAU_CC				0x81C
#define ADAU_CC_WIDTH		2

#define ADAU_CCH_GD				0x30		// GPIO debounce
#define ADAU_CCH_GD_20MS		0x00		// 00 - 20ms
#define ADAU_CCH_GD_40MS		0x10		// 01 - 40ms
#define ADAU_CCH_GD_10MS		0x20		// 10 - 10ms
#define ADAU_CCH_GD_5MS			0x30		// 11 - 5ms

#define ADAU_CCH_AACW			0x01		// Auxiliary ADC Data Registers Control Port Write Mode
#define ADAU_CCL_GPCW			0x80		// GPIO Pin Setting Register Control Port Write Mode
#define ADAU_CCL_IFCW			0x40		// Interface Registers Control Port Write Mode
#define ADAU_CCL_IST			0x20		// Initiate Safeload Transfer.
#define ADAU_CCL_ADM			0x10		// Unmute ADCs
#define ADAU_CCL_DAM			0x08		// Unmute DACs
#define ADAU_CCL_CR				0x04		// Clear Internal Registers to 0, Low active

#define ADAU_CCL_SR				0x03		// Sample Rate
#define ADAU_CCL_SR_48KHZ		0x00		// 00 - 48/41.1kHz
#define ADAU_CCL_SR_96KHZ		0x01		// 01 - 96kHz
#define ADAU_CCL_SR_192KHZ		0x02		// 10 - 192kHz

// Serial Output Control Register
#define ADAU_SOC			0x81E
#define ADAU_SOC_WIDTH		2

#define ADAU_SOCL_OWL			0x03
#define ADAU_SOCL_OWL_24BIT		0x00
#define ADAU_SOCL_OWL_20BIT		0x01
#define ADAU_SOCL_OWL_16BIT		0x02

#define ADAU_SOCL_MSB			0x1C
#define ADAU_SOCL_MSB_DELAY1	0x00
#define ADAU_SOCL_MSB_DELAY0	0x04
#define ADAU_SOCL_MSB_DELAY8	0x08
#define ADAU_SOCL_MSB_DELAY12	0x0C
#define ADAU_SOCL_MSB_DELAY16	0x10

#define ADAU_SOCL_TDM			0x20
#define ADAU_SOCL_FST			0x40
#define ADAU_SOCL_OLF0			0x80
#define ADAU_SOCH_OLF1			0x01

#define ADAU_SOCH_OBF			0x06
#define ADAU_SOCH_OBF_DIV16		0x00
#define ADAU_SOCH_OBF_DIV8		0x02
#define ADAU_SOCH_OBF_DIV4		0x04
#define ADAU_SOCH_OBF_DIV2		0x06

#define ADAU_SOCH_MS			0x08
#define ADAU_SOCH_OBP			0x10
#define ADAU_SOCH_OLRP			0x20

// Serial input Control Register
#define ADAU_SIC			0x081F
#define ADAU_SIC_WIDTH		1

#define ADAU_SIC_MODE			0x07
#define ADAU_SIC_MODE_I2S		0x00
#define ADAU_SIC_MODE_LEFT		0x01
#define ADAU_SIC_MODE_TDM		0x02
#define ADAU_SIC_MODE_RIGHT24	0x03
#define ADAU_SIC_MODE_RIGHT20	0x04
#define ADAU_SIC_MODE_RIGHT18	0x05
#define ADAU_SIC_MODE_RIGHT16	0x06

#define ADAU_SIC_IBP			0x08
#define ADAU_SIC_ILRP			0x10

// Multipurpose Pin Configuration Register 0 (MP0 - MP5)
#define ADAU_MPPC0			0x0820
#define ADAU_MPPC0_WIDTH	3

// MP0
#define ADAU_MPPC0L_MP0				0x0F
#define ADAU_MPPC0L_MP0_INVERSE		0x08
#define ADAU_MPPC0L_MP0_INPUT_DB	0x00
#define ADAU_MPPC0L_MP0_INPUT		0x01
#define ADAU_MPPC0L_MP0_OUTPUT		0x02
#define ADAU_MPPC0L_MP0_OPENDRAIN	0x03
#define ADAU_MPPC0L_MP0_SDATA_IN0	0x04

// MP1
#define ADAU_MPPC0L_MP1				0xF0
#define ADAU_MPPC0L_MP1_INVERSE		0x80
#define ADAU_MPPC0L_MP1_INPUT_DB	0x00
#define ADAU_MPPC0L_MP1_INPUT		0x10
#define ADAU_MPPC0L_MP1_OUTPUT		0x20
#define ADAU_MPPC0L_MP1_OPENDRAIN	0x30
#define ADAU_MPPC0L_MP1_SDATA_IN1	0x40

// MP2
#define ADAU_MPPC0M_MP2				0x0F
#define ADAU_MPPC0M_MP2_INVERSE		0x08
#define ADAU_MPPC0M_MP2_INPUT_DB	0x00
#define ADAU_MPPC0M_MP2_INPUT		0x01
#define ADAU_MPPC0M_MP2_OUTPUT		0x02
#define ADAU_MPPC0M_MP2_OPENDRAIN	0x03
#define ADAU_MPPC0M_MP2_SDATA_IN2	0x04
#define ADAU_MPPC0M_MP2_ADC0		0x0F

// MP3
#define ADAU_MPPC0M_MP3				0xF0
#define ADAU_MPPC0M_MP3_INVERSE		0x80
#define ADAU_MPPC0M_MP3_INPUT_DB	0x00
#define ADAU_MPPC0M_MP3_INPUT		0x10
#define ADAU_MPPC0M_MP3_OUTPUT		0x20
#define ADAU_MPPC0M_MP3_OPENDRAIN	0x30
#define ADAU_MPPC0M_MP3_SDATA_IN3	0x40
#define ADAU_MPPC0M_MP3_ADC1		0xF0

// MP4
#define ADAU_MPPC0H_MP4				0x0F
#define ADAU_MPPC0H_MP4_INVERSE		0x08
#define ADAU_MPPC0H_MP4_INPUT_DB	0x00
#define ADAU_MPPC0H_MP4_INPUT		0x01
#define ADAU_MPPC0H_MP4_OUTPUT		0x02
#define ADAU_MPPC0H_MP4_OPENDRAIN	0x03
#define ADAU_MPPC0H_MP4_SDATA_LRIN	0x04

// MP5
#define ADAU_MPPC0H_MP5				0xF0
#define ADAU_MPPC0H_MP5_INVERSE		0x80
#define ADAU_MPPC0H_MP5_INPUT_DB	0x00
#define ADAU_MPPC0H_MP5_INPUT		0x10
#define ADAU_MPPC0H_MP5_OUTPUT		0x20
#define ADAU_MPPC0H_MP5_OPENDRAIN	0x30
#define ADAU_MPPC0H_MP5_SDATA_BCIN	0x40

// Multipurpose Pin Configuration Register 1 (MP6 - MP11)
#define ADAU_MPPC1			0x0821
#define ADAU_MPPC1_WIDTH	3

// MP6
#define ADAU_MPPC1L_MP6				0x0F
#define ADAU_MPPC1L_MP6_INVERSE		0x08
#define ADAU_MPPC1L_MP6_INPUT_DB	0x00
#define ADAU_MPPC1L_MP6_INPUT		0x01
#define ADAU_MPPC1L_MP6_OUTPUT		0x02
#define ADAU_MPPC1L_MP6_OPENDRAIN	0x03
#define ADAU_MPPC1L_MP6_SDATA_OUT0	0x04

// MP7
#define ADAU_MPPC1L_MP7				0xF0
#define ADAU_MPPC1L_MP7_INVERSE		0x80
#define ADAU_MPPC1L_MP7_INPUT_DB	0x00
#define ADAU_MPPC1L_MP7_INPUT		0x10
#define ADAU_MPPC1L_MP7_OUTPUT		0x20
#define ADAU_MPPC1L_MP7_OPENDRAIN	0x30
#define ADAU_MPPC1L_MP7_SDATA_OUT1	0x40

// MP8
#define ADAU_MPPC1M_MP8				0x0F
#define ADAU_MPPC1M_MP8_INVERSE		0x08
#define ADAU_MPPC1M_MP8_INPUT_DB	0x00
#define ADAU_MPPC1M_MP8_INPUT		0x01
#define ADAU_MPPC1M_MP8_OUTPUT		0x02
#define ADAU_MPPC1M_MP8_OPENDRAIN	0x03
#define ADAU_MPPC1M_MP8_SDATA_OUT2	0x04
#define ADAU_MPPC1M_MP8_ADC2		0x0F

// MP9
#define ADAU_MPPC1M_MP9				0xF0
#define ADAU_MPPC1M_MP9_INVERSE		0x80
#define ADAU_MPPC1M_MP9_INPUT_DB	0x00
#define ADAU_MPPC1M_MP9_INPUT		0x10
#define ADAU_MPPC1M_MP9_OUTPUT		0x20
#define ADAU_MPPC1M_MP9_OPENDRAIN	0x30
#define ADAU_MPPC1M_MP9_SDATA_OUT3	0x40
#define ADAU_MPPC1M_MP9_ADC3		0xF0

// MP10
#define ADAU_MPPC1H_MP10			0x0F
#define ADAU_MPPC1H_MP10_INVERSE	0x08
#define ADAU_MPPC1H_MP10_INPUT_DB	0x00
#define ADAU_MPPC1H_MP10_INPUT		0x01
#define ADAU_MPPC1H_MP10_OUTPUT		0x02
#define ADAU_MPPC1H_MP10_OPENDRAIN	0x03
#define ADAU_MPPC1H_MP10_SDATA_LROUT	0x04

// MP11
#define ADAU_MPPC1H_MP11			0xF0
#define ADAU_MPPC1H_MP11_INVERSE	0x80
#define ADAU_MPPC1H_MP11_INPUT_DB	0x00
#define ADAU_MPPC1H_MP11_INPUT		0x10
#define ADAU_MPPC1H_MP11_OUTPUT		0x20
#define ADAU_MPPC1H_MP11_OPENDRAIN	0x30
#define ADAU_MPPC1H_MP11_SDATA_BCOUT	0x40

// Auxilliary ADC and Power Control
#define ADAU_ADCPC			0x0822
#define	ADAU_ADCPC_WIDTH	2

#define	ADAU_ADCPCL_DAC3PD			0x01
#define	ADAU_ADCPCL_DAC2PD			0x02
#define	ADAU_ADCPCL_DAC1PD			0x04
#define	ADAU_ADCPCL_DAC0PD			0x08
#define	ADAU_ADCPCL_VRPD			0x20
#define	ADAU_ADCPCL_VBPD			0x40
#define	ADAU_ADCPCL_ADCPD			0x80

#define	ADAU_ADCPCH_AADC_FIL		0x03
#define	ADAU_ADCPCH_AADC_FIL_4BIT	0x00
#define	ADAU_ADCPCH_AADC_FIL_5BIT	0x01
#define	ADAU_ADCPCH_AADC_FIL_NONE	0x02
#define	ADAU_ADCPCH_AADC_FIL_LPONLY	0x03

// Auxilliary ADC Enable
#define ADAU_AUXADCE		0x0824
#define ADAU_AUXADCE_WIDTH	2

#define ADAU_AUXADCEH_AAEN		0x80

// Oscillator Power Down
#define ADAU_OSCPD			0x0826
#define ADAU_OSCPD_WIDTH	2

#define ADAU_OSCPDH_OPD			0x80

// DAC Setup
#define ADAU_DACSU			0x0827
#define ADAU_DACSU_WIDTH	2

#define ADAU_DACSUL_DS			0x03
#define ADAU_DACSUL_DS_INIT		0x01

#endif
