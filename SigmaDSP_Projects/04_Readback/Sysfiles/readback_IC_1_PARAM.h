/*
 * File:           Z:\Dokumente\Analog Devices\SigmaStudio 3.12\Projects\I2C Guide\04_Readback\Sysfiles\readback_IC_1_PARAM.h
 *
 * Created:        Saturday, January 16, 2016 6:58:47 PM
 * Description:    readback:IC 1 parameter RAM definitions.
 *
 * This software is distributed in the hope that it will be useful,
 * but is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * This software may only be used to program products purchased from
 * Analog Devices for incorporation by you into audio products that
 * are intended for resale to audio product end users. This software
 * may not be distributed whole or in any part to third parties.
 *
 * Copyright ©2016 Analog Devices, Inc. All rights reserved.
 */
#ifndef __READBACK_IC_1_PARAM_H__
#define __READBACK_IC_1_PARAM_H__


/* Module DC1 - DC Input Entry*/
#define MOD_DC1_COUNT                                  1
#define MOD_DC1_DEVICE                                 "IC1"
#define MOD_DC1_DCINPALG1_ADDR                         0
#define MOD_DC1_DCINPALG1_FIXPT                        0x000CCCCC
#define MOD_DC1_DCINPALG1_VALUE                        SIGMASTUDIOTYPE_FIXPOINT_CONVERT(0.1)
#define MOD_DC1_DCINPALG1_TYPE                         SIGMASTUDIOTYPE_FIXPOINT

/* Module AvgEnv1 - Running Average*/
#define MOD_AVGENV1_COUNT                              3
#define MOD_AVGENV1_DEVICE                             "IC1"
#define MOD_AVGENV1_ALG0_MONORUNAVGDETECTALG1RMS_ADDR  1
#define MOD_AVGENV1_ALG0_MONORUNAVGDETECTALG1RMS_FIXPT 0x000012DE
#define MOD_AVGENV1_ALG0_MONORUNAVGDETECTALG1RMS_VALUE SIGMASTUDIOTYPE_FIXPOINT_CONVERT(0.000575811989360853)
#define MOD_AVGENV1_ALG0_MONORUNAVGDETECTALG1RMS_TYPE  SIGMASTUDIOTYPE_FIXPOINT
#define MOD_AVGENV1_ALG0_MONORUNAVGDETECTALG1HOLD_ADDR 2
#define MOD_AVGENV1_ALG0_MONORUNAVGDETECTALG1HOLD_FIXPT 0x000001E0
#define MOD_AVGENV1_ALG0_MONORUNAVGDETECTALG1HOLD_VALUE SIGMASTUDIOTYPE_INTEGER_CONVERT(480)
#define MOD_AVGENV1_ALG0_MONORUNAVGDETECTALG1HOLD_TYPE SIGMASTUDIOTYPE_INTEGER
#define MOD_AVGENV1_ALG0_MONORUNAVGDETECTALG1DECAY_ADDR 3
#define MOD_AVGENV1_ALG0_MONORUNAVGDETECTALG1DECAY_FIXPT 0x00000012
#define MOD_AVGENV1_ALG0_MONORUNAVGDETECTALG1DECAY_VALUE SIGMASTUDIOTYPE_FIXPOINT_CONVERT(2.17013888888889E-06)
#define MOD_AVGENV1_ALG0_MONORUNAVGDETECTALG1DECAY_TYPE SIGMASTUDIOTYPE_FIXPOINT

/* Module AvgEnv1_2 - Running Average*/
#define MOD_AVGENV1_2_COUNT                            3
#define MOD_AVGENV1_2_DEVICE                           "IC1"
#define MOD_AVGENV1_2_ALG0_MONORUNAVGDETECTALG2RMS_ADDR 4
#define MOD_AVGENV1_2_ALG0_MONORUNAVGDETECTALG2RMS_FIXPT 0x000012DE
#define MOD_AVGENV1_2_ALG0_MONORUNAVGDETECTALG2RMS_VALUE SIGMASTUDIOTYPE_FIXPOINT_CONVERT(0.000575811989360853)
#define MOD_AVGENV1_2_ALG0_MONORUNAVGDETECTALG2RMS_TYPE SIGMASTUDIOTYPE_FIXPOINT
#define MOD_AVGENV1_2_ALG0_MONORUNAVGDETECTALG2HOLD_ADDR 5
#define MOD_AVGENV1_2_ALG0_MONORUNAVGDETECTALG2HOLD_FIXPT 0x000001E0
#define MOD_AVGENV1_2_ALG0_MONORUNAVGDETECTALG2HOLD_VALUE SIGMASTUDIOTYPE_INTEGER_CONVERT(480)
#define MOD_AVGENV1_2_ALG0_MONORUNAVGDETECTALG2HOLD_TYPE SIGMASTUDIOTYPE_INTEGER
#define MOD_AVGENV1_2_ALG0_MONORUNAVGDETECTALG2DECAY_ADDR 6
#define MOD_AVGENV1_2_ALG0_MONORUNAVGDETECTALG2DECAY_FIXPT 0x00000012
#define MOD_AVGENV1_2_ALG0_MONORUNAVGDETECTALG2DECAY_VALUE SIGMASTUDIOTYPE_FIXPOINT_CONVERT(2.17013888888889E-06)
#define MOD_AVGENV1_2_ALG0_MONORUNAVGDETECTALG2DECAY_TYPE SIGMASTUDIOTYPE_FIXPOINT

/* Module ReadBack1_3 - DSP Readback*/
#define MOD_READBACK1_3_COUNT                          2
#define MOD_READBACK1_3_DEVICE                         "IC1"
#define MOD_READBACK1_3_ALG0_VAL1_ADDR                 2074
#define MOD_READBACK1_3_ALG0_VAL1_VALUE                SIGMASTUDIOTYPE_5_19_CONVERT(0)
#define MOD_READBACK1_3_ALG0_VAL1_TYPE                 SIGMASTUDIOTYPE_5_19
#define MOD_READBACK1_3_ALG0_VAL0_ADDR                 2074
#define MOD_READBACK1_3_ALG0_VAL0_VALUES               SIGMASTUDIOTYPE_SPECIAL(0x010A)
#define MOD_READBACK1_3_ALG0_VAL0_TYPE                 SIGMASTUDIOTYPE_SPECIAL
#define MOD_READBACK1_3_ALG0_VAL0_READBACK_ADDR        0

/* Module ReadBack1_2 - DSP Readback*/
#define MOD_READBACK1_2_COUNT                          2
#define MOD_READBACK1_2_DEVICE                         "IC1"
#define MOD_READBACK1_2_ALG0_VAL1_ADDR                 2074
#define MOD_READBACK1_2_ALG0_VAL1_VALUE                SIGMASTUDIOTYPE_5_19_CONVERT(0)
#define MOD_READBACK1_2_ALG0_VAL1_TYPE                 SIGMASTUDIOTYPE_5_19
#define MOD_READBACK1_2_ALG0_VAL0_ADDR                 2074
#define MOD_READBACK1_2_ALG0_VAL0_VALUES               SIGMASTUDIOTYPE_SPECIAL(0x0122)
#define MOD_READBACK1_2_ALG0_VAL0_TYPE                 SIGMASTUDIOTYPE_SPECIAL
#define MOD_READBACK1_2_ALG0_VAL0_READBACK_ADDR        0

/* Module ReadBack1 - DSP Readback*/
#define MOD_READBACK1_COUNT                            2
#define MOD_READBACK1_DEVICE                           "IC1"
#define MOD_READBACK1_ALG0_VAL1_ADDR                   2074
#define MOD_READBACK1_ALG0_VAL1_VALUE                  SIGMASTUDIOTYPE_5_19_CONVERT(0)
#define MOD_READBACK1_ALG0_VAL1_TYPE                   SIGMASTUDIOTYPE_5_19
#define MOD_READBACK1_ALG0_VAL0_ADDR                   2074
#define MOD_READBACK1_ALG0_VAL0_VALUES                 SIGMASTUDIOTYPE_SPECIAL(0x0116)
#define MOD_READBACK1_ALG0_VAL0_TYPE                   SIGMASTUDIOTYPE_SPECIAL
#define MOD_READBACK1_ALG0_VAL0_READBACK_ADDR          0

#endif