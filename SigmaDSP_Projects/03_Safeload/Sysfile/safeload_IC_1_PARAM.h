/*
 * File:           Z:\Dokumente\Analog Devices\SigmaStudio 3.12\Projects\I2C Guide\03_Safeload\Sysfile\safeload_IC_1_PARAM.h
 *
 * Created:        Saturday, January 16, 2016 3:59:46 PM
 * Description:    safeload:IC 1 parameter RAM definitions.
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
#ifndef __SAFELOAD_IC_1_PARAM_H__
#define __SAFELOAD_IC_1_PARAM_H__


/* Module Gen Filter1 - General (2nd order)*/
#define MOD_GENFILTER1_COUNT                           5
#define MOD_GENFILTER1_DEVICE                          "IC1"
#define MOD_GENFILTER1_ALG0_STAGE0_B0_ADDR             0
#define MOD_GENFILTER1_ALG0_STAGE0_B0_FIXPT            0x00800000
#define MOD_GENFILTER1_ALG0_STAGE0_B0_VALUE            SIGMASTUDIOTYPE_FIXPOINT_CONVERT(1)
#define MOD_GENFILTER1_ALG0_STAGE0_B0_TYPE             SIGMASTUDIOTYPE_FIXPOINT
#define MOD_GENFILTER1_ALG0_STAGE0_B1_ADDR             1
#define MOD_GENFILTER1_ALG0_STAGE0_B1_FIXPT            0x00000000
#define MOD_GENFILTER1_ALG0_STAGE0_B1_VALUE            SIGMASTUDIOTYPE_FIXPOINT_CONVERT(0)
#define MOD_GENFILTER1_ALG0_STAGE0_B1_TYPE             SIGMASTUDIOTYPE_FIXPOINT
#define MOD_GENFILTER1_ALG0_STAGE0_B2_ADDR             2
#define MOD_GENFILTER1_ALG0_STAGE0_B2_FIXPT            0x00000000
#define MOD_GENFILTER1_ALG0_STAGE0_B2_VALUE            SIGMASTUDIOTYPE_FIXPOINT_CONVERT(0)
#define MOD_GENFILTER1_ALG0_STAGE0_B2_TYPE             SIGMASTUDIOTYPE_FIXPOINT
#define MOD_GENFILTER1_ALG0_STAGE0_A1_ADDR             3
#define MOD_GENFILTER1_ALG0_STAGE0_A1_FIXPT            0x00000000
#define MOD_GENFILTER1_ALG0_STAGE0_A1_VALUE            SIGMASTUDIOTYPE_FIXPOINT_CONVERT(0)
#define MOD_GENFILTER1_ALG0_STAGE0_A1_TYPE             SIGMASTUDIOTYPE_FIXPOINT
#define MOD_GENFILTER1_ALG0_STAGE0_A2_ADDR             4
#define MOD_GENFILTER1_ALG0_STAGE0_A2_FIXPT            0x00000000
#define MOD_GENFILTER1_ALG0_STAGE0_A2_VALUE            SIGMASTUDIOTYPE_FIXPOINT_CONVERT(0)
#define MOD_GENFILTER1_ALG0_STAGE0_A2_TYPE             SIGMASTUDIOTYPE_FIXPOINT

#endif
