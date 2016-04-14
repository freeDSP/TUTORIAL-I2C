/*
 * File:           Z:\Dokumente\Analog Devices\SigmaStudio 3.12\Projects\I2C Guide\01_Volume_Control\Sysfiles\volume_control_IC_1_PARAM.h
 *
 * Created:        Saturday, January 16, 2016 11:04:34 AM
 * Description:    volume_control:IC 1 parameter RAM definitions.
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
#ifndef __VOLUME_CONTROL_IC_1_PARAM_H__
#define __VOLUME_CONTROL_IC_1_PARAM_H__


/* Module Single 1 - Single Volume*/
#define MOD_SINGLE1_COUNT                              2
#define MOD_SINGLE1_DEVICE                             "IC1"
#define MOD_SINGLE1_ALG0_GAIN1940ALGNS1_ADDR           0
#define MOD_SINGLE1_ALG0_GAIN1940ALGNS1_FIXPT          0x00800000
#define MOD_SINGLE1_ALG0_GAIN1940ALGNS1_VALUE          SIGMASTUDIOTYPE_FIXPOINT_CONVERT(1)
#define MOD_SINGLE1_ALG0_GAIN1940ALGNS1_TYPE           SIGMASTUDIOTYPE_FIXPOINT
#define MOD_SINGLE1_ALG1_GAIN1940ALGNS2_ADDR           1
#define MOD_SINGLE1_ALG1_GAIN1940ALGNS2_FIXPT          0x00800000
#define MOD_SINGLE1_ALG1_GAIN1940ALGNS2_VALUE          SIGMASTUDIOTYPE_FIXPOINT_CONVERT(1)
#define MOD_SINGLE1_ALG1_GAIN1940ALGNS2_TYPE           SIGMASTUDIOTYPE_FIXPOINT

#endif
