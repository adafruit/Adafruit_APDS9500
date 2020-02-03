/*!
 *  @file Adafruit_APDS9500.h
 *
 * 	I2C Driver for the Adafruit APDS9500 Gesture Sensor library
 *
 * 	This is a library for the Adafruit APDS9500 breakout:
 * 	https://www.adafruit.com/products/889X
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!

  *	BSD license (see license.txt)

   *
 * Derived from:
 * APDS9500_gestures.ino
 * by: Kris Winer, Tlera Corporation, Copyright 2017
 * date: May 29, 2017
 * license: Beerware - Use this code however you'd like with attribution. If you
 * find it useful you can buy me a beer some time.
 *
 */

#ifndef _ADAFRUIT_APDS9500_H
#define _ADAFRUIT_APDS9500_H

#include "Arduino.h"
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_I2CDevice.h>
#include <Wire.h>

#define APDS9500_I2CADDR_DEFAULT 0x73 ///< APDS9500 default i2c address
#define APDS9500_CHIP_ID 0x7620 ///< APDS9500 default device id from WHOAMI

#define APDS9500_WHOAMI 0x00 ///< Chip ID register

// UNUSED:

// APDS9500_R_AE_Exposure_LB_H
// APDS9500_R_AE_Gain_Default
// APDS9500_R_AE_Gain_Step
// APDS9500_R_AE_Gain_UB
// APDS9500_R_AbortLength
// APDS9500_R_CursorClampCenterX_H
// APDS9500_R_CursorClampCenterX_L
// APDS9500_R_CursorClampCenterY_H
// APDS9500_R_CursorClampCenterY_L
// APDS9500_R_CursorClampDown
// APDS9500_R_CursorUseTop
// APDS9500_R_DiffAngleThd

// APDS9500_R_Disable45Degree
// APDS9500_R_DistThd
// APDS9500_R_Exp_Sel
// APDS9500_R_GestureDetEn
// APDS9500_R_GestureEndTh_H
// APDS9500_R_GestureEndTh_L
// APDS9500_R_GestureStartTh_H
// APDS9500_R_GestureStartTh_L
// APDS9500_R_IDLE_TIME_H
// APDS9500_R_IDLE_TIME_SLEEP_1_H
// APDS9500_R_Int1_En
// APDS9500_R_LightThd
// APDS9500_R_MCU_Int_Flag
// APDS9500_R_NoObjectCountThd
// APDS9500_R_NormalizedImageWidth
// APDS9500_R_ObjectMaxZ
// APDS9500_R_ObjectMinZ
// APDS9500_R_Object_TIME_1_H
// APDS9500_R_Object_TIME_1_L
// APDS9500_R_Offest_X
// APDS9500_R_PositionFilterStartSizeTh_H
// APDS9500_R_PositionFilterStartSizeTh_L
// APDS9500_R_PositionResolution
// APDS9500_R_ProcessFilterStartSizeTh_H
// APDS9500_R_ProcessFilterStartSizeTh_L
// APDS9500_R_ProcessResolution
// APDS9500_R_RotateZThd
// APDS9500_R_TG_INIT_TIME
// APDS9500_R_TG_POWERON_WAKEUP_TIME
// APDS9500_R_TimeDelayNum
// APDS9500_R_WaveEnH
// APDS9500_R_XDirectionThd
// APDS9500_R_XtoYGain
// APDS9500_R_YDirectionThd
// APDS9500_R_ggh
// APDS9500_R_global
// APDS9500_R_RotateConti
// APDS9500_R_RotateAngleThd

////////////////////////////////////////////////////////////////////
#define APDS9500_R_RegBankSet 0xEF // 0x00 register bank 0, 0x01 register bank 1

/* Bank 0*/
#define APDS9500_PartID_L 0x00  // Low  byte of Part ID
#define APDS9500_PartID_H 0x01  // High byte of Part ID
#define APDS9500_VersionID 0x02 // High byte of Part ID

/* Cursor Mode Controls */
// #define APDS9500_R_CursorUseTop 0x32
#define APDS9500_R_PositionFilterStartSizeTh_L 0x33
#define APDS9500_R_PositionFilterStartSizeTh_H 0x34
#define APDS9500_R_ProcessFilterStartSizeTh_L 0x35
#define APDS9500_R_ProcessFilterStartSizeTh_H 0x36
#define APDS9500_R_CursorClampLeft 0x37
#define APDS9500_R_CursorClampRight 0x38
#define APDS9500_R_CursorClampUp 0x39
// #define APDS9500_R_CursorClampDown 0x3A
// #define APDS9500_R_CursorClampCenterX_L 0x3B
// #define APDS9500_R_CursorClampCenterX_H 0x3C
// #define APDS9500_R_CursorClampCenterY_L 0x3D
// #define APDS9500_R_CursorClampCenterY_H 0x3E
#define APDS9500_R_Cursor_ObjectSizeTh 0x8B
#define APDS9500_R_PositionResolution 0x8C

/* Interrupt Controls */
#define APDS9500_R_MCU_Int_Flag 0x40
// #define APDS9500_R_Int1_En 0x41
#define APDS9500_R_Int2_En 0x42
#define APDS9500_Int_Flag_1 0x43
#define APDS9500_Int_Flag_2 0x44

/* AE/AG Controls */
#define APDS9500_R_AELedOff_UB 0x46
#define APDS9500_R_AELedOff_LB 0x47
#define APDS9500_R_AE_Exposure_UB_L 0x48
#define APDS9500_R_AE_Exposure_UB_H 0x49
#define APDS9500_R_AE_Exposure_LB_L 0x4A
// #define APDS9500_R_AE_Exposure_LB_H 0x4B
// #define APDS9500_R_AE_Gain_UB 0x4C
#define APDS9500_R_AE_Gain_LB 0x4D
// #define APDS9500_R_AE_Gain_Step 0x4E
// #define APDS9500_R_AE_Gain_Default 0x4F
// #define APDS9500_R_Exp_Sel 0x50
#define APDS9500_R_Manual 0x51
#define APDS9500_AG_Stage_GG 0x54
#define APDS9500_Reg_ExposureNum_L 0x55
#define APDS9500_Reg_ExposureNum_H 0x56
#define APDS9500_Reg_global 0x57
#define APDS9500_AE_LED_Off_YAvg 0x58
#define APDS9500_AE_Dec_Inc 0x59
#define APDS9500_AE_Normal_Factor 0x5A

/* GPIO Setting*/
#define APDS9500_InputMode_GPIO_0_1 0x80
#define APDS9500_InputMode_GPIO_2_3 0x81
#define APDS9500_InputMode_INT 0x82

/* Gesture Mode Controls */
// #define APDS9500_R_LightThd 0x83
// #define APDS9500_R_GestureStartTh_L 0x84
// #define APDS9500_R_GestureStartTh_H 0x85
// #define APDS9500_R_GestureEndTh_L 0x86
// #define APDS9500_R_GestureEndTh_H 0x87
#define APDS9500_R_ObjectMinZ 0x88
#define APDS9500_R_ObjectMaxZ 0x89
#define APDS9500_R_ProcessResolution 0x8C
#define APDS9500_R_TimeDelayNum 0x8D
// #define APDS9500_R_Disable45Degree 0x8E
#define APDS9500_R_XtoYGain 0x8F
#define APDS9500_R_NoMotionCountThd 0x90
#define APDS9500_R_NoObjectCountThd 0x91
#define APDS9500_R_NormalizedImageWidth 0x92
#define APDS9500_R_XDirectionThd 0x93
#define APDS9500_R_YDirectionThd 0x94
#define APDS9500_R_ZDirectionThd 0x95
#define APDS9500_R_ZDirectionXYThd 0x96
#define APDS9500_R_ZDirectionAngleThd 0x97
#define APDS9500_R_RotateAngleThd 0x98
#define APDS9500_R_RotateConti 0x99
#define APDS9500_R_RotateXYThd 0x9A
#define APDS9500_R_RotateZThd 0x9B
#define APDS9500_R_Filter 0x9C
// #define APDS9500_R_DistThd 0x9D
// #define APDS9500_R_GestureDetEn 0x9F
#define APDS9500_R_FilterImage 0xA5
// #define APDS9500_R_DiffAngleThd 0xA9
#define APDS9500_ObjectCenterX_L 0xAC
#define APDS9500_ObjectCenterX_H 0xAD
#define APDS9500_ObjectCenterY_L 0xAE
#define APDS9500_ObjectCenterY_H 0xAF
#define APDS9500_ObjectAvgY 0xB0
#define APDS9500_ObjectSize_L 0xB1
#define APDS9500_ObjectSize_H 0xB2
#define APDS9500_Gx 0xB3
#define APDS9500_Gy 0xB4
#define APDS9500_Gy 0xB5
#define APDS9500_GestureResult 0xB6
#define APDS9500_WaveCount 0xB7
#define APDS9500_NoObjectCount 0xB8
#define APDS9500_NoMotionCount 0xB9
#define APDS9500_LightCount 0xBA
#define APDS9500_LightAcc_L 0xBB
#define APDS9500_LightAcc_H 0xBC
#define APDS9500_TimeAcc_L 0xBD
#define APDS9500_TimeAcc_H 0xBE
#define APDS9500_AngleAcc_L 0xC7
#define APDS9500_AngleAcc_H 0xC8
#define APDS9500_XGainValue 0xCA
#define APDS9500_YGainValue 0xCB
#define APDS9500_R_YtoZSum 0xCC
#define APDS9500_R_YtoZFactor 0xCD
#define APDS9500_R_FilterLength 0xCE
#define APDS9500_R_WaveThd 0xCF
#define APDS9500_R_AbortCountThd 0xD0
// #define APDS9500_R_AbortLength 0xD1
#define APDS9500_R_WaveEnH 0xD2
#define APDS9500_PositionFilterCenterX_L 0xD3
#define APDS9500_PositionFilterCenterXY_H 0xD4
#define APDS9500_PositionFilterCenterY_L 0xD5
#define APDS9500_PositionFilterAvgY_L 0xD6
#define APDS9500_PositionFilterAvgY_H 0xD7
#define APDS9500_PositionFilterSize_L 0xD8
#define APDS9500_ProcessFilterAvgY_H 0xD9
#define APDS9500_ProcessFilterCenterX_L 0xDA
#define APDS9500_ProcessFilterCenterXY_H 0xDB
#define APDS9500_ProcessFilterCenterY_L 0xDC
#define APDS9500_ProcessFilterSize_L 0xDD
#define APDS9500_ProcessFilterAvgY_L 0xDE
#define APDS9500_AbortIntervalCount_L 0xDF

/* Bank 1 */

/* Image size settings */
#define APDS9500_Cmd_HSize 0x00
#define APDS9500_Cmd_VSize 0x01
#define APDS9500_Cmd_HStart 0x02
#define APDS9500_Cmd_VStart 0x03
#define APDS9500_Cmd_HV 0x04

/* Lens Shading */
#define APDS9500_R_LensShadingComp_EnH 0x25
#define APDS9500_R_Offest_X 0x26
#define APDS9500_R_Offest_Y 0x27
#define APDS9500_R_LSC 0x28
#define APDS9500_R_LSFT 0x29

#define APDS9500_R_global 0x42
#define APDS9500_R_ggh 0x44

/* Sleep Mode Parameters */
#define APDS9500_R_IDLE_TIME_L 0x65
// #define APDS9500_R_IDLE_TIME_H 0x66
#define APDS9500_R_IDLE_TIME_SLEEP_1_L 0x67
// #define APDS9500_R_IDLE_TIME_SLEEP_1_H 0x68
#define APDS9500_R_IDLE_TIME_SLEEP_2_L 0x69
#define APDS9500_R_IDLE_TIME_SLEEP_2_H 0x6A
#define APDS9500_R_Object_TIME_1_L 0x6B
#define APDS9500_R_Object_TIME_1_H 0x6C
#define APDS9500_R_Object_TIME_2_L 0x6D
#define APDS9500_R_Object_TIME_2_H 0x6E
#define APDS9500_R_TG_INIT_TIME 0x6F
#define APDS9500_R_TG_POWERON_WAKEUP_TIME 0x71
#define APDS9500_R_TG_EnH 0x72
#define APDS9500_R_Auto_SLEEP_Mode 0x73
#define APDS9500_R_Wake_Up_Sig_Sel 0x74

/* Image Controls */
#define APDS9500_R_SRAM_Read_EnH 0x77

/* I2C Address */
#define APDS9500_ADDRESS 0x73

/////////////////////////////////////////////////////////////////

/**
 * @brief Example enum values
 *
 * Allowed values for `setProximityLEDCurrent`.
 */
typedef enum led_current {
  APDS9500_EXAMPLE_50MA,
  APDS9500_EXAMPLE_75MA,
} APDS9500_example_t;

/*!
 *    @brief  Class that stores state and functions for interacting with
 *            the APDS9500 I2C Digital Potentiometer
 */
class Adafruit_APDS9500 {
public:
  Adafruit_APDS9500();
  bool begin(uint8_t i2c_address = APDS9500_I2CADDR_DEFAULT,
             TwoWire *wire = &Wire);

  uint16_t getDetectedGestures(void);

private:
  bool _init(void);
  bool writeByte(uint8_t address, uint8_t value);
  uint8_t buffer[2];

  Adafruit_I2CDevice *i2c_dev;
};

#endif
