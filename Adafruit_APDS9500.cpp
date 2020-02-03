
/*!
 *  @file Adafruit_APDS9500.cpp
 *
 *  @mainpage Adafruit APDS9500 Gesture Sensor library
 *
 *  @section intro_sec Introduction
 *
 * 	I2C Driver for the Adafruit APDS9500 Gesture Sensor library
 *
 * 	This is a library for the Adafruit APDS9500 breakout:
 * 	https://www.adafruit.com/product/889X
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *
    > Derived from:
    > APDS9500_gestures.ino
    > by: Kris Winer, Tlera Corporation, Copyright 2017
    > date: May 29, 2017
    > license: Beerware - Use this code however you'd like with attribution. If
 you > find it useful you can buy me a beer some time.
 *
 *
 *
 *  @section dependencies Dependencies
 *  This library depends on the Adafruit BusIO library
 *
 *
 *  @section author Author
 *
 *  Bryan Siepert for Adafruit Industries
 *  Kris Winer, Tlera Corporation, Copyright 2017
 * 	@section license License
 *
 * 	BSD (see license.txt)
 *
 * 	@section  HISTORY
 *
 *     v1.0 - First release
 */

#include "Arduino.h"
#include <Wire.h>

#include "Adafruit_APDS9500.h"

/*!
 *    @brief  Instantiates a new APDS9500 class
 */
Adafruit_APDS9500::Adafruit_APDS9500(void) {}

/*!
 *    @brief  Sets up the hardware and initializes I2C
 *    @param  i2c_address
 *            The I2C address to be used.
 *    @param  wire
 *            The Wire object to be used for I2C connections.
 *    @return True if initialization was successful, otherwise false.
 */
bool Adafruit_APDS9500::begin(uint8_t i2c_address, TwoWire *wire) {
  i2c_dev = new Adafruit_I2CDevice(i2c_address, wire);

  if (!i2c_dev->begin()) {
    return false;
  }
  return _init();
}

bool Adafruit_APDS9500::_init(void) {

  // uint8_t buffer[2];

  // buffer[0] = APDS9500_R_RegBankSet;
  // buffer[1] = 0;
  // if(!i2c_dev->write(buffer, 2)){
  //   Serial.println("Issue writing wake up buffer");
  //   return false;
  // }
  if (!writeByte(APDS9500_R_RegBankSet, 0)){
    return false;
  }

  buffer[0] = APDS9500_PartID_L;
  buffer[1] = 0;
  if(!i2c_dev->write_then_read(buffer, 1, buffer, 2)){
    return false;
  }

  if ((buffer[1]<<8 | buffer[0]) != APDS9500_CHIP_ID){
    return false;
  }

  // writeByte(APDS9500_ADDRESS, APDS9500_R_RegBankSet, 0x00);         // select
  // bank 0

  // // Define cursor limits
  // writeByte(APDS9500_ADDRESS, APDS9500_R_CursorClampLeft, 0x07);    // min
  // horiz value writeByte(APDS9500_ADDRESS, APDS9500_R_CursorClampRight,0x17);
  // // max horiz value writeByte(APDS9500_ADDRESS,
  // APDS9500_R_CursorClampUp,0x06);       // min vert value
  // writeByte(APDS9500_ADDRESS, APDS9500_R_Int2_En,0x01);             // enable
  // interrupt on proximity
  // // Auto exposure/Auto gain Controls
  // writeByte(APDS9500_ADDRESS, APDS9500_R_AELedOff_UB, 0x2D);        //
  // exposure time upper bound writeByte(APDS9500_ADDRESS,
  // APDS9500_R_AELedOff_LB, 0x0F);        // exposure time lower bound
  // writeByte(APDS9500_ADDRESS, APDS9500_R_AE_Exposure_UB_L, 0x3C);   // low
  // byte auto exposure upper bound writeByte(APDS9500_ADDRESS,
  // APDS9500_R_AE_Exposure_UB_H, 0x00);   // high byte auto exposure upper
  // bound writeByte(APDS9500_ADDRESS, APDS9500_R_AE_Exposure_LB_L, 0x1E);   //
  // low byte auto exposure lower bound writeByte(APDS9500_ADDRESS,
  // APDS9500_R_AE_Gain_LB, 0x20);         // auto gain upper bound
  // writeByte(APDS9500_ADDRESS, APDS9500_R_Manual, 0x10);             // enable
  // auto exposure writeByte(APDS9500_ADDRESS, 0x5E, 0x10); // don't know
  // writeByte(APDS9500_ADDRESS, 0x60, 0x27);                          // don't
  // know
  // // Set up Interrupt
  // writeByte(APDS9500_ADDRESS, APDS9500_InputMode_GPIO_0_1, 0x42);   // set
  // GPIO0 as OUTPUT, GPIO1 as INPUT writeByte(APDS9500_ADDRESS,
  // APDS9500_InputMode_GPIO_2_3, 0x44);   // set GPIO2 as INPUT, GPIO3 as INPUT
  // writeByte(APDS9500_ADDRESS, APDS9500_InputMode_INT, 0x04);        // set
  // INT as INPUT
  // // Detection thresholds
  // writeByte(APDS9500_ADDRESS, APDS9500_R_Cursor_ObjectSizeTh, 0x01); //
  // object size threshold for cursor mode writeByte(APDS9500_ADDRESS,
  // APDS9500_R_NoMotionCountThd, 0x06);    // no motion counter threshold
  // writeByte(APDS9500_ADDRESS, APDS9500_R_ZDirectionThd, 0x0A);       //
  // gesture detection z threshold writeByte(APDS9500_ADDRESS,
  // APDS9500_R_ZDirectionXYThd, 0x0C);     // gesture detection x and y
  // thresholds writeByte(APDS9500_ADDRESS, APDS9500_R_ZDirectionAngleThd,
  // 0x05);  // angle threshold for forward and backward detection
  // writeByte(APDS9500_ADDRESS, APDS9500_R_RotateXYThd, 0x14);         //
  // rotation detection threshold writeByte(APDS9500_ADDRESS, APDS9500_R_Filter,
  // 0x3F);              // filter weight and frame position threshold
  // writeByte(APDS9500_ADDRESS, APDS9500_R_FilterImage, 0x19);         // use
  // pixel brightness for weak average filter writeByte(APDS9500_ADDRESS,
  // APDS9500_R_YtoZSum, 0x19);             // z-direction mapping parameter
  // writeByte(APDS9500_ADDRESS, APDS9500_R_YtoZFactor, 0x0B);          //
  // z-direction mapping parameter writeByte(APDS9500_ADDRESS,
  // APDS9500_R_FilterLength, 0x03);        // filter length for cursor object
  // center writeByte(APDS9500_ADDRESS, APDS9500_R_WaveThd, 0x64); // wave
  // gesture counter and angle thresholds writeByte(APDS9500_ADDRESS,
  // APDS9500_R_AbortCountThd, 0x21);       // abort gesture counter threshold

  // // Change to Bank 1
  // writeByte(APDS9500_ADDRESS, APDS9500_R_RegBankSet, 0x01);          //
  // select bank 1

  // // Image size settings
  // writeByte(APDS9500_ADDRESS, APDS9500_Cmd_HStart, 0x0F);            //
  // horizontal starting point writeByte(APDS9500_ADDRESS, APDS9500_Cmd_VStart,
  // 0x10);            // vertical starting point writeByte(APDS9500_ADDRESS,
  // APDS9500_Cmd_HV, 0x02);                // vertical flip
  // writeByte(APDS9500_ADDRESS, APDS9500_R_LensShadingComp_EnH, 0x01); //
  // enable lens shading compensation writeByte(APDS9500_ADDRESS,
  // APDS9500_R_Offest_Y, 0x39);            // vertical offset of lens, set to
  // 55 writeByte(APDS9500_ADDRESS, APDS9500_R_LSC, 0x7F);                 //
  // Lens shading coefficient, set to 127 writeByte(APDS9500_ADDRESS,
  // APDS9500_R_LSFT, 0x08);                // shift amount, initialize to 10
  // writeByte(APDS9500_ADDRESS, 0x3E,0xFF);                            // don't
  // know writeByte(APDS9500_ADDRESS, 0x5E,0x3D);                            //
  // don't know
  // /* Sleep mode parameters */
  // writeByte(APDS9500_ADDRESS, APDS9500_R_IDLE_TIME_L, 0x96);         // idle
  // time low byte = 150 which is set for ~120 fps writeByte(APDS9500_ADDRESS,
  // APDS9500_R_IDLE_TIME_SLEEP_1_L, 0x97); // idle time for weak sleep, set for
  // report rate ~ 120 Hz writeByte(APDS9500_ADDRESS,
  // APDS9500_R_IDLE_TIME_SLEEP_2_L, 0xCD); // idle time for deep sleep, low
  // byte writeByte(APDS9500_ADDRESS, APDS9500_R_IDLE_TIME_SLEEP_2_H, 0x01); //
  // idle time for deep sleep, high byte writeByte(APDS9500_ADDRESS,
  // APDS9500_R_Object_TIME_2_L, 0x2C);     // deep sleep enter time, low byte
  // writeByte(APDS9500_ADDRESS, APDS9500_R_Object_TIME_2_H, 0x01);     // deep
  // sleep enter time, high byte writeByte(APDS9500_ADDRESS, APDS9500_R_TG_EnH,
  // 0x01);              // enable time gating writeByte(APDS9500_ADDRESS,
  // APDS9500_R_Auto_SLEEP_Mode, 0x35);     // no object weak and deep sleep,
  // object wake writeByte(APDS9500_ADDRESS, APDS9500_R_Wake_Up_Sig_Sel, 0x00);
  // // interrupt on time gate start
  // /* Start sensor */
  // writeByte(APDS9500_ADDRESS, APDS9500_R_SRAM_Read_EnH, 0x01);       //SRAM
  // read enable

  // // Change back to bank 0 for data read
  // writeByte(APDS9500_ADDRESS, APDS9500_R_RegBankSet, 0x00);         // select
  // bank 0

  // do any software reset or other initial setup
  return true;
}

/*********** typdef enum getter with bitfield *********************/

/**************************************************************************/
/*!
    @brief Retrieves the gestures that were detected.
    @returns The EXAMPLE VALUE.
*/
uint16_t Adafruit_APDS9500::getDetectedGestures(void) {

  Adafruit_BusIO_Register gesture_flags1 =
      // Adafruit_I2CDevice pointer, address, number of bytes
      Adafruit_BusIO_Register(i2c_dev, APDS9500_Int_Flag_1);

  Adafruit_BusIO_Register gesture_flags2 =
      // Adafruit_I2CDevice pointer, address, number of bytes
      Adafruit_BusIO_Register(i2c_dev, APDS9500_Int_Flag_2);

  uint8_t intFlag1 = gesture_flags1.read();
  uint8_t intFlag2 = gesture_flags2.read();
  uint16_t gestResult = (intFlag2 << 8 | intFlag1);
}

bool Adafruit_APDS9500::writeByte(uint8_t address, uint8_t value){
  buffer[0] = address;
  buffer[1] = value;
  return i2c_dev->write(buffer, 2);
}
