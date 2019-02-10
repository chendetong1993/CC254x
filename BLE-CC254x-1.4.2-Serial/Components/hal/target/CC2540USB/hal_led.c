/******************************************************************************

 @file  hal_led.c

 @brief This file contains the interface to the HAL LED Service.

 Group: WCS, BTS
 Target Device: CC2540, CC2541

 ******************************************************************************
 
 Copyright (c) 2006-2016, Texas Instruments Incorporated
 All rights reserved.

 IMPORTANT: Your use of this Software is limited to those specific rights
 granted under the terms of a software license agreement between the user
 who downloaded the software, his/her employer (which must be your employer)
 and Texas Instruments Incorporated (the "License"). You may not use this
 Software unless you agree to abide by the terms of the License. The License
 limits your use, and you acknowledge, that the Software may not be modified,
 copied or distributed unless embedded on a Texas Instruments microcontroller
 or used solely and exclusively in conjunction with a Texas Instruments radio
 frequency transceiver, which is integrated into your product. Other than for
 the foregoing purpose, you may not use, reproduce, copy, prepare derivative
 works of, modify, distribute, perform, display or sell this Software and/or
 its documentation for any purpose.

 YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
 PROVIDED “AS IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
 INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
 NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
 TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
 NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
 LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
 INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
 OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
 OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
 (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

 Should you have any questions regarding your right to use this Software,
 contact Texas Instruments Incorporated at www.TI.com.

 ******************************************************************************
 Release Name: ble_sdk_1.4.2.2
 Release Date: 2016-06-09 06:57:09
 *****************************************************************************/

/***************************************************************************************************
 *                                             INCLUDES
 ***************************************************************************************************/

#include "hal_types.h"
#include "hal_led.h"
#include "osal.h"

/***************************************************************************************************
 *                                              TYPEDEFS
 ***************************************************************************************************/

typedef struct {
  bool Inited;
  uint8 Port;
} HAL_TYPE_LED_CONFIG;


/***************************************************************************************************
 *                                           GLOBAL VARIABLES
 ***************************************************************************************************/

HAL_TYPE_LED_CONFIG HAL_LED_CONFIG[HAL_LED_CHANNEL_NUM]  = {
  {.Inited = false, .Port = 0x00 },
  {.Inited = false, .Port = 0x01 },
  {.Inited = false, .Port = 0x02 },
  {.Inited = false, .Port = 0x03 },
  {.Inited = false, .Port = 0x04 },
  {.Inited = false, .Port = 0x05 },
  {.Inited = false, .Port = 0x06 },
  {.Inited = false, .Port = 0x07 },
  {.Inited = false, .Port = 0x10 },
  {.Inited = false, .Port = 0x11 },
  {.Inited = false, .Port = 0x12 },
  {.Inited = false, .Port = 0x13 },
  {.Inited = false, .Port = 0x14 },
  {.Inited = false, .Port = 0x15 },
  {.Inited = false, .Port = 0x16 },
  {.Inited = false, .Port = 0x17 }
};
/***************************************************************************************************
 *                                            LOCAL FUNCTION
 ***************************************************************************************************/


/***************************************************************************************************
 *                                            FUNCTIONS - API
 ***************************************************************************************************/

/***************************************************************************************************
 * @fn      HalLedSet
 *
 * @brief   Tun ON/OFF/TOGGLE given LEDs
 *
 * @param   led - bit mask value of leds to be turned ON/OFF/TOGGLE
 *          mode - BLINK, FLASH, TOGGLE, ON, OFF
 * @return  None
 ***************************************************************************************************/
bool HalLedSet (uint8 Idx, bool Status)
{
  if(Idx >= HAL_LED_CHANNEL_NUM) return false;
  uint8 Port = HAL_LED_CONFIG[Idx].Port;
  uint8 Portx = Port >> 4;
  uint8 Portx_x = Port & 0x0F;
  uint8 s = BV(Portx_x);
  switch(Portx){
    case 0:
      if(HAL_LED_CONFIG[Idx].Inited == false) {
        P0SEL &= ~s;
        P0DIR |= s;
        P0INP &= ~s;
        HAL_LED_CONFIG[Idx].Inited = true;
      }
      Status ? (P0 |= s) : (P0 &= ~s);
      break;
    case 1:
      if(HAL_LED_CONFIG[Idx].Inited == false) {
        P1SEL &= ~s;
        P1DIR |= s;
        P1INP &= ~s;
        HAL_LED_CONFIG[Idx].Inited = true;
      }
      Status ? (P1 |= s) : (P1 &= ~s);
      break;
  }
  return true;
}



