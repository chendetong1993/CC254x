/******************************************************************************

 @file  hal_key.h

 @brief This file contains the interface to the KEY Service.

 Group: WCS, BTS
 Target Device: CC2540, CC2541

 ******************************************************************************
 
 Copyright (c) 2005-2016, Texas Instruments Incorporated
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
 PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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

#ifndef HAL_KEY_H
#define HAL_KEY_H

#ifdef __cplusplus
extern "C"
{
#endif

/**************************************************************************************************
 *                                             INCLUDES
 **************************************************************************************************/
#include "hal_board.h"
  
/**************************************************************************************************
 * MACROS
 **************************************************************************************************/

/**************************************************************************************************
 *                                            CONSTANTS
 **************************************************************************************************/

#define HAL_KEY_DETECT_INTERVAL         10
  

#define HAL_KEY_CHANNEL_P0_0       0x00
#define HAL_KEY_CHANNEL_P0_1       0x01
#define HAL_KEY_CHANNEL_P0_2       0x02
#define HAL_KEY_CHANNEL_P0_3       0x03
#define HAL_KEY_CHANNEL_P0_4       0x04
#define HAL_KEY_CHANNEL_P0_5       0x05
#define HAL_KEY_CHANNEL_P0_6       0x06
#define HAL_KEY_CHANNEL_P0_7       0x07
#define HAL_KEY_CHANNEL_P1_0       0x08
#define HAL_KEY_CHANNEL_P1_1       0x09
#define HAL_KEY_CHANNEL_P1_2       0x0a
#define HAL_KEY_CHANNEL_P1_3       0x0b
#define HAL_KEY_CHANNEL_P1_4       0x0c
#define HAL_KEY_CHANNEL_P1_5       0x0d
#define HAL_KEY_CHANNEL_P1_6       0x0e
#define HAL_KEY_CHANNEL_P1_7       0x0f

#define HAL_KEY_CHANNEL_NUM        0x10
   
/*********************************************************************
 * TYPEDEFS
 */


/*********************************************************************
 * GLOBAL VARIABLES
 */

/*
 * Put LEDs in sleep state - store current values
 */
extern void HalKeyEnterSleep( void );

/*
 * Retore LEDs from sleep state
 */
extern void HalKeyExitSleep( void );

/*
 * Return LED state
 */
extern bool HalKeyGet( uint8 key );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif
