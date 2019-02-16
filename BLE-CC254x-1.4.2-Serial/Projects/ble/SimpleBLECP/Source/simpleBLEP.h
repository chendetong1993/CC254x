/**************************************************************************************************
  Filename:       simpleBLEperipheral.h
  Revised:        $Date: 2010-08-01 14:03:16 -0700 (Sun, 01 Aug 2010) $
  Revision:       $Revision: 23256 $

  Description:    This file contains the Simple BLE Peripheral sample application
                  definitions and prototypes.

  Copyright 2010 - 2011 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
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
**************************************************************************************************/
#if defined ( BLE_PERIPHERAL )

#ifndef SIMPLEBLEPERIPHERAL_H
#define SIMPLEBLEPERIPHERAL_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "simpleBLE.h"
  
/*********************************************************************
 * CONSTANTS
 */

#define BLEP_Parms_Flash_Idx            (BLE_NVID_CUST_START + 2)
   
typedef enum{
  BLEP_SetParm_INFO = 0x00,
  BLEP_SetParm_ADVERTISING_INTERVAL = 0x01,
  BLEP_SetParm_ENABLE_DESIRED_REQUEST = 0x02,
  BLEP_SetParm_DESIRED_MIN_CONN_INTERVAL = 0x03,
  BLEP_SetParm_DESIRED_MAX_CONN_INTERVAL = 0x04,
  BLEP_SetParm_DESIRED_SLAVE_LATENCY = 0x05,
  BLEP_SetParm_DESIRED_CONN_TIMEOUT = 0x06,
  BLEP_SetParm_CONN_PAUSE_PERIPHERAL = 0x07,
  BLEP_SetParm_POWER_LEVEL = 0x08,
  BLEP_SetParm_END_ALL_SCAN_CONN_REQ_TIME = 0x09,
  BLEP_SetParm_NAME = 0x0a,
  BLEP_SetParm_SCAN_RSP_DATA = 0x0b,
  BLEP_SetParm_PAIR_MODE = 0x0c,
  BLEP_SetParm_PASSCODE = 0x0d,
  BLEP_SetParm_AUTO_ADVERT = 0x0e,
  BLEP_SetParm_SEND_DONE_DELAY = 0x0f,
  BLEP_SetParm_ENABLE_TRANSMIT_ENCRYPT = 0x10,
  BLEP_SetParm_TRANSMIT_ENCRYPT_KEY = 0x11,
  BLEP_SetParm_WATCHDOG = 0x12,
  BLEP_SetParm_ENABLE_CMD_CHECK_BIT = 0x13,
  BLEP_SetParm_RESET = 0xFF
} BLEP_Type_SetParm;


typedef struct {
  uint32 INTER_VERSION;
  uint16 ADVERTISING_INTERVAL;                  // What is the advertising interval when device is discoverable (units of 625us, 160=100ms)
  bool ENABLE_DESIRED_REQUEST;                  //Enable Desired_Request
  uint16 DESIRED_MIN_CONN_INTERVAL;             //400      Minimum connection interval (units of 1.25ms) if automatic parameter update request is enabled
  uint16 DESIRED_MAX_CONN_INTERVAL;             //800       Maximum connection interval (units of 1.25ms) if automatic parameter update request is enabled
  uint16 DESIRED_SLAVE_LATENCY;
  uint16 DESIRED_CONN_TIMEOUT;
  uint8 CONN_PAUSE_PERIPHERAL;
  uint8 POWER_LEVEL;
  uint16 END_ALL_SCAN_CONN_REQ_TIME;
  uint8 NAME[BLE_Parm_Dev_Name_Max_Len];
  uint8 NAME_LEN;
  uint8 INFO[BLE_Parm_Dev_Info_Max_Len];
  uint8 INFO_LEN;
  uint8 SCAN_RSP_DATA[BLE_SCAN_RSP_DATA_Max_Len];
  uint8 SCAN_RSP_DATA_LEN;
  BLE_Type_PairMode PAIR_MODE;
  uint32 PASSCODE;
  bool AUTO_ADVERT;
  uint16 SEND_DONE_DELAY;
  bool ENABLE_TRANSMIT_ENCRYPT;
  uint8 TRANSMIT_ENCRYPT_KEY[BLE_TRANSMIT_ENCRYPT_DATA_LEN];
  uint8 WATCHDOG_MODE;
  bool ENABLE_CMD_CHECK_BIT;
} BLEP_Type_EParms;

/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task Initialization for the BLE Application
 */
extern void BLEP_Init( uint8 , BLE_Type_CommFunc*);

/*
 * Task Event Processor for the BLE Application
 */
extern uint16 BLEP_ProcessEvent( uint8, uint16 );

extern void BLEP_RECEIVE_UART( uint8* , uint8 );
/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SIMPLEBLEPERIPHERAL_H */

#endif