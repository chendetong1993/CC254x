/**************************************************************************************************
  Filename:       simpleBLECentral.h
  Revised:        $Date: 2011-03-03 15:46:41 -0800 (Thu, 03 Mar 2011) $
  Revision:       $Revision: 12 $

  Description:    This file contains the Simple BLE Central sample application
                  definitions and prototypes.

  Copyright 2011 Texas Instruments Incorporated. All rights reserved.

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
#if defined ( BLE_CENTRAL )

#ifndef SIMPLEBLECENTRAL_H
#define SIMPLEBLECENTRAL_H

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
  
#define BLEC_Parms_Flash_Idx            (BLE_NVID_CUST_START + 1)
  
typedef enum{
  BLEC_SetParm_INFO = 0x00,
  BLEC_SetParm_ENABLE_UPDATE_REQUEST = 0x01,
  BLEC_SetParm_UPDATE_MIN_CONN_INTERVAL = 0x02,
  BLEC_SetParm_UPDATE_MAX_CONN_INTERVAL = 0x03,
  BLEC_SetParm_UPDATE_SLAVE_LATENCY = 0x04,
  BLEC_SetParm_UPDATE_CONN_TIMEOUT = 0x05,
  BLEC_SetParm_SCAN_RSSI_THRESHOLD = 0x06,
  BLEC_SetParm_SCAN_DEV_NAME = 0x07,
  BLEC_SetParm_SCAN_DURATION = 0x08,
  BLEC_SetParm_SCAN_TOTAL_DURATION = 0x09,
  BLEC_SetParm_PERIOD_SCAN_DEVICE_MAX_NUM = 0x0a,
  BLEC_SetParm_PERIOD_SCAN_DEVICE_OVERFLOW_COUNT = 0x0b,
  BLEC_SetParm_MULTI_CONNECT_INTERVAL = 0x0c,
  BLEC_SetParm_END_ALL_SCAN_CONN_REQ_TIME = 0x0d,
  BLEC_SetParm_NAME = 0x0e,
  BLEC_SetParm_PAIR_MODE = 0x0f,
  BLEC_SetParm_PASSCODE = 0x10,
  BLEC_SetParm_ENABLE_TRANSMIT_ENCRYPT = 0x11,
  BLEC_SetParm_TRANSMIT_ENCRYPT_KEY = 0x12,
  BLEC_SetParm_WATCHDOG = 0x13,
  BLEC_SetParm_ENABLE_CMD_CHECK_BIT = 0x14,
  BLEC_SetParm_RESET = 0xFF
} BLEC_Type_SetParm;

typedef struct {
  uint32 INTER_VERSION;
  bool ENABLE_UPDATE_REQUEST;
  uint16 UPDATE_MIN_CONN_INTERVAL;              //400   Minimum connection interval (units of 1.25ms) if automatic parameter update request is enabled
  uint16 UPDATE_MAX_CONN_INTERVAL;              //800   Maximum connection interval (units of 1.25ms) if automatic parameter update request is enabled
  uint8 UPDATE_SLAVE_LATENCY;                   //Slave latency to use if automatic parameter update request is enabled    
  uint16 UPDATE_CONN_TIMEOUT;                   //Supervision timeout value (units of 10ms) if automatic parameter update request is enabled
  int8 SCAN_RSSI_THRESHOLD;                     // Scan RSSI
  uint8 SCAN_DEV_NAME[BLEC_Dev_Limit_Num][BLE_Parm_Dev_Name_Len];
  uint8 SCAN_DEV_NAME_LEN[BLEC_Dev_Limit_Num];
  uint16 SCAN_DURATION;                         // Scan duration in ms
  uint16 SCAN_TOTAL_DURATION;
  uint16 MULTI_CONNECT_INTERVAL;                      // Default Connect Timer interval in ms
  uint8 PERIOD_SCAN_DEVICE_MAX_NUM;             // Max Device Scan Number in a period
  uint8 PERIOD_SCAN_DEVICE_OVERFLOW_COUNT;      // Scan overflow
  uint16 END_ALL_SCAN_CONN_REQ_TIME;
  uint8 NAME[BLE_Parm_Dev_Name_Len];
  uint8 NAME_LEN;
  uint8 INFO[BLE_Parm_Dev_Info_Len];
  uint8 INFO_LEN;
  BLE_Type_PairMode PAIR_MODE;
  uint32 PASSCODE;
  bool ENABLE_TRANSMIT_ENCRYPT;
  uint8 TRANSMIT_ENCRYPT_KEY[BLE_TRANSMIT_ENCRYPT_DATA_LEN];
  uint8 WATCHDOG_MODE;
  bool ENABLE_CMD_CHECK_BIT;
} BLEC_Type_EParms;

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task Initialization for the BLE Application
 */
extern void BLEC_Init( uint8 , BLE_Type_CommFunc*);

extern uint16 BLEC_ProcessEvent( uint8, uint16 );
/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SIMPLEBLECENTRAL_H */

#endif