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

#ifndef SIMPLEBLE_H
#define SIMPLEBLE_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */
  
//User Definition
#define BLE_CMD_Msg_Max_Len                             25
#define BLE_CMD_Msg_RelDev_C     	                0x00
#define BLE_CMD_Msg_RelDev_P    	   	        0x01
   
#define BLE_CMD_Msg_LenIdx                              0x00
#define BLE_CMD_Msg_RelDevIdx                           0x01
#define BLE_CMD_Msg_TypeIdx                             0x02
#define BLE_CMD_Msg_ExtHeadIdx                          0x03

#define BLE_CMD_Msg_Len(__ExtLen__)                     (4 + __ExtLen__)
#define BLE_CMD_Msg_ExtEndIdx(__ExtLen__)               (2 + __ExtLen__)

#define BLE_CMD_Msg_ExtLen(__MsgLen__)                  (__MsgLen__ - 4)

#define BLE_CMD_Msg_CheckSumHeadIdx                     0x00
#define BLE_CMD_Msg_CheckSumEndIdx(__ExtLen__)          (2 + __ExtLen__)
#define BLE_CMD_Msg_CheckSumIdx(__ExtLen__)             (3 + __ExtLen__)
   
#define BLE_Parm_INTER_VERSION                          0xD4000001
#define BLE_Parm_Name_Len                               12
  
#define BLEC_DEVICE_LIMIT_NUM                           3               // Number Limit of stored Connected/Scanned Devices information
  
#define BLE_ReceiveBufferTimeout                        30

#define BLE_TRANSMIT_ENCRYPT_DATA_LEN                   16


typedef struct{
  void (*SwitchRole)();
  void (*ForceWakeup)();
  void (*WakeupNotify)();
  void (*ForceSleep)();
  void (*UartReceive)(uint8*, uint8);
  void (*UartSend)(uint8*, uint8);
} BLE_Type_CommFunc;
  

typedef enum{
  BLE_PowerStatus_Sleep = 0x00,
  BLE_PowerStatus_Awake = 0x01
} BLE_Type_PowerStatus;

typedef enum{
  BLE_PairMode_Initiate = 0x00,
  BLE_PairMode_WaitForReq = 0x01,
  BLE_PairMode_Passcode_Initiate = 0x02,
  BLE_PairMode_Passcode_WaitForReq = 0x03
} BLE_Type_PairMode;

typedef enum{
  BLE_SetHal_ReadAdcStatus = 0x00,
  BLE_SetHal_SetLedStatus = 0x01,
  BLE_SetHal_ReadKeyStatus = 0x02,
  BLE_SetHal_ReadRssiStatus = 0x03,
} BLE_Type_SetHal;

typedef enum{
  BLE_HalInfoReturned_ReadAdcStatus = 0x00,
  BLE_HalInfoReturned_SetLedStatus = 0x01,
  BLE_HalInfoReturned_ReadKeyStatus = 0x02,
  BLE_HalInfoReturned_ReadRssiStatus = 0x03,
} BLE_Type_HalInfoReturned;

typedef enum {
  BLE_MsgType_Connect_EnAdvert = 0x00,
  BLE_MsgType_Disconnect_DisAdvert = 0x01,
  BLE_MsgType_RoleStatus_Get = 0x02,
  BLE_MsgType_ConnStatus_Get = 0x03,
  BLE_MsgType_Parms_Set = 0x04,
  BLE_MsgType_Send = 0x05,
  BLE_MsgType_Reboot = 0x06,
  BLE_MsgType_SwitchRole = 0x07,
  BLE_MsgType_Sleep = 0x08,
  BLE_MsgType_Hal_Set = 0x09,
  
  BLE_MsgType_Device_Inited = 0x70,
  BLE_MsgType_ConnStatus_Returned = 0x71,
  BLE_MsgType_CMD_Invalid = 0x72,
  BLE_MsgType_Parms_Setd = 0x73,
  BLE_MsgType_Sended = 0x74,
  BLE_MsgType_UnSended = 0x75,
  BLE_MsgType_Received = 0x76,
  BLE_MsgType_UnReceived = 0x77,
  BLE_MsgType_HalInfo_Returned = 0x78,
} BLE_Type_MsgType;

typedef enum{
  BLEC_SetParm_VERSION = 0x00,
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


typedef enum{
  BLEP_SetParm_VERSION = 0x00,
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
  BLEP_SetParm_PAIR_MODE = 0x0b,
  BLEP_SetParm_PASSCODE = 0x0c,
  BLEP_SetParm_AUTO_ADVERT = 0x0d,
  BLEP_SetParm_SEND_DONE_DELAY = 0x0e,
  BLEP_SetParm_ENABLE_TRANSMIT_ENCRYPT = 0x0f,
  BLEP_SetParm_TRANSMIT_ENCRYPT_KEY = 0x10,
  BLEP_SetParm_WATCHDOG = 0x11,
  BLEP_SetParm_ENABLE_CMD_CHECK_BIT = 0x12,
  BLEP_SetParm_RESET = 0xFF
} BLEP_Type_SetParm;

typedef struct {
  uint32 INTER_VERSION;
  bool ENABLE_UPDATE_REQUEST;
  uint16 UPDATE_MIN_CONN_INTERVAL;              //400   Minimum connection interval (units of 1.25ms) if automatic parameter update request is enabled
  uint16 UPDATE_MAX_CONN_INTERVAL;              //800   Maximum connection interval (units of 1.25ms) if automatic parameter update request is enabled
  uint8 UPDATE_SLAVE_LATENCY;                   //Slave latency to use if automatic parameter update request is enabled    
  uint16 UPDATE_CONN_TIMEOUT;                   //Supervision timeout value (units of 10ms) if automatic parameter update request is enabled
  int8 SCAN_RSSI_THRESHOLD;                     // Scan RSSI
  uint8 SCAN_DEV_NAME[BLEC_DEVICE_LIMIT_NUM][BLE_Parm_Name_Len];
  uint8 SCAN_DEV_NAME_LEN[BLEC_DEVICE_LIMIT_NUM];
  uint16 SCAN_DURATION;                         // Scan duration in ms
  uint16 SCAN_TOTAL_DURATION;
  uint16 MULTI_CONNECT_INTERVAL;                      // Default Connect Timer interval in ms
  uint8 PERIOD_SCAN_DEVICE_MAX_NUM;             // Max Device Scan Number in a period
  uint8 PERIOD_SCAN_DEVICE_OVERFLOW_COUNT;      // Scan overflow
  uint16 END_ALL_SCAN_CONN_REQ_TIME;
  uint8 NAME[BLE_Parm_Name_Len];
  uint8 NAME_LEN;
  uint32 VERSION;
  BLE_Type_PairMode PAIR_MODE;
  uint32 PASSCODE;
  bool ENABLE_TRANSMIT_ENCRYPT;
  uint8 TRANSMIT_ENCRYPT_KEY[BLE_TRANSMIT_ENCRYPT_DATA_LEN];
  uint8 WATCHDOG_MODE;
  bool ENABLE_CMD_CHECK_BIT;
} BLEC_Type_EParms;


typedef struct {
  uint32 INTER_VERSION;
  uint16 ADVERTISING_INTERVAL;                  // What is the advertising interval when device is discoverable (units of 625us, 160=100ms)
  bool ENABLE_DESIRED_REQUEST;                  //Enable Desired_Request
  uint16 DESIRED_MIN_CONN_INTERVAL;             //400      Minimum connection interval (units of 1.25ms) if automatic parameter update request is enabled
  uint16 DESIRED_MAX_CONN_INTERVAL;             //800       Maximum connection interval (units of 1.25ms) if automatic parameter update request is enabled
  uint8 DESIRED_SLAVE_LATENCY;
  uint16 DESIRED_CONN_TIMEOUT;
  uint8 CONN_PAUSE_PERIPHERAL;
  uint8 POWER_LEVEL;
  uint16 END_ALL_SCAN_CONN_REQ_TIME;
  uint8 NAME[BLE_Parm_Name_Len];
  uint8 NAME_LEN;
  uint32 VERSION;
  BLE_Type_PairMode PAIR_MODE;
  uint32 PASSCODE;
  bool AUTO_ADVERT;
  uint32 SEND_DONE_DELAY;
  bool ENABLE_TRANSMIT_ENCRYPT;
  uint8 TRANSMIT_ENCRYPT_KEY[BLE_TRANSMIT_ENCRYPT_DATA_LEN];
  uint8 WATCHDOG_MODE;
  bool ENABLE_CMD_CHECK_BIT;
} BLEP_Type_EParms;

typedef enum{
  BLEMode_Central,
  BLEMode_Peripheral
} BLE_Type_BLEMode;

typedef struct {
  uint32 INTER_VERSION;
  BLE_Type_BLEMode CP;
} BLE_Type_Editable_CP;

#define BLE_CP_Flash_Idx                (BLE_NVID_CUST_START)   // 0x00 ~ 0x0F Space
#define BLEC_Parms_Flash_Idx            (BLE_NVID_CUST_START + 1)
#define BLEP_Parms_Flash_Idx            (BLE_NVID_CUST_START + 2)

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task Initialization for the BLE Application
 */
  
/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SIMPLEBLECENTRAL_H */
