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
  
  

//****************************************************** method *****************************************************//
#define BLE_Array_Len(_A) (sizeof(_A) / sizeof(_A[0]))
#define BLE_Copy_Array(_D, _DL, _S, _SL) osal_memcpy(_D, _S, _SL);_DL = _SL;
#define BLE_Compare_Array(_D, _DL, _S, _SL) ((_DL == _SL) && osal_memcmp(_D, _S, _DL))
   
//****************************************************** Const *****************************************************//
//User Definition

#define BLE_CMD_Msg_Max_Len                             6 + 20
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
#define BLE_Parm_Dev_Name_Len                           20
  
#define BLE_Parm_Dev_Info_Len                           20
  
#define BLEC_Dev_Limit_Num                              3               // Number Limit of stored Connected/Scanned Devices information
  
#define BLE_ReceiveBufferTimeout                        30

#define BLE_TRANSMIT_ENCRYPT_DATA_LEN                   16

typedef struct{
  void (*SwitchRole)();
  void (*ForceWakeup)();
  void (*WakeupNotify)();
  void (*ForceSleep)();
  void (*CmdReceive)(uint8*, uint8);
  void (*CmdSend)(uint8*, uint8);
} BLE_Type_CommFunc;

typedef struct{
  uint8 Role;
  uint8* Info;
  uint8 InfoLen;
  bool EnCmdCheckBit;
  bool EnTranEncry;
  uint8* TranEncryKey;
  void (*CmdSend)(uint8*, uint8);
  void (*CmdRec)(uint8*, uint8);
  bool (*BleSend)(uint8, uint8*, uint8);
} BLE_Type_Argv;
  
typedef enum{
  BLE_PairMode_Initiate = 0x00,
  BLE_PairMode_WaitForReq = 0x01,
  BLE_PairMode_Passcode_Initiate = 0x02,
  BLE_PairMode_Passcode_WaitForReq = 0x03,
  BLE_PairMode_Num = 0x04
} BLE_Type_PairMode;

typedef enum{
  BLE_HalSet_ReadRssi = 0x00
} BLE_Type_HalSet;

typedef enum{
  BLE_HalRet_ReadRssi = 0x00,
} BLE_Type_HalRet;

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
  BLE_MsgType_Hal_Ret = 0x78,
} BLE_Type_MsgType;


typedef enum{
  BLEMode_Central,
  BLEMode_Peripheral
} BLE_Type_BLEMode;

typedef struct {
  uint32 INTER_VERSION;
  BLE_Type_BLEMode CP;
} BLE_Type_Editable_CP;

#define BLE_CP_Flash_Idx                (BLE_NVID_CUST_START)   // 0x00 ~ 0x0F Space


extern void BLE_Init(BLE_Type_Argv*);
extern void BLE_CmdGetFromUart(uint8*, uint8);
extern void BLE_BleRetToUart(uint8, uint8*, uint8);
extern bool BLE_SendBLE(uint8, uint8*, uint8);
extern void BLE_UartRecTimeout();
extern void BLE_CmdRetTransmitDone(uint8, bool);
extern void BLE_CmdRetConnStatus(uint8*, uint8);
extern void BLE_CmdRetRoleStatus();
extern void BLE_CmdRetCmdInvalid();
extern void BLE_CmdRetParmSetd();
extern void BLE_CmdRetHal(uint8, uint8, uint8);


extern bool BLE_CmdExtParse_8_32(uint8*, uint8, uint8*, uint32*);
extern bool BLE_CmdExtParse_N(uint8*, uint8, uint8**, uint8*);
extern bool BLE_CmdExtParse_8(uint8*, uint8, uint8*);
extern bool BLE_CmdExtParse_8_8_N(uint8*, uint8, uint8*, uint8*, uint8**, uint8*);
extern bool BLE_CmdExtParse_8_8(uint8*, uint8, uint8*, uint8*);
extern bool BLE_CmdExtParse_8_8_32(uint8*, uint8, uint8*, uint8*, uint32*);
extern bool BLE_CmdExtParse_8_N(uint8*, uint8, uint8*, uint8**, uint8*);
extern bool BLE_CmdExtParse(uint8*, uint8);
extern bool BLE_CmdParse(uint8*, uint8, uint8*, uint8*, uint8**, uint8*);

extern bool BLE_CmdStringify_Type_8_32(uint8**, uint8*, BLE_Type_MsgType, uint8, uint32);
extern bool BLE_CmdStringify_Type_8_N(uint8**, uint8*, BLE_Type_MsgType, uint8, uint8*, uint8);
extern bool BLE_CmdStringify_Type_N(uint8**, uint8*, BLE_Type_MsgType, uint8*, uint8);
extern bool BLE_CmdStringify_Type_8_8_N(uint8**, uint8*, BLE_Type_MsgType, uint8, uint8, uint8*, uint8);
extern bool BLE_CmdStringify_Type_8_8(uint8**, uint8*, BLE_Type_MsgType, uint8, uint8);
extern bool BLE_CmdStringify_Type_8_8_32(uint8**, uint8*, BLE_Type_MsgType, uint8, uint8, uint32);
extern bool BLE_CmdStringify_Type_8(uint8**, uint8*, BLE_Type_MsgType, uint8);
extern bool BLE_CmdStringify_Type(uint8** , uint8*, BLE_Type_MsgType);
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
