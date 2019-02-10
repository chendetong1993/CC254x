/**************************************************************************************************
  Filename:       simpleBLE.c
  Revised:        $Date: 2010-08-06 08:56:11 -0700 (Fri, 06 Aug 2010) $
  Revision:       $Revision: 23333 $

  Description:    This file contains the Simple BLE Peripheral sample application
                  for use with the CC2540 Bluetooth Low Energy Protocol Stack.

  Copyright 2010 - 2013 Texas Instruments Incorporated. All rights reserved.

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
  PROVIDED �AS IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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

/*********************************************************************
 * INCLUDES
 */
#include "OSAL.h"
#include "simpleBLE.h"
#include "ll.h"
/*********************************************************************
 * MACROS
 */
/*********************************************************************
 * CONSTANTS
 */

uint8 BLE_CmdRecBuffer[BLE_CMD_Msg_Max_Len];
uint8 BLE_CmdRecBufferLen = 0;
uint8 BLE_CmdSendBuffer[BLE_CMD_Msg_Max_Len];

BLE_Type_Argv BLE_Argv;
/*********************************************************************
* TYPEDEFS
*/

/*********************************************************************
 * LOCAL FUNCTIONS
 */
void BLE_Init(BLE_Type_Argv*);
void BLE_U32ToU8Array(uint32, uint8*, uint8*);
void BLE_U8ArrayToU32(uint8*, uint8, uint32*);

void BLE_CmdGetFromUart(uint8*, uint8);
void BLE_CmdRetToUart(BLE_Type_MsgType, uint8*, uint8);

void BLE_BleRetToUart(uint8, uint8*, uint8);
bool BLE_UartRetToBLE(uint8*, uint8, uint8**, uint8*);

void BLE_UartRecTimeout();

void BLE_CmdRetTransmitDone(uint8, bool);
void BLE_CmdRetConnStatus(uint8*, uint8);
void BLE_CmdRetRoleStatus();
void BLE_CmdRetCmdInvalid();
void BLE_CmdRetParmSetd();
void BLE_CmdRetHal(uint8, uint8, uint8);

bool BLE_CmdCheckValid(uint8*, uint8);

bool BLE_CmdExtParse_8_32(uint8*, uint8, uint8*, uint32*);
bool BLE_CmdExtParse_8_N(uint8*, uint8, uint8*, uint8**, uint8*);
bool BLE_CmdExtParse_8_8_N(uint8*, uint8, uint8*, uint8*, uint8**, uint8*);
bool BLE_CmdExtParse_8_8(uint8*, uint8, uint8*, uint8*);
bool BLE_CmdExtParse_8_8_32(uint8*, uint8, uint8*, uint8*, uint32*);
bool BLE_CmdExtParse_8(uint8*, uint8, uint8*);
bool BLE_CmdExtParse(uint8*, uint8);
bool BLE_CmdParse(uint8*, uint8, uint8*, uint8*, uint8**, uint8*);

void BLE_CmdStringify(uint8*, BLE_Type_MsgType, uint8*, uint8);
bool BLE_CmdStringify_Type_N(uint8**, uint8*, BLE_Type_MsgType, uint8*, uint8);
bool BLE_CmdStringify_Type_8_32(uint8**, uint8*, BLE_Type_MsgType, uint8, uint32);
bool BLE_CmdStringify_Type_8_N(uint8**, uint8*, BLE_Type_MsgType, uint8, uint8*, uint8);
bool BLE_CmdStringify_Type_8_8_N(uint8**, uint8*, BLE_Type_MsgType, uint8, uint8, uint8*, uint8);
bool BLE_CmdStringify_Type_8_8(uint8**, uint8*, BLE_Type_MsgType, uint8, uint8);
bool BLE_CmdStringify_Type_8_8_32(uint8**, uint8*, BLE_Type_MsgType, uint8, uint8, uint32);
bool BLE_CmdStringify_Type_8(uint8**, uint8*, BLE_Type_MsgType, uint8);
bool BLE_CmdStringify_Type(uint8**Cmd , uint8* CmdLen, BLE_Type_MsgType);
/*********************************************************************
 * LOCAL Variables
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */
   
void BLE_Init(BLE_Type_Argv* Argv){
  osal_memcpy(&BLE_Argv, Argv, sizeof(BLE_Type_Argv));
}
   
/*********************************************************************
 * @fn      BLE_U32_To_U8Array
 *
 * @brief   
 *
 * @param   
 *
 * @return  none
 */
void BLE_U32ToU8Array(uint32 source, uint8* dest, uint8* destLen){
  uint8 Offset = 32, Val = 0, FindHead = false;
  do {
    Offset -= 8;
    if(((Val = (source >> Offset) & 0xFF) != 0) || (FindHead == true)){
      FindHead = true;
      dest[(*destLen)++] = Val;
    }
  } while(Offset >= 8);
  if(FindHead == false){
    dest[(*destLen)++] = 0;
  }
}

/*********************************************************************
 * @fn      BLE_U8Array_To_U32
 *
 * @brief   
 *
 * @param   
 *
 * @return  none
 */
void BLE_U8ArrayToU32(uint8* source, uint8 sourceLen, uint32* dest){
  (*dest) = 0;
  for(uint8 i = 0; i < sourceLen; i++){
    (*dest)  = ((*dest) << 8) + source[i];
  }
}

/*********************************************************************
 * @fn      BLE_CmdParse
 *
 * @brief   
 *
 * @param   ......
 *
 * @return  none
 */
bool BLE_CmdParse(uint8* RecData, uint8 RecLen, uint8* Role, uint8* Type, uint8** Ext, uint8* ExtLen){
  if(BLE_CmdCheckValid(RecData, RecLen)){
    *Role = RecData[BLE_CMD_Msg_RelDevIdx];
    *Type = RecData[BLE_CMD_Msg_TypeIdx];
    *Ext = &RecData[BLE_CMD_Msg_ExtHeadIdx];
    *ExtLen = BLE_CMD_Msg_ExtLen(RecLen);
    return true;
  } else {
    return false;
  }
}

/*********************************************************************
 * @fn      BLE_CmdGetFromUart
 *
 * @brief   Handle disconnected msg
 *
 * @param   ......
 *
 * @return  none
 */
void BLE_UartRecTimeout(){
  if(BLE_CmdRecBufferLen != 0){
     BLE_CmdRecBufferLen = 0;
     BLE_CmdRetToUart(BLE_MsgType_CMD_Invalid, 0, 0);
  }
}
     

/*********************************************************************
 * @fn      BLE_CmdGetFromUart
 *
 * @brief   
 *
 * @param   ......
 *
 * @return  none
 */
void BLE_CmdGetFromUart(uint8* RecData, uint8 RecLen){
   for(uint8 i = 0; i < RecLen; i++){
     // Data does not exceed Max Length
     if(BLE_CmdRecBufferLen < BLE_CMD_Msg_Max_Len){
      BLE_CmdRecBuffer[BLE_CmdRecBufferLen++] = RecData[i];
     }
     if(BLE_CmdRecBuffer[0] == BLE_CmdRecBufferLen){
       BLE_Argv.CmdRec(BLE_CmdRecBuffer, BLE_CmdRecBufferLen);
       BLE_CmdRecBufferLen = 0;
     }
   }
}
        
/*********************************************************************
 * @fn      BLE_Check_SerailMsgValid
 *
 * @brief   
 *
 * @param   Check whether serial data is valid
 *
 * @return  none
 */
bool BLE_CmdCheckValid(uint8* msg, uint8 Len){
  //Check Length
  if(msg[BLE_CMD_Msg_LenIdx] != Len || Len < BLE_CMD_Msg_Len(0)){
    return false;
  }

  // Check Related Device
  if(msg[BLE_CMD_Msg_RelDevIdx] != BLE_Argv.Role){
    return false;
  }

  //Check Msg Type
  if(BLE_Argv.EnCmdCheckBit){
    uint8 ExtLen = BLE_CMD_Msg_ExtLen(Len);
    //Check sum
    uint8 sum = 0;
    uint8 CheckSumEndIdx = BLE_CMD_Msg_CheckSumEndIdx(ExtLen);
    uint8 CheckSumIdx = BLE_CMD_Msg_CheckSumIdx(ExtLen);
    for(uint8 i = BLE_CMD_Msg_CheckSumHeadIdx; i <= CheckSumEndIdx; i++){
      sum += msg[i];
    }
    if(sum != msg[CheckSumIdx]){
      return false;
    }
  }
  return true;
}


/*********************************************************************
 * @fn      BLE_CmdRetToUart
 *
 * @brief   Send Result
 *
 * @param   Check whether data is valid
 *
 * @return  none
 */
void BLE_CmdStringify(uint8* Buffer, BLE_Type_MsgType MsgType, uint8* ExtData, uint8 ExtDataLen){
  //
  Buffer[BLE_CMD_Msg_LenIdx] = BLE_CMD_Msg_Len(ExtDataLen);
  Buffer[BLE_CMD_Msg_RelDevIdx] = BLE_Argv.Role;
  Buffer[BLE_CMD_Msg_TypeIdx] = MsgType;
  if(ExtData != (&Buffer[BLE_CMD_Msg_ExtHeadIdx])){
    osal_memcpy(&Buffer[BLE_CMD_Msg_ExtHeadIdx], ExtData, ExtDataLen);
  }
  //Calculate Sum
  Buffer[BLE_CMD_Msg_CheckSumIdx(ExtDataLen)] = 0;
  if(BLE_Argv.EnCmdCheckBit){
    for(uint8 i = BLE_CMD_Msg_CheckSumHeadIdx, end = BLE_CMD_Msg_CheckSumEndIdx(ExtDataLen); i <= end; i++){
      Buffer[BLE_CMD_Msg_CheckSumIdx(ExtDataLen)] += Buffer[i];
    }
  }
}

/*********************************************************************
 * @fn      BLE_CmdRetToUart
 *
 * @brief   Send Result
 *
 * @param   Check whether data is valid
 *
 * @return  none
 */
void BLE_CmdRetToUart(BLE_Type_MsgType MsgType, uint8* ExtData, uint8 ExtDataLen){
  BLE_CmdStringify(BLE_CmdSendBuffer, MsgType, ExtData, ExtDataLen);
  BLE_Argv.CmdSend(BLE_CmdSendBuffer, BLE_CmdSendBuffer[BLE_CMD_Msg_LenIdx]);
}

bool BLE_SendBLE(uint8 connHandle, uint8* TransmitData, uint8 TransmitDataLen){
  if(BLE_Argv.EnTranEncry == true) {
    if(TransmitDataLen < BLE_TRANSMIT_ENCRYPT_DATA_LEN){
      osal_memcpy(BLE_CmdSendBuffer, TransmitData, TransmitDataLen);
      BLE_CmdSendBuffer[BLE_TRANSMIT_ENCRYPT_DATA_LEN - 1] = TransmitDataLen;
      if(LL_Encrypt(BLE_Argv.TranEncryKey,  BLE_CmdSendBuffer, BLE_CmdSendBuffer) == SUCCESS){
        return BLE_Argv.BleSend(connHandle, BLE_CmdSendBuffer, BLE_TRANSMIT_ENCRYPT_DATA_LEN);
      }
    }
  } else {
    return BLE_Argv.BleSend(connHandle, TransmitData, TransmitDataLen);
  }
  return false;
}

/*********************************************************************
 * @fn      BLE_Receive_BLE
 *
 * @brief  
 *
 * @param 
 *
 * @return  none
 */
void BLE_BleRetToUart(uint8 ConnHandle, uint8* RecData, uint8 RecDataLen){
  bStatus_t status = FAILURE;
  uint8 ContentLen = RecDataLen;
  
  uint8 *Msg = &BLE_CmdSendBuffer[BLE_CMD_Msg_ExtHeadIdx];
  uint8 *MsgFrom = &Msg[0];
  uint8 *MsgContent = &Msg[1];
  (*MsgFrom) = ConnHandle;
  osal_memcpy(MsgContent, RecData, RecDataLen);

  if(BLE_Argv.EnTranEncry == true) {
    if(RecDataLen == BLE_TRANSMIT_ENCRYPT_DATA_LEN && LL_EXT_Decrypt(BLE_Argv.TranEncryKey, MsgContent, MsgContent) == SUCCESS) {
      ContentLen = MsgContent[BLE_TRANSMIT_ENCRYPT_DATA_LEN - 1];
      if(ContentLen < BLE_TRANSMIT_ENCRYPT_DATA_LEN){
        status = SUCCESS;
      }
    }
  } else {
    status = SUCCESS;
  }
  if(status == SUCCESS){
    BLE_CmdRetToUart(BLE_MsgType_Received, MsgFrom, ContentLen + 1);
  } else {
    BLE_CmdRetToUart(BLE_MsgType_UnReceived, MsgFrom, 1);
  }
}

/*********************************************************************
 * @fn      BLE_CmdRetTransmitDone
 *
 * @brief   
 *
 * @param   
 *
 * @return  none
 */
void BLE_CmdRetTransmitDone(uint8 ConnHandle, bool Success){
  BLE_CmdRetToUart(Success ? BLE_MsgType_Sended : BLE_MsgType_UnSended, &ConnHandle, 1);
}

/*********************************************************************
 * @fn      BLEC_SerialReturnConnStatus
 *
 * @brief   R
 *
 * @param   
 *
 * @return  none
 */
void BLE_CmdRetConnStatus(uint8* ConnHandles, uint8 ConnHandleNum)
{
  BLE_CmdRetToUart(BLE_MsgType_ConnStatus_Returned, ConnHandles, ConnHandleNum);
}

/*********************************************************************
 * @fn      BLE_CmdRetRoleStatus
 *
 * @brief   
 *
 * @param   
 *
 * @return  none
 */
void BLE_CmdRetRoleStatus()
{
  BLE_CmdRetToUart(BLE_MsgType_Device_Inited, BLE_Argv.Info, BLE_Argv.InfoLen);
}

/*********************************************************************
 * @fn      BLE_CmdRetCmdInvalid
 *
 * @brief   
 *
 * @param   
 *
 * @return  none
 */
void BLE_CmdRetCmdInvalid()
{
  BLE_CmdRetToUart(BLE_MsgType_CMD_Invalid, 0, 0); 
}

/*********************************************************************
 * @fn      BLE_CmdRetCmdInvalid
 *
 * @brief   
 *
 * @param   
 *
 * @return  none
 */
void BLE_CmdRetParmSetd()
{
  BLE_CmdRetToUart(BLE_MsgType_Parms_Setd, 0, 0);
}

/*********************************************************************
 * @fn      BLE_CmdRetHal
 *
 * @brief   
 *
 * @param   
 *
 * @return  none
 */
void BLE_CmdRetHal(uint8 Type, uint8 Chann, uint8 Content)
{
  uint8 Ext[] = {Type, Chann, Content };
  BLE_CmdRetToUart(BLE_MsgType_Hal_Ret, Ext, sizeof(Ext) );
}

bool BLE_CmdExtParse_8_32(uint8* Ext, uint8 ExtLen, uint8* V0, uint32* V1){
  if(ExtLen >= 2 && ExtLen <= 5) {
    *V0 = Ext[0];
    BLE_U8ArrayToU32(&Ext[1], ExtLen - 1, V1);
    return true;
  }
  return false;
}

bool BLE_CmdExtParse_8_N(uint8* Ext, uint8 ExtLen, uint8* V0, uint8** V1, uint8* V1Len){
  if(ExtLen >= 1) {
    *V0 = Ext[0];
    *V1 = &Ext[1];
    *V1Len = ExtLen - 1;
    return true;
  }
  return false;
}

bool BLE_CmdExtParse_8_8_N(uint8* Ext, uint8 ExtLen, uint8* V0, uint8* V1, uint8** V2, uint8* V2Len){
  if(ExtLen >= 2) {
    *V0 = Ext[0];
    *V1 = Ext[1];
    *V2 = &Ext[2];
    *V2Len = ExtLen - 2;
    return true;
  }
  return false;
}

bool BLE_CmdExtParse_8_8(uint8* Ext, uint8 ExtLen, uint8* V0, uint8* V1){
  if(ExtLen == 2) {
    *V0 = Ext[0];
    *V1 = Ext[1];
    return true;
  }
  return false;
}

bool BLE_CmdExtParse_8_8_32(uint8* Ext, uint8 ExtLen, uint8* V0, uint8* V1, uint32* V2){
  if(ExtLen >= 3 && ExtLen <= 6) {
    *V0 = Ext[0];
    *V1 = Ext[1];
    BLE_U8ArrayToU32(&Ext[2], ExtLen - 2, V2);
    return true;
  }
  return false;
}

bool BLE_CmdExtParse_N(uint8* Ext, uint8 ExtLen, uint8** V0, uint8* V0Len){
  *V0 = &Ext[0];
  *V0Len = ExtLen;
  return true;
  /*
  if(ExtLen >= 0) {
    *V0 = &Ext[0];
    *V0Len = ExtLen;
    return true;
  }
  return false;
  */
}
bool BLE_CmdExtParse_8(uint8* Ext, uint8 ExtLen, uint8* V0){
  if(ExtLen == 1) {
    *V0 = Ext[0];
    return true;
  }
  return false;
}

bool BLE_CmdExtParse(uint8* Ext, uint8 ExtLen){
  if(ExtLen == 0) {
    return true;
  }
  return false;
}


bool BLE_CmdStringify_Type_8_32(uint8** Cmd, uint8* CmdLen, BLE_Type_MsgType Type, uint8 U8_0, uint32 U32_0){
  if(BLE_CMD_Msg_Len(1 + 4) > BLE_CMD_Msg_Max_Len) return false;
  uint8 ExtLen = 0;
  uint8 *Ext = &BLE_CmdSendBuffer[BLE_CMD_Msg_ExtHeadIdx];
  Ext[ExtLen++] = U8_0;
  BLE_U32ToU8Array(U32_0, Ext, &ExtLen);


  BLE_CmdStringify(BLE_CmdSendBuffer, Type, Ext, ExtLen);
  *Cmd = BLE_CmdSendBuffer;
  *CmdLen = BLE_CmdSendBuffer[BLE_CMD_Msg_LenIdx];
  return true;
}

bool BLE_CmdStringify_Type_N(uint8** Cmd, uint8* CmdLen, BLE_Type_MsgType Type, uint8* UA_0, uint8 UL_0){
  if(BLE_CMD_Msg_Len(UL_0) > BLE_CMD_Msg_Max_Len) return false;
  uint8 ExtLen = 0;
  uint8 *Ext = &BLE_CmdSendBuffer[BLE_CMD_Msg_ExtHeadIdx];
  osal_memcpy(&Ext[ExtLen], UA_0, UL_0);
  ExtLen += UL_0;

  BLE_CmdStringify(BLE_CmdSendBuffer, Type, Ext, ExtLen);
  *Cmd = BLE_CmdSendBuffer;
  *CmdLen = BLE_CmdSendBuffer[BLE_CMD_Msg_LenIdx];
  return true;
}

bool BLE_CmdStringify_Type_8_N(uint8** Cmd, uint8* CmdLen, BLE_Type_MsgType Type, uint8 U8_0, uint8* UA_0, uint8 UL_0){
  if(BLE_CMD_Msg_Len(1 + UL_0) > BLE_CMD_Msg_Max_Len) return false;
  uint8 ExtLen = 0;
  uint8 *Ext = &BLE_CmdSendBuffer[BLE_CMD_Msg_ExtHeadIdx];
  Ext[ExtLen++] = U8_0;
  osal_memcpy(&Ext[ExtLen], UA_0, UL_0);
  ExtLen += UL_0;

  BLE_CmdStringify(BLE_CmdSendBuffer, Type, Ext, ExtLen);
  *Cmd = BLE_CmdSendBuffer;
  *CmdLen = BLE_CmdSendBuffer[BLE_CMD_Msg_LenIdx];
  return true;
}

bool BLE_CmdStringify_Type_8_8_N(uint8** Cmd, uint8* CmdLen, BLE_Type_MsgType Type, uint8 U8_0, uint8 U8_1, uint8* UA_0, uint8 UL_0){
  if(BLE_CMD_Msg_Len(1 + 1 + UL_0) > BLE_CMD_Msg_Max_Len) return false;
  uint8 ExtLen = 0;
  uint8 *Ext = &BLE_CmdSendBuffer[BLE_CMD_Msg_ExtHeadIdx];
  Ext[ExtLen++] = U8_0;
  Ext[ExtLen++] = U8_1;
  osal_memcpy(&Ext[ExtLen], UA_0, UL_0);
  ExtLen += UL_0;

  BLE_CmdStringify(BLE_CmdSendBuffer, Type, Ext, ExtLen);
  *Cmd = BLE_CmdSendBuffer;
  *CmdLen = BLE_CmdSendBuffer[BLE_CMD_Msg_LenIdx];
  return true;
}

bool BLE_CmdStringify_Type_8_8(uint8** Cmd, uint8* CmdLen, BLE_Type_MsgType Type, uint8 U8_0, uint8 U8_1){
  if(BLE_CMD_Msg_Len(1 + 1) > BLE_CMD_Msg_Max_Len) return false;
  uint8 ExtLen = 0;
  uint8 *Ext = &BLE_CmdSendBuffer[BLE_CMD_Msg_ExtHeadIdx];
  Ext[ExtLen++] = U8_0;
  Ext[ExtLen++] = U8_1;
  
  BLE_CmdStringify(BLE_CmdSendBuffer, Type, Ext, ExtLen);
  *Cmd = BLE_CmdSendBuffer;
  *CmdLen = BLE_CmdSendBuffer[BLE_CMD_Msg_LenIdx];
  return true;
}

bool BLE_CmdStringify_Type_8_8_32(uint8** Cmd, uint8* CmdLen, BLE_Type_MsgType Type, uint8 U8_0, uint8 U8_1, uint32 U32_){
  if(BLE_CMD_Msg_Len(1 + 1 + 4) > BLE_CMD_Msg_Max_Len) return false;
  uint8 ExtLen = 0;
  uint8 *Ext = &BLE_CmdSendBuffer[BLE_CMD_Msg_ExtHeadIdx];
  Ext[ExtLen++] = U8_0;
  Ext[ExtLen++] = U8_1;
  BLE_U32ToU8Array(U32_, Ext, &ExtLen);
  
  BLE_CmdStringify(BLE_CmdSendBuffer, Type, Ext, ExtLen);
  *Cmd = BLE_CmdSendBuffer;
  *CmdLen = BLE_CmdSendBuffer[BLE_CMD_Msg_LenIdx];
  return true;
}

bool BLE_CmdStringify_Type_8(uint8**Cmd , uint8* CmdLen, BLE_Type_MsgType Type, uint8 U8_0){
  if(BLE_CMD_Msg_Len(1) > BLE_CMD_Msg_Max_Len) return false;
  uint8 ExtLen = 0;
  uint8 *Ext = &BLE_CmdSendBuffer[BLE_CMD_Msg_ExtHeadIdx];
  Ext[ExtLen++] = U8_0;
  
  BLE_CmdStringify(BLE_CmdSendBuffer, Type, Ext, ExtLen);
  *Cmd = BLE_CmdSendBuffer;
  *CmdLen = BLE_CmdSendBuffer[BLE_CMD_Msg_LenIdx];
  return true;
}

bool BLE_CmdStringify_Type(uint8**Cmd , uint8* CmdLen, BLE_Type_MsgType Type){
  if(BLE_CMD_Msg_Len(0) > BLE_CMD_Msg_Max_Len) return false;
  BLE_CmdStringify(BLE_CmdSendBuffer, Type, 0, 0);
  *Cmd = BLE_CmdSendBuffer;
  *CmdLen = BLE_CmdSendBuffer[BLE_CMD_Msg_LenIdx];
  return true;
}
/*********************************************************************
*********************************************************************/
