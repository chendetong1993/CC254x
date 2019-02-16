/**************************************************************************************************
  Filename:       simpleBLE.c
  Revised:        $Date: 2010-08-06 08:56:11 -0700 (Fri, 06 Aug 2010) $
  Revision:       $Revision: 23333 $

  Description:    
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

uint8 BLE_CmdSendBuf[BLE_CMD_Msg_Max_Len];
uint8 BLE_QueueDeBuf[BLE_CMD_Msg_Max_Len];

BLE_Type_Argv BLE_Argv;
/*********************************************************************
* TYPEDEFS
*/

/*********************************************************************
 * LOCAL FUNCTIONS
 */
void BLE_Init(BLE_Type_Argv*);

bool BLE_QueueEn(BLE_Type_Queue*, uint8*, uint8);
bool BLE_QueueDe(BLE_Type_Queue*, uint8**, uint8*);
void BLE_U32ToU8Array(uint32, uint8*, uint8*);
void BLE_U8ArrayToU32(uint8*, uint8, uint32*);

void BLE_CmdGetFromExter(uint8*, uint8);
void BLE_CmdRetToExter(BLE_Type_MsgType, uint8*, uint8);

void BLE_BleRetToExter(uint8, uint8*, uint8);
bool BLE_UartRetToBLE(uint8*, uint8, uint8**, uint8*);
void BLE_CmdRetTransmitDone(uint8, uint8);
void BLE_CmdRetConnStatus(uint8*, uint8);
void BLE_CmdRetRoleStatus();
uint8 BLE_CmdRetError(BLE_Type_Error, uint8);
void BLE_CmdRetCmdParsed();
void BLE_CmdRetAddInfo(uint8, uint8, uint8*, uint8);
bool BLE_CmdCheckValid(uint8*, uint8);

bool BLE_CmdExtParse_8_32(uint8*, uint8, uint8*, uint32*, bool);
bool BLE_CmdExtParse_8_N(uint8*, uint8, uint8*, uint8**, uint8*, bool);
bool BLE_CmdExtParse_8_8_N(uint8*, uint8, uint8*, uint8*, uint8**, uint8*, bool);
bool BLE_CmdExtParse_8_8(uint8*, uint8, uint8*, uint8*, bool);
bool BLE_CmdExtParse_8_8_32(uint8*, uint8, uint8*, uint8*, uint32*, bool);
bool BLE_CmdExtParse_8(uint8*, uint8, uint8*, bool);
bool BLE_CmdExtParse(uint8*, uint8, bool);
bool BLE_CmdParse(uint8*, uint8, uint8*, uint8*, uint8**, uint8*, bool);

void BLE_CmdStringify(uint8*, BLE_Type_MsgType, uint8*, uint8);
bool BLE_CmdStringify_Type_N(uint8**, uint8*, BLE_Type_MsgType, uint8*, uint8);
bool BLE_CmdStringify_Type_8_32(uint8**, uint8*, BLE_Type_MsgType, uint8, uint32);
bool BLE_CmdStringify_Type_8_N(uint8**, uint8*, BLE_Type_MsgType, uint8, uint8*, uint8);
bool BLE_CmdStringify_Type_8_8_N(uint8**, uint8*, BLE_Type_MsgType, uint8, uint8, uint8*, uint8);
bool BLE_CmdStringify_Type_8_8(uint8**, uint8*, BLE_Type_MsgType, uint8, uint8);
bool BLE_CmdStringify_Type_8_8_32(uint8**, uint8*, BLE_Type_MsgType, uint8, uint8, uint32);
bool BLE_CmdStringify_Type_8(uint8**, uint8*, BLE_Type_MsgType, uint8);
bool BLE_CmdStringify_Type(uint8**Cmd , uint8* CmdLen, BLE_Type_MsgType);
void BLE_ScanAdvertData_Construct(BLE_Type_ScanAdvertData*, uint8 DataLen, uint8*, uint8*);
/*********************************************************************
 * LOCAL Variables
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */
   
void BLE_Init(BLE_Type_Argv* Argv){
  osal_memcpy(&BLE_Argv, Argv, sizeof(BLE_Type_Argv));
}
   
bool BLE_QueueEn(BLE_Type_Queue* Queue, uint8* Data, uint8 Len){
  if((Queue->Len + (Len + 1)) <= BLE_Queue_Max_Len){
    uint8 Idx = Queue->Idx + Queue->Len;
    if(Idx >= BLE_Queue_Max_Len) Idx -= BLE_Queue_Max_Len;
    Queue->Queue[Idx++] = Len;
    for(uint8 i = 0; i < Len; i++){
      if(Idx >= BLE_Queue_Max_Len) Idx -= BLE_Queue_Max_Len;
      Queue->Queue[Idx++] = Data[i];
    }
    Queue->Len += (Len + 1);
    return true;
  } else {
    return false;
  }
}

bool BLE_QueueDe(BLE_Type_Queue* Queue, uint8** Data, uint8* DataLen){
  if(Queue->Len != 0){
    *Data = BLE_QueueDeBuf;
    *DataLen = Queue->Queue[Queue->Idx];
    uint8 Idx = Queue->Idx + 1;
    uint8 Len = *DataLen;
    for(uint8 i = 0; i < Len; i++){
      if(Idx >= BLE_Queue_Max_Len) Idx -= BLE_Queue_Max_Len;
      (*Data)[i] = Queue->Queue[Idx++];
    }
    Queue->Len -= (Len + 1);
    Queue->Idx += (Len + 1);
    if(Queue->Idx >= BLE_Queue_Max_Len) Queue->Idx -= BLE_Queue_Max_Len;
    return true;
  } else {
    return false;
  }
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
bool BLE_CmdParse(uint8* RecData, uint8 RecLen, uint8* Role, uint8* Type, uint8** Ext, uint8* ExtLen, bool CheckValid){
  if((CheckValid == false) || BLE_CmdCheckValid(RecData, RecLen)){
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
 * @fn      BLE_CmdGetFromExter
 *
 * @brief   
 *
 * @param   ......
 *
 * @return  none
 */
void BLE_CmdGetFromExter(uint8* RecData, uint8 RecLen){
  BLE_Argv.CmdRec(RecData, RecLen);
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
  if(Len < BLE_CMD_Msg_Len(0) || msg[BLE_CMD_Msg_LenIdx] != Len){
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
 * @fn      BLE_CmdRetToExter
 *
 * @brief   Send Result
 *
 * @param   Check whether data is valid
 *
 * @return  none
 */
void BLE_CmdStringify(uint8* Buf, BLE_Type_MsgType MsgType, uint8* ExtData, uint8 ExtDataLen){
  //
  Buf[BLE_CMD_Msg_LenIdx] = BLE_CMD_Msg_Len(ExtDataLen);
  Buf[BLE_CMD_Msg_RelDevIdx] = BLE_Argv.Role;
  Buf[BLE_CMD_Msg_TypeIdx] = MsgType;
  if(ExtData != (&Buf[BLE_CMD_Msg_ExtHeadIdx])){
    osal_memcpy(&Buf[BLE_CMD_Msg_ExtHeadIdx], ExtData, ExtDataLen);
  }
  //Calculate Sum
  Buf[BLE_CMD_Msg_CheckSumIdx(ExtDataLen)] = 0;
  if(BLE_Argv.EnCmdCheckBit){
    for(uint8 i = BLE_CMD_Msg_CheckSumHeadIdx, end = BLE_CMD_Msg_CheckSumEndIdx(ExtDataLen); i <= end; i++){
      Buf[BLE_CMD_Msg_CheckSumIdx(ExtDataLen)] += Buf[i];
    }
  }
}

/*********************************************************************
 * @fn      BLE_CmdRetToExter
 *
 * @brief   Send Result
 *
 * @param   Check whether data is valid
 *
 * @return  none
 */
void BLE_CmdRetToExter(BLE_Type_MsgType MsgType, uint8* ExtData, uint8 ExtDataLen){
  BLE_CmdStringify(BLE_CmdSendBuf, MsgType, ExtData, ExtDataLen);
  BLE_Argv.CmdSend(BLE_CmdSendBuf, BLE_CmdSendBuf[BLE_CMD_Msg_LenIdx]);
}

bool BLE_SendBLE(uint8 connHandle, uint8* TransmitData, uint8 TransmitDataLen){
  if(BLE_Argv.EnTranEncry == true) {
    if(TransmitDataLen < BLE_TRANSMIT_ENCRYPT_DATA_LEN){
      osal_memcpy(BLE_CmdSendBuf, TransmitData, TransmitDataLen);
      BLE_CmdSendBuf[BLE_TRANSMIT_ENCRYPT_DATA_LEN - 1] = TransmitDataLen;
      if(LL_Encrypt(BLE_Argv.TranEncryKey,  BLE_CmdSendBuf, BLE_CmdSendBuf) == SUCCESS){
        return BLE_Argv.BleSend(connHandle, BLE_CmdSendBuf, BLE_TRANSMIT_ENCRYPT_DATA_LEN);
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
void BLE_BleRetToExter(uint8 ConnHandle, uint8* RecData, uint8 RecDataLen){
  bStatus_t status = FAILURE;
  uint8 ContentLen = RecDataLen;
  
  uint8 *Msg = &BLE_CmdSendBuf[BLE_CMD_Msg_ExtHeadIdx];
  uint8 *MsgFrom = &Msg[0];
  uint8 *MsgContent = &Msg[1];
  (*MsgFrom) = ConnHandle;
  osal_memcpy(MsgContent, RecData, RecDataLen);

  if(BLE_Argv.EnTranEncry == true) {
    if((BLE_CmdRetError(BLE_Error_Hci, (RecDataLen == BLE_TRANSMIT_ENCRYPT_DATA_LEN) ? SUCCESS : LL_STATUS_ERROR_LL_TIMEOUT) == SUCCESS) && 
       (BLE_CmdRetError(BLE_Error_Hci, LL_EXT_Decrypt(BLE_Argv.TranEncryKey, MsgContent, MsgContent)) == SUCCESS)) {
      ContentLen = MsgContent[BLE_TRANSMIT_ENCRYPT_DATA_LEN - 1];
      if(ContentLen < BLE_TRANSMIT_ENCRYPT_DATA_LEN){
        status = SUCCESS;
      }
    }
  } else {
    status = SUCCESS;
  }
  if(status == SUCCESS){
    BLE_CmdRetToExter(BLE_MsgType_Received, MsgFrom, ContentLen + 1);
  } else {
    BLE_CmdRetToExter(BLE_MsgType_UnReceived, MsgFrom, 1);
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
  if(Success){
    BLE_CmdRetToExter(BLE_MsgType_Sended, &ConnHandle, 1);
  } else {
    BLE_CmdRetToExter(BLE_MsgType_UnSended, &ConnHandle, 1);
  }
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
  BLE_CmdRetToExter(BLE_MsgType_ConnStatus_Returned, ConnHandles, ConnHandleNum);
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
  BLE_CmdRetToExter(BLE_MsgType_Device_Inited, BLE_Argv.Info, BLE_Argv.InfoLen);
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
uint8 BLE_CmdRetError(BLE_Type_Error Type, uint8 ErrorCode){
  if(ErrorCode != SUCCESS){
    uint8 Ext[] = {Type, ErrorCode};
    BLE_CmdRetToExter(BLE_MsgType_Error_Happened, Ext, sizeof(Ext)); 
  }
  return ErrorCode;
}

/*********************************************************************
 * @fn      BLE_CmdRetCmdParsed
 *
 * @brief   
 *
 * @param   
 *
 * @return  none
 */
void BLE_CmdRetCmdParsed(){
  BLE_CmdRetToExter(BLE_MsgType_Cmd_Parsed, 0, 0);
}

/*********************************************************************
 * @fn      BLE_CmdRetAddInfo
 *
 * @brief   
 *
 * @param   
 *
 * @return  none
 */
void BLE_CmdRetAddInfo(uint8 Type, uint8 Chann, uint8* Content, uint8 ContentLen){
  uint8 Len = BLE_CMD_Msg_ExtHeadIdx;
  BLE_Array_Append(BLE_CmdSendBuf, Len, &Type, 1);
  BLE_Array_Append(BLE_CmdSendBuf, Len, &Chann, 1);
  BLE_Array_Append(BLE_CmdSendBuf, Len, Content, ContentLen);
  BLE_CmdRetToExter(BLE_MsgType_AdditInfo_Ret, &BLE_CmdSendBuf[BLE_CMD_Msg_ExtHeadIdx], Len - BLE_CMD_Msg_ExtHeadIdx);
}

bool BLE_CmdExtParse_8_32(uint8* Ext, uint8 ExtLen, uint8* V0, uint32* V1, bool CheckValid){
  if(CheckValid == false || (ExtLen >= 2 && ExtLen <= 5)) {
    *V0 = Ext[0];
    BLE_U8ArrayToU32(&Ext[1], ExtLen - 1, V1);
    return true;
  }
  return false;
}

bool BLE_CmdExtParse_8_N(uint8* Ext, uint8 ExtLen, uint8* V0, uint8** V1, uint8* V1Len, bool CheckValid){
  if(CheckValid == false || ExtLen >= 1) {
    *V0 = Ext[0];
    *V1 = &Ext[1];
    *V1Len = ExtLen - 1;
    return true;
  }
  return false;
}

bool BLE_CmdExtParse_8_8_N(uint8* Ext, uint8 ExtLen, uint8* V0, uint8* V1, uint8** V2, uint8* V2Len, bool CheckValid){
  if(CheckValid == false || ExtLen >= 2) {
    *V0 = Ext[0];
    *V1 = Ext[1];
    *V2 = &Ext[2];
    *V2Len = ExtLen - 2;
    return true;
  }
  return false;
}

bool BLE_CmdExtParse_8_8(uint8* Ext, uint8 ExtLen, uint8* V0, uint8* V1, bool CheckValid){
  if(CheckValid == false || ExtLen == 2) {
    *V0 = Ext[0];
    *V1 = Ext[1];
    return true;
  }
  return false;
}

bool BLE_CmdExtParse_8_8_32(uint8* Ext, uint8 ExtLen, uint8* V0, uint8* V1, uint32* V2, bool CheckValid){
  if(CheckValid == false || (ExtLen >= 3 && ExtLen <= 6)) {
    *V0 = Ext[0];
    *V1 = Ext[1];
    BLE_U8ArrayToU32(&Ext[2], ExtLen - 2, V2);
    return true;
  }
  return false;
}

bool BLE_CmdExtParse_N(uint8* Ext, uint8 ExtLen, uint8** V0, uint8* V0Len, bool CheckValid){
  if(CheckValid == false || true) {
    *V0 = &Ext[0];
    *V0Len = ExtLen;
    return true;
  }
  return false;
  /*
  if(ExtLen >= 0) {
    *V0 = &Ext[0];
    *V0Len = ExtLen;
    return true;
  }
  return false;
  */
}
bool BLE_CmdExtParse_8(uint8* Ext, uint8 ExtLen, uint8* V0, bool CheckValid){
  if(CheckValid == false || ExtLen == 1) {
    *V0 = Ext[0];
    return true;
  }
  return false;
}

bool BLE_CmdExtParse(uint8* Ext, uint8 ExtLen, bool CheckValid){
  if(CheckValid == false || ExtLen == 0) {
    return true;
  }
  return false;
}


bool BLE_CmdStringify_Type_8_32(uint8** Cmd, uint8* CmdLen, BLE_Type_MsgType Type, uint8 U8_0, uint32 U32_0){
  if(BLE_CMD_Msg_Len(1 + 4) > BLE_CMD_Msg_Max_Len) return false;
  uint8 ExtLen = 0;
  uint8 *Ext = &BLE_CmdSendBuf[BLE_CMD_Msg_ExtHeadIdx];
  Ext[ExtLen++] = U8_0;
  BLE_U32ToU8Array(U32_0, Ext, &ExtLen);


  BLE_CmdStringify(BLE_CmdSendBuf, Type, Ext, ExtLen);
  *Cmd = BLE_CmdSendBuf;
  *CmdLen = BLE_CmdSendBuf[BLE_CMD_Msg_LenIdx];
  return true;
}

bool BLE_CmdStringify_Type_N(uint8** Cmd, uint8* CmdLen, BLE_Type_MsgType Type, uint8* UA_0, uint8 UL_0){
  if(BLE_CMD_Msg_Len(UL_0) > BLE_CMD_Msg_Max_Len) return false;
  uint8 ExtLen = 0;
  uint8 *Ext = &BLE_CmdSendBuf[BLE_CMD_Msg_ExtHeadIdx];
  osal_memcpy(&Ext[ExtLen], UA_0, UL_0);
  ExtLen += UL_0;

  BLE_CmdStringify(BLE_CmdSendBuf, Type, Ext, ExtLen);
  *Cmd = BLE_CmdSendBuf;
  *CmdLen = BLE_CmdSendBuf[BLE_CMD_Msg_LenIdx];
  return true;
}

bool BLE_CmdStringify_Type_8_N(uint8** Cmd, uint8* CmdLen, BLE_Type_MsgType Type, uint8 U8_0, uint8* UA_0, uint8 UL_0){
  if(BLE_CMD_Msg_Len(1 + UL_0) > BLE_CMD_Msg_Max_Len) return false;
  uint8 ExtLen = 0;
  uint8 *Ext = &BLE_CmdSendBuf[BLE_CMD_Msg_ExtHeadIdx];
  Ext[ExtLen++] = U8_0;
  osal_memcpy(&Ext[ExtLen], UA_0, UL_0);
  ExtLen += UL_0;

  BLE_CmdStringify(BLE_CmdSendBuf, Type, Ext, ExtLen);
  *Cmd = BLE_CmdSendBuf;
  *CmdLen = BLE_CmdSendBuf[BLE_CMD_Msg_LenIdx];
  return true;
}

bool BLE_CmdStringify_Type_8_8_N(uint8** Cmd, uint8* CmdLen, BLE_Type_MsgType Type, uint8 U8_0, uint8 U8_1, uint8* UA_0, uint8 UL_0){
  if(BLE_CMD_Msg_Len(1 + 1 + UL_0) > BLE_CMD_Msg_Max_Len) return false;
  uint8 ExtLen = 0;
  uint8 *Ext = &BLE_CmdSendBuf[BLE_CMD_Msg_ExtHeadIdx];
  Ext[ExtLen++] = U8_0;
  Ext[ExtLen++] = U8_1;
  osal_memcpy(&Ext[ExtLen], UA_0, UL_0);
  ExtLen += UL_0;

  BLE_CmdStringify(BLE_CmdSendBuf, Type, Ext, ExtLen);
  *Cmd = BLE_CmdSendBuf;
  *CmdLen = BLE_CmdSendBuf[BLE_CMD_Msg_LenIdx];
  return true;
}

bool BLE_CmdStringify_Type_8_8(uint8** Cmd, uint8* CmdLen, BLE_Type_MsgType Type, uint8 U8_0, uint8 U8_1){
  if(BLE_CMD_Msg_Len(1 + 1) > BLE_CMD_Msg_Max_Len) return false;
  uint8 ExtLen = 0;
  uint8 *Ext = &BLE_CmdSendBuf[BLE_CMD_Msg_ExtHeadIdx];
  Ext[ExtLen++] = U8_0;
  Ext[ExtLen++] = U8_1;
  
  BLE_CmdStringify(BLE_CmdSendBuf, Type, Ext, ExtLen);
  *Cmd = BLE_CmdSendBuf;
  *CmdLen = BLE_CmdSendBuf[BLE_CMD_Msg_LenIdx];
  return true;
}

bool BLE_CmdStringify_Type_8_8_32(uint8** Cmd, uint8* CmdLen, BLE_Type_MsgType Type, uint8 U8_0, uint8 U8_1, uint32 U32_){
  if(BLE_CMD_Msg_Len(1 + 1 + 4) > BLE_CMD_Msg_Max_Len) return false;
  uint8 ExtLen = 0;
  uint8 *Ext = &BLE_CmdSendBuf[BLE_CMD_Msg_ExtHeadIdx];
  Ext[ExtLen++] = U8_0;
  Ext[ExtLen++] = U8_1;
  BLE_U32ToU8Array(U32_, Ext, &ExtLen);
  
  BLE_CmdStringify(BLE_CmdSendBuf, Type, Ext, ExtLen);
  *Cmd = BLE_CmdSendBuf;
  *CmdLen = BLE_CmdSendBuf[BLE_CMD_Msg_LenIdx];
  return true;
}

bool BLE_CmdStringify_Type_8(uint8**Cmd , uint8* CmdLen, BLE_Type_MsgType Type, uint8 U8_0){
  if(BLE_CMD_Msg_Len(1) > BLE_CMD_Msg_Max_Len) return false;
  uint8 ExtLen = 0;
  uint8 *Ext = &BLE_CmdSendBuf[BLE_CMD_Msg_ExtHeadIdx];
  Ext[ExtLen++] = U8_0;
  
  BLE_CmdStringify(BLE_CmdSendBuf, Type, Ext, ExtLen);
  *Cmd = BLE_CmdSendBuf;
  *CmdLen = BLE_CmdSendBuf[BLE_CMD_Msg_LenIdx];
  return true;
}

bool BLE_CmdStringify_Type(uint8**Cmd , uint8* CmdLen, BLE_Type_MsgType Type){
  if(BLE_CMD_Msg_Len(0) > BLE_CMD_Msg_Max_Len) return false;
  BLE_CmdStringify(BLE_CmdSendBuf, Type, 0, 0);
  *Cmd = BLE_CmdSendBuf;
  *CmdLen = BLE_CmdSendBuf[BLE_CMD_Msg_LenIdx];
  return true;
}


/*********************************************************************
 * @fn      BLE_ScanAdvertData_Construct
 *
 * @brief   
 *
 * @param   
 *
 * @return  none
 */
void BLE_ScanAdvertData_Construct(BLE_Type_ScanAdvertData* Data, uint8 DataLen, uint8* Dest, uint8 *DestLen)
{
 (*DestLen) = 0;
  for(uint8 i = 0; i < DataLen; i++){
   (Dest)[(*DestLen)++] = 1 + Data[i].DataLen;        //Data Length
   (Dest)[(*DestLen)++] = Data[i].DataType;   //Data Type
    osal_memcpy(&(Dest)[(*DestLen)], Data[i].Data, Data[i].DataLen); //Data Content
   (*DestLen) += Data[i].DataLen;
  }
}

/*********************************************************************
*********************************************************************/
