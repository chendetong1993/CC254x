/**************************************************************************************************
  Filename:       simpleBLE.h
  Revised:        $Date: 2011-03-03 15:46:41 -0800 (Thu, 03 Mar 2011) $
  Revision:       $Revision: 12 $

  Description:    
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
  
 
//****************************************************** Array Method *****************************************************//
#define BLE_Array_Len(_A) (sizeof(_A) / sizeof(_A[0]))
  
#define BLE_Array_SetStr(_D, _DL, _S) {\
    uint8 Str[] = _S;\
     _DL = sizeof(Str) - 1;\
    osal_memcpy(_D, Str, _DL);\
  }
  
#define BLE_Array_CopyFL(_D, _S) osal_memcpy(_D, _S, sizeof(_D));
  
#define BLE_Array_Copy(_D, _DL, _S, _SL) {\
    osal_memcpy(_D, _S, (_SL));\
    _DL = (_SL);\
  }

#define BLE_Array_Append(_D, _DL, _S, _SL) {\
    osal_memcpy(&_D[_DL], _S, (_SL));\
    _DL += (_SL);\
  }
  
#define BLE_Array_Compare(_D, _DL, _S, _SL) (((_DL) == (_SL)) && osal_memcmp(_D, _S, (_DL)))
    
#define BLE_Array_CompareFL(_D, _S) ((sizeof(_D) == sizeof(_S)) && osal_memcmp(_D, _S, sizeof(_D)))
    
#define BLE_Value_Swap(_A, _B, _T) {\
    _T SwapVal = _A;\
    _A = _B;\
    _B = SwapVal;\
  }
  
#define BLE_Array_SwapFL(_A, _B, _T) {\
    _T SwapVal;\
    uint8 SwapI = 0, SwapLen = sizeof(_A);\
    for(; SwapI <= SwapLen; SwapI++) {\
      SwapVal = _A[SwapI];\
      _A[SwapI] = _B[SwapI];\
      _B[SwapI] = SwapVal;\
    }\
  }
  
#define BLE_Array_Swap(_A, _AL, _B, _BL, _T) {\
    _T SwapVal;\
    uint8 SwapI = 0;\
    for(; SwapI <= _AL || SwapI <= _BL; SwapI++) {\
      SwapVal = _A[SwapI];\
      _A[SwapI] = _B[SwapI];\
      _B[SwapI] = SwapVal;\
    }\
    SwapI = _AL;\
    _AL = _BL;\
    _BL = SwapI;\
  }

#define BLE_Array_ItemExist_Array(_A, _AK, _V, _R) {\
    _R = false;\
    for(uint8 ItemIdx = 0, ItemLen = BLE_Array_Len(_A); ItemIdx < ItemLen; ItemIdx++){\
      if(BLE_Array_CompareFL(_A[ItemIdx]._AK, _V)){\
        _R = true;\
        break;\
      }\
    }\
  }
    
#define BLE_Array_ItemExist_Value(_A, _AK, _V, _R) {\
    _R = false;\
    for(uint8 ItemIdx = 0, ItemLen = BLE_Array_Len(_A); ItemIdx < ItemLen; ItemIdx++){\
      if(_A[ItemIdx]._AK == (_V)){\
        _R = true;\
        break;\
      }\
    }\
  }
  
#define BLE_Array_ItemFind_Array(_A, _AK, _V, _I) {\
    _I = 0;\
    for(uint8 ItemIdx = 0, ItemLen = BLE_Array_Len(_A); ItemIdx < ItemLen; ItemIdx++){\
      if(BLE_Array_CompareFL(_A[ItemIdx]._AK, _V)){\
        _I = &_A[ItemIdx];\
        break;\
      }\
    }\
  }
  
#define BLE_Array_ItemFind_Value(_A, _AK, _V, _I) {\
    _I = 0;\
    for(uint8 ItemIdx = 0, ItemLen = BLE_Array_Len(_A); ItemIdx < ItemLen; ItemIdx++){\
      if(_A[ItemIdx]._AK == (_V)){\
        _I = &_A[ItemIdx];\
        break;\
      }\
    }\
  }
//****************************************************** Const *****************************************************//
//User Definition

#define BLE_CMD_Msg_Max_Len                             6 + 31
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
#define BLE_Parm_Dev_Name_Max_Len                       20
  
#define BLE_Parm_Dev_Info_Max_Len                       16
  
#define BLEC_Dev_Max_Num                                3               // Number Limit of stored Connected/Scanned Devices information
  
#define BLE_TRANSMIT_ENCRYPT_DATA_LEN                   16

#define BLE_Queue_Max_Len                               80

#define BLE_Info_Max_Len                                16

#define BLE_SCAN_RSP_DATA_Max_Len                       31

#define BLE_OWN_CONN_HANDLE                            0xFF

#define BLE_TI_COMPANY_ID                         0x000D

typedef struct{
  uint8 Queue[BLE_Queue_Max_Len];
  uint8 Idx;
  uint8 Len;
} BLE_Type_Queue;

typedef struct{
#if defined(BLE_PERIPHERAL) && defined(BLE_CENTRAL)
  void (*SwitchRole)();
#endif
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
  BLE_AddOper_ReadRssi = 0x00,
  BLE_AddOper_ReadMac = 0x01,
} BLE_Type_AddOper;

typedef enum{
  BLE_AddRet_ReadRssi = 0x00,
  BLE_AddRet_ReadMac = 0x01,
} BLE_Type_AddRet;

typedef enum{
  BLE_Error_Hci = 0x00,
  BLE_Error_ATT = 0x01,
  BLE_Error_General = 0x02,
  BLE_Error_BleStatus = 0x03,
  BLE_Error_BleTerminate = 0x04,
  BLE_Error_CmdParse = 0x05
} BLE_Type_Error;

typedef enum {
  BLE_MsgType_Connect_EnAdvert = 0x00,
  BLE_MsgType_Disconnect_DisAdvert = 0x01,
  BLE_MsgType_RoleStatus_Get = 0x02,
  BLE_MsgType_ConnStatus_Get = 0x03,
  BLE_MsgType_Parms_Set = 0x04,
  BLE_MsgType_Send = 0x05,
  BLE_MsgType_Reboot = 0x06,
#if defined(BLE_PERIPHERAL) && defined(BLE_CENTRAL)
  BLE_MsgType_SwitchRole = 0x07,
#endif
  BLE_MsgType_Sleep = 0x08,
  BLE_MsgType_AdditOper = 0x09,
  BLE_MsgType_Customized_Get = 0x0a,
  
  BLE_MsgType_Cmd_Parsed = 0x78,
  BLE_MsgType_Error_Happened = 0x79,
  BLE_MsgType_Device_Inited = 0x80,
  BLE_MsgType_ConnStatus_Returned = 0x81,
  BLE_MsgType_Sended = 0x82,
  BLE_MsgType_UnSended = 0x83,
  BLE_MsgType_Received = 0x84,
  BLE_MsgType_UnReceived = 0x85,
  BLE_MsgType_AdditInfo_Ret = 0x86,
  BLE_MsgType_Customized_Ret = 0x87,
} BLE_Type_MsgType;


typedef enum{
  BLEMode_Central,
  BLEMode_Peripheral
} BLE_Type_BLEMode;

typedef struct {
  uint32 INTER_VERSION;
  BLE_Type_BLEMode CP;
  uint8 Info[BLE_Info_Max_Len];
  uint8 InfoLen;
} BLE_Type_Editable_CP;

typedef struct {
  uint8 DataType;
  uint8* Data;
  uint8 DataLen;
} BLE_Type_ScanAdvertData;

#define BLE_CP_Flash_Idx                (BLE_NVID_CUST_START)   // 0x00 ~ 0x0F Space


extern void BLE_Init(BLE_Type_Argv*);
extern bool BLE_QueueEn(BLE_Type_Queue*, uint8*, uint8);
extern bool BLE_QueueDe(BLE_Type_Queue*, uint8**, uint8*);
extern void BLE_CmdGetFromExter(uint8*, uint8);
extern void BLE_BleRetToExter(uint8, uint8*, uint8);
extern bool BLE_SendBLE(uint8, uint8*, uint8);
extern void BLE_UartRecTimeout();
extern void BLE_CmdRetTransmitDone(uint8, bool);
extern void BLE_CmdRetConnStatus(uint8*, uint8);
extern void BLE_CmdRetRoleStatus();
extern uint8 BLE_CmdRetError(BLE_Type_Error, uint8);
extern void BLE_CmdRetCmdParsed();
extern void BLE_CmdRetAddInfo(uint8, uint8, uint8*, uint8);

extern bool BLE_CmdExtParse_8_32(uint8*, uint8, uint8*, uint32*, bool);
extern bool BLE_CmdExtParse_N(uint8*, uint8, uint8**, uint8*, bool);
extern bool BLE_CmdExtParse_8(uint8*, uint8, uint8*, bool);
extern bool BLE_CmdExtParse_8_8_N(uint8*, uint8, uint8*, uint8*, uint8**, uint8*, bool);
extern bool BLE_CmdExtParse_8_8(uint8*, uint8, uint8*, uint8*, bool);
extern bool BLE_CmdExtParse_8_8_32(uint8*, uint8, uint8*, uint8*, uint32*, bool);
extern bool BLE_CmdExtParse_8_N(uint8*, uint8, uint8*, uint8**, uint8*, bool);
extern bool BLE_CmdExtParse(uint8*, uint8, bool);
extern bool BLE_CmdParse(uint8*, uint8, uint8*, uint8*, uint8**, uint8*, bool);

extern bool BLE_CmdStringify_Type_8_32(uint8**, uint8*, BLE_Type_MsgType, uint8, uint32);
extern bool BLE_CmdStringify_Type_8_N(uint8**, uint8*, BLE_Type_MsgType, uint8, uint8*, uint8);
extern bool BLE_CmdStringify_Type_N(uint8**, uint8*, BLE_Type_MsgType, uint8*, uint8);
extern bool BLE_CmdStringify_Type_8_8_N(uint8**, uint8*, BLE_Type_MsgType, uint8, uint8, uint8*, uint8);
extern bool BLE_CmdStringify_Type_8_8(uint8**, uint8*, BLE_Type_MsgType, uint8, uint8);
extern bool BLE_CmdStringify_Type_8_8_32(uint8**, uint8*, BLE_Type_MsgType, uint8, uint8, uint32);
extern bool BLE_CmdStringify_Type_8(uint8**, uint8*, BLE_Type_MsgType, uint8);
extern bool BLE_CmdStringify_Type(uint8** , uint8*, BLE_Type_MsgType);
extern void BLE_ScanAdvertData_Construct(BLE_Type_ScanAdvertData*, uint8 DataLen, uint8*, uint8*);
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
