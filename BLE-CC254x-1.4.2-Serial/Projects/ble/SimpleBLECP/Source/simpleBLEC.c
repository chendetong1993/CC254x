/**************************************************************************************************
  Filename:       BLEC_Central.c
  Revised:        $Date: 2011-06-20 11:57:59 -0700(Mon, 20 Jun 2011) $
  Revision:       $Revision: 28 $

  Description:    This file contains the Simple BLE Central sample application 
                  for use with the CC2540 Bluetooth Low Energy Protocol Stack.

  Copyright 2010 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer(which must be your employer)
  and Texas Instruments Incorporated(the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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
#if defined(BLE_CENTRAL)
   
#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "OnBoard.h"
#include "gatt.h"
#include "ll.h"
#include "hci.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "central.h"
#include "gapbondmgr.h"
#include "simpleGATTprofile.h"
#include "SimpleBLEC.h"
#include "OSAL_snv.h"
#include "simpleBLE.h"
#include "npi.h"
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
// Simple BLE Central Task Events
#define E       BLEC_GVar
#define PS      BLEC_GVar->EParms_Save
#define P      BLEC_GVar->EParms

#define Bool(_V_)(_V_ == 0 || _V_ == 1)
#define PU16(_V_)(0 < _V_ && _V_ <= 0xFFFF)
#define U16(_V_)(_V_ <= 0xFFFF)
#define PU8(_V_)(0 < _V_ && _V_ <= 0xFF)
#define U8(_V_)(_V_ <= 0xFF)
#define PU32(_V_)(0 < _V_ && _V_ <= 0xFFFFFFFF)
#define U32(_V_)(_V_ <= 0xFFFFFFFF)
#define Sm(_V0_, _V1_)(_V0_ < _V1_)
#define SE(_V0_, _V1_)(_V0_ <= _V1_)
#define Eq(_V0_, _V1_)(_V0_ == _V1_)
#define La(_V0_, _V1_) (_V0_ > _V1_)
#define LE(_V0_, _V1_) (_V0_ >= _V1_)
#define BLEC_ADDR_LEN B_ADDR_LEN

#define BLEC_START_DEVICE_EVT                 (0x0001 << 0)
#define BLEC_RESET_EVT                        (0x0001 << 1)             // Reset Timer
#define BLEC_SCAN_EVT                         (0x0001 << 2)             // Scan Timer
#define BLEC_END_SCAN_EVT                     (0x0001 << 3)             // End Scan Timer
#define BLEC_CONNECT_EVT                      (0x0001 << 4)             // Connect Timer
#define BLEC_Sleep_EVT                        (0x0001 << 5)             // Sleep
#define BLEC_PARM_SAVE_EVT                    (0x0001 << 6)             // Save Parameter

#define BLEC_DISCOVERY_MODE                DEVDISC_MODE_ALL     // Discovey mode(limited, general, all)

#define BLEC_DISCOVERY_ACTIVE_SCAN         true         // true to use active scan

#define BLEC_DISCOVERY_WHITE_LIST          FALSE        // true to use white list during discovery

#define BLEC_LINK_HIGH_DUTY_CYCLE          FALSE        // true to use high scan duty cycle when creating link

#define BLEC_LINK_WHITE_LIST               FALSE        // true to use white list when creating link

#define BLEC_PARM_SAVE_DELAY               200

/*********************************************************************
 * TYPEDEFS
 */

// Discovery states
typedef enum {
  BLE_DISC_STATE_IDLE,                // Idle
  BLE_DISC_STATE_SVC,                 // Service discovery
  BLE_DISC_STATE_CHAR_SC                 // Characteristic discovery
} BLEC_Type_DISC_STATE;

typedef enum {
  BLEC_ConnStatus_Unknown,
  BLEC_ConnStatus_Scanned,
  BLEC_ConnStatus_Connecting,
  BLEC_ConnStatus_Connected,
} BLEC_Type_ConnStatus;

//Scaned && Connected Device
typedef struct{
  uint16 ConnHandle;    // Connection handle of current connection 
  BLEC_Type_DISC_STATE DiscState;      // Discovery state
  uint16 SvcStartHdl;   // Discovered service start and end handle
  uint16 SvcEndHdl;
  uint16 CharHdSC;       // Discovered characteristic handle
  bool CharSCDoWrite;
  BLEC_Type_ConnStatus ConnStatus;
  uint8 addrType;
  uint8 addr[BLEC_ADDR_LEN];
  uint8 ScanRspData[BLE_SCAN_RSP_DATA_Max_Len];
  uint8 ScanRspDataLen;
  BLE_Type_Queue SendQueue;
  int8 ScanRssi;
  uint8 Token;
} BLEC_Type_Dev;

typedef struct{
  BLE_Type_CommFunc* BLE_CommFunc;
  BLEC_Type_EParms EParms;
  BLEC_Type_EParms EParms_Save;
  bool EParms_Save_Updated;
  uint16 PERIOD_SCAN_DEVICE_COUNT;                                              // Current Scan Device
  uint8 TaskId;                                                                // Task ID for internal task/event processing
  bool Scanning;                                                            // Scanning state
  bool Connecting; 
  BLEC_Type_Dev DevList[BLEC_Dev_Max_Num];
  bool DevIsConn[BLEC_Dev_Max_Num];
  uint8 Addr[BLEC_ADDR_LEN];
  int8 SCANNED_DEV_MEAN_RSSI; 
  gapCentralRoleCB_t RoleCB;                                                    // GAP Role Callbacks
  gapBondCBs_t BondCB;                                                          // Bond Manager Callbacks
} BLEC_Type_GlobalVar;

/*********************************************************************
 * GLOBAL VARIABLES
 */
BLEC_Type_GlobalVar* BLEC_GVar = 0;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
void BLEC_WakeupNotify();
void BLEC_Reset_EParms(BLEC_Type_EParms*);
void BLEC_Init_GlobalVar();
void BLEC_ProcessGATTMsg(gattMsgEvent_t *);
void BLEC_RssiCB(uint16 connHandle, int8);
uint8 BLEC_EventCB(gapCentralRoleEvent_t *);
void BLEC_ProcessOSALMsg(osal_event_hdr_t *);
void BLEC_GATTDiscoveryEvent(gattMsgEvent_t *);
bool BLEC_ScanAdvertDataSearch(uint8 type, uint8 *, uint8, uint8*, uint8*);
void BLEC_Connect();
void BLEC_CmdHandle(uint8*, uint8);
void BLEC_CentralPasscodeCB(uint8*, uint16, uint8, uint8);
void BLEC_CentralPairStateCB(uint16, uint8, uint8);
uint32 BLEC_End_Scan_Conn();
bool BLEC_BleSend(uint8, uint8*, uint8);
void BLEC_Main_Loop();
/*********************************************************************
 * PUBLIC FUNCTIONS
 */
   
/*********************************************************************
 * @fn      BLEP_Wakeup
 *
 * @brief   
 * @param   
 *
 f @return  none
 */
void BLEC_WakeupNotify(){
}

/*********************************************************************
 * @fn      BLEC_Reset_EParms
 *
 * @brief   
 * @param   
 *
 f @return  none
 */
void BLEC_Reset_EParms(BLEC_Type_EParms* EParms){
  EParms->INTER_VERSION = BLE_Parm_INTER_VERSION;
  EParms->ENABLE_UPDATE_REQUEST = true;
  EParms->UPDATE_MIN_CONN_INTERVAL = 32;
  EParms->UPDATE_MAX_CONN_INTERVAL = 32;
  EParms->UPDATE_SLAVE_LATENCY = 2;
  EParms->UPDATE_CONN_TIMEOUT = 200;
  EParms->SCAN_RSSI_THRESHOLD  = -68;
  EParms->SCAN_DURATION = 1000;
  EParms->SCAN_TOTAL_DURATION = 4000;
  EParms->MULTI_CONNECT_INTERVAL = 200;
  EParms->PERIOD_SCAN_DEVICE_MAX_NUM = 10;
  EParms->PERIOD_SCAN_DEVICE_OVERFLOW_COUNT = 9;
  EParms->END_ALL_SCAN_CONN_REQ_TIME = 150;
  BLE_Array_SetStr(EParms->NAME, EParms->NAME_LEN, "BLEC");
  EParms->PAIR_MODE = BLE_PairMode_Passcode_WaitForReq;
  EParms->PASSCODE = 123456;
  EParms->INFO_LEN = 0;
  
  EParms->ENABLE_TRANSMIT_ENCRYPT = false;
  osal_memset(EParms->TRANSMIT_ENCRYPT_KEY, 0x00, BLE_TRANSMIT_ENCRYPT_DATA_LEN);
  EParms->WATCHDOG_MODE = HAL_SYSTEM_WD_MODE_DISABLE;
  EParms->ENABLE_CMD_CHECK_BIT = false;
  
  uint8 COMPANY_ID[] = { HI_UINT16(BLE_TI_COMPANY_ID), LO_UINT16(BLE_TI_COMPANY_ID)};
  for(uint8 i = 0; i < BLEC_Dev_Max_Num; i++){
    EParms->SCAN_RSP_DATA_LEN[i] = 0;
    BLE_Type_ScanAdvertData SCAN_DATA[] = {
      {GAP_ADTYPE_MANUFACTURER_SPECIFIC, COMPANY_ID, BLE_Array_Len(COMPANY_ID)}
    };
    BLE_ScanAdvertData_Construct(SCAN_DATA, BLE_Array_Len(SCAN_DATA), EParms->SCAN_RSP_DATA[i], &EParms->SCAN_RSP_DATA_LEN[i]);
  }
}

/*********************************************************************
 * @fn      BLEC_Init_GlobalVar
 *
 * @brief   
 * @param   
 *
 f @return  none
 */
void BLEC_Init_GlobalVar(){
  
  E =(BLEC_Type_GlobalVar*)osal_mem_alloc(sizeof(BLEC_Type_GlobalVar));
  // Others
  E->EParms_Save_Updated = false;
  E->PERIOD_SCAN_DEVICE_COUNT = 0;
  E->TaskId = 0;
  E->Scanning = FALSE;
  E->Connecting = FALSE;
  E->SCANNED_DEV_MEAN_RSSI = 0; 
  osal_memset(E->Addr, 0, BLEC_ADDR_LEN);
  //System
  E->RoleCB.rssiCB = BLEC_RssiCB;
  E->RoleCB.eventCB = BLEC_EventCB;
  BLEC_Type_Dev* T;
  for(uint8 i = 0; i < BLEC_Dev_Max_Num; i++){
    T = &E->DevList[i];
    T->Token = i;
    T->ConnStatus = BLEC_ConnStatus_Unknown;
    T->ConnHandle = 0xFFFF;
  }
  osal_memset(E->DevIsConn, false, BLEC_Dev_Max_Num);
  E->BondCB.passcodeCB = BLEC_CentralPasscodeCB;
  E->BondCB.pairStateCB = BLEC_CentralPairStateCB;
}

/*********************************************************************
 * @fn      BLEC_Init
 *
 * @brief   Initialization function for the Simple BLE Central App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization(ie. hardware
 *          initialization/setup, table initialization, power up
 *          notification).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 f @return  none
 */
void BLEC_Init(uint8 task_id , BLE_Type_CommFunc* BLE_CommFunc)
{
  BLEC_Init_GlobalVar();
  osal_set_loop_func(BLEC_Main_Loop);
  E->BLE_CommFunc = BLE_CommFunc;
  E->BLE_CommFunc->CmdReceive = BLE_CmdGetFromExter;
  E->BLE_CommFunc->WakeupNotify = BLEC_WakeupNotify;
  
  E->TaskId = task_id;
  
  //Read Parms from flash
  if(osal_snv_read(BLEC_Parms_Flash_Idx, sizeof(P),(uint8*)&P) != SUCCESS ||
  P.INTER_VERSION != BLE_Parm_INTER_VERSION){      //Check whether data is valid
    BLEC_Reset_EParms(&P);
  }
  PS = P;
  for(uint8 i = 0; i < BLEC_Dev_Max_Num; i++){
    BLE_Array_Copy(E->DevList[i].ScanRspData, E->DevList[i].ScanRspDataLen, P.SCAN_RSP_DATA[i], P.SCAN_RSP_DATA_LEN[i]);
  }

  BLE_Type_Argv BLE_Argv = {
    .Role = BLE_CMD_Msg_RelDev_C,
    .Info = P.INFO,
    .InfoLen = P.INFO_LEN,
    .EnCmdCheckBit= P.ENABLE_CMD_CHECK_BIT,
    .EnTranEncry = P.ENABLE_TRANSMIT_ENCRYPT,
    .TranEncryKey = P.TRANSMIT_ENCRYPT_KEY,
    .CmdSend = BLE_CommFunc->CmdSend,
    .BleSend = BLEC_BleSend,
    .CmdRec = BLEC_CmdHandle
  };
  BLE_Init(&BLE_Argv);
  
  // Setup Central Profile
  {
    uint8 PERIOD_SCAN_DEVICE_MAX_NUM = P.PERIOD_SCAN_DEVICE_MAX_NUM;
    BLE_CmdRetError(BLE_Error_BleStatus, GAPCentralRole_SetParameter(GAPCENTRALROLE_MAX_SCAN_RES, sizeof(uint8), &PERIOD_SCAN_DEVICE_MAX_NUM));
  }
  
  // Setup GAP
  BLE_CmdRetError(BLE_Error_BleStatus, GAP_SetParamValue(TGAP_GEN_DISC_SCAN, P.SCAN_DURATION));
  BLE_CmdRetError(BLE_Error_BleStatus, GAP_SetParamValue(TGAP_LIM_DISC_SCAN, P.SCAN_DURATION));
  BLE_CmdRetError(BLE_Error_BleStatus, GGS_SetParameter(GGS_DEVICE_NAME_ATT, P.NAME_LEN,(uint8*)P.NAME));

  // Setup the GAP Bond Manager
  {
    uint32 passkey = 0;
    uint8 pairMode = 
    (P.PAIR_MODE == BLE_PairMode_Passcode_Initiate || P.PAIR_MODE == BLE_PairMode_Initiate)?
      GAPBOND_PAIRING_MODE_INITIATE:
      GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    uint8 mitm = 
    (P.PAIR_MODE == BLE_PairMode_Passcode_Initiate || P.PAIR_MODE == BLE_PairMode_Passcode_WaitForReq)?
      true:
      false;
    uint8 ioCap =(P.PAIR_MODE == BLE_PairMode_Passcode_Initiate || P.PAIR_MODE == BLE_PairMode_Initiate)?
      GAPBOND_IO_CAP_DISPLAY_ONLY:
      GAPBOND_IO_CAP_KEYBOARD_ONLY;
    uint8 bonding =(P.PAIR_MODE == BLE_PairMode_Passcode_Initiate || P.PAIR_MODE == BLE_PairMode_Initiate)?
      false:
      true;
    BLE_CmdRetError(BLE_Error_BleStatus, GAPBondMgr_SetParameter(GAPBOND_DEFAULT_PASSCODE, sizeof(passkey), &passkey));
    BLE_CmdRetError(BLE_Error_BleStatus, GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(pairMode), &pairMode));
    BLE_CmdRetError(BLE_Error_BleStatus, GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(mitm), &mitm));
    BLE_CmdRetError(BLE_Error_BleStatus, GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(ioCap), &ioCap));
    BLE_CmdRetError(BLE_Error_BleStatus, GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(bonding), &bonding));
  }

  
  // Initialize GATT Client
  GATT_InitClient();

  // Register to receive incoming ATT Indications/Notifications
  GATT_RegisterForInd(E->TaskId);

  // Initialize GATT attributes
  BLE_CmdRetError(BLE_Error_BleStatus, GGS_AddService(GATT_ALL_SERVICES));         // GAP
  BLE_CmdRetError(BLE_Error_BleStatus, GATTServApp_AddService(GATT_ALL_SERVICES)); // GATT attributes

  // Setup a delayed profile startup
  BLE_CmdRetError(BLE_Error_General, osal_set_event(E->TaskId, BLEC_START_DEVICE_EVT));
   
  BLE_CmdRetError(BLE_Error_Hci, HCI_EXT_ClkDivOnHaltCmd(LL_EXT_DISABLE_CLK_DIVIDE_ON_HALT));
  
  HAL_SYSTEM_SET_WATCHDOG(P.WATCHDOG_MODE);
}

/*********************************************************************
 * @fn      BLEC_ProcessEvent
 *
 * @brief   Simple BLE Central Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 BLEC_ProcessEvent(uint8 task_id, uint16 events)
{
  VOID task_id; // OSAL required parameter that isn't used in this function
  if(events & SYS_EVENT_MSG)
  {
    uint8 *pMsg;
    if((pMsg = osal_msg_receive(E->TaskId)) != NULL)
    {
      BLEC_ProcessOSALMsg((osal_event_hdr_t *)pMsg);
      // Release the OSAL message
      VOID osal_msg_deallocate(pMsg);
    }
    // return unprocessed events
    return(events ^ SYS_EVENT_MSG);
  }
  if(events & BLEC_START_DEVICE_EVT)
  {
    // Start the Device
    BLE_CmdRetError(BLE_Error_BleStatus, GAPCentralRole_StartDevice((gapCentralRoleCB_t *) &E->RoleCB));
    // Register with bond manager after starting device
    GAPBondMgr_Register((gapBondCBs_t *) &E->BondCB);
    return(events ^ BLEC_START_DEVICE_EVT);
  }
  // *********************** Handle Own defined Timer ********************************* //
  if(events & BLEC_RESET_EVT)
  {
    BLEC_ProcessEvent(task_id, BLEC_PARM_SAVE_EVT);
    SystemResetSoft();
    return(events ^ BLEC_RESET_EVT);
  }
  if(events & BLEC_PARM_SAVE_EVT)
  {
    if(E->EParms_Save_Updated == true) {
      BLE_CmdRetError(BLE_Error_General, osal_snv_write(BLEC_Parms_Flash_Idx, sizeof(PS),(uint8*)&PS));
      E->EParms_Save_Updated = false;
    }
    return(events ^ BLEC_PARM_SAVE_EVT);
  }
  
  if(events & BLEC_SCAN_EVT)
  {
    E->Scanning = true;
    BLE_CmdRetError(BLE_Error_BleStatus, GAPCentralRole_StartDiscovery(BLEC_DISCOVERY_MODE, BLEC_DISCOVERY_ACTIVE_SCAN, BLEC_DISCOVERY_WHITE_LIST));
    BLE_CmdRetError(BLE_Error_General, osal_start_timerEx(E->TaskId, BLEC_END_SCAN_EVT, P.SCAN_TOTAL_DURATION));
    return(events ^ BLEC_SCAN_EVT);
  }
  if(events & BLEC_END_SCAN_EVT)
  {
    BLE_CmdRetError(BLE_Error_BleStatus, GAPCentralRole_CancelDiscovery());
    E->PERIOD_SCAN_DEVICE_COUNT = 0;
    return(events ^ BLEC_END_SCAN_EVT);
  }
  
  if(events & BLEC_CONNECT_EVT)
  {
    BLEC_Connect();
    return(events ^ BLEC_CONNECT_EVT);
  }
  
  if(events & BLEC_Sleep_EVT) {
    E->BLE_CommFunc->ForceSleep();
    return(events ^ BLEC_Sleep_EVT);
  }
  // *********************************************************************** //
  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      BLEC_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
void BLEC_ProcessOSALMsg(osal_event_hdr_t *pMsg)
{
  switch(pMsg->event)
  {
    case GATT_MSG_EVENT:
      BLEC_ProcessGATTMsg((gattMsgEvent_t *) pMsg);
      break;
  }
}

/*********************************************************************
 * @fn      BLEC_ProcessGATTMsg
 *
 * @brief   Process GATT messages
 *
 * @return  none
 */
void BLEC_ProcessGATTMsg(gattMsgEvent_t *pMsg)
{
  BLEC_Type_Dev* T = 0;
  BLE_Array_ItemFind_Value(E->DevList, ConnHandle, pMsg->connHandle, T);
  if(T != 0){
    if((pMsg->method == ATT_READ_RSP) ||
    ((pMsg->method == ATT_ERROR_RSP) &&
    (pMsg->msg.errorRsp.reqOpcode == ATT_READ_REQ))){
      if(pMsg->method == ATT_ERROR_RSP) {
        //uint8 status = pMsg->msg.errorRsp.errCode;
        ////LCD_WRITE_STRING_VALUE("Read Error", status, 10, HAL_LCD_LINE_1);
        BLE_CmdRetError(BLE_Error_ATT, pMsg->msg.errorRsp.errCode);
      }
      else{
        // After a successful read, display the read value
        //uint8 valueRead = pMsg->msg.readRsp.value[0];
        //LCD_WRITE_STRING_VALUE("Read rsp:", valueRead, 10, HAL_LCD_LINE_1);
      }
    }
    else if((pMsg->method == ATT_WRITE_RSP) ||((pMsg->method == ATT_ERROR_RSP) &&(pMsg->msg.errorRsp.reqOpcode == ATT_WRITE_REQ))){
      if(T->CharSCDoWrite == false) {
        bool Sended = !(pMsg->method == ATT_ERROR_RSP == ATT_ERROR_RSP);
        // After a succesful write, display the value that was written and increment value
        BLE_CmdRetError(BLE_Error_ATT, Sended ? SUCCESS : pMsg->msg.errorRsp.errCode);
        BLE_CmdRetTransmitDone(T->Token, Sended ? true : false);
        T->CharSCDoWrite = true;
        uint8* NewSend;
        uint8 NewSendLen;
        while(BLE_QueueDe(&T->SendQueue, &NewSend, &NewSendLen)){
          if(BLEC_BleSend(T->ConnHandle, NewSend, NewSendLen) == false){
            BLE_CmdRetTransmitDone(T->ConnHandle, false);
          } else {
            break;
          }
        }
      }
    } else if(T->DiscState != BLE_DISC_STATE_IDLE) {
      BLEC_GATTDiscoveryEvent(pMsg);
    } else if((pMsg->method == ATT_HANDLE_VALUE_NOTI)){    //通知 
      if(pMsg->msg.handleValueNoti.handle == SIMPLEPROFILE_CHAR_CS_UUID_READ_HANDLE){   //CHAR7的通知  串口打印
        BLE_BleRetToExter(T->Token, pMsg->msg.handleValueNoti.pValue, pMsg->msg.handleValueNoti.len);
      }
    }
  }
  GATT_bm_free(&pMsg->msg, pMsg->method);
}

/*********************************************************************
 * @fn      BLEC_RssiCB
 *
 * @brief   RSSI callback.
 *
 * @param   connHandle - connection handle
 * @param   rssi - RSSI
 *
 * @return  none
 */
void BLEC_RssiCB(uint16 connHandle, int8 rssi)
{
  BLEC_Type_Dev* T;
  BLE_Array_ItemFind_Value(E->DevList, ConnHandle, connHandle, T);
  if(T != 0){
    uint8 rssiPos = -1 * rssi;
    BLE_CmdRetAddInfo(BLE_AddRet_ReadRssi, T->Token, &rssiPos, 1);
  }
}

/*********************************************************************
 * @fn      pairStateCB
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
void BLEC_CentralPairStateCB(uint16 connHandle, uint8 state, uint8 status){
  if(state == GAPBOND_PAIRING_STATE_STARTED) {
    //LCD_WRITE_STRING("Pairing started", HAL_LCD_LINE_1);
  } else if(state == GAPBOND_PAIRING_STATE_COMPLETE) {
    if(status == SUCCESS || status == SMP_PAIRING_FAILED_UNSPECIFIED) {
      //LCD_WRITE_STRING("Pairing success", HAL_LCD_LINE_1);
    } else {
      BLE_CmdRetError(BLE_Error_BleStatus, GAPCentralRole_TerminateLink(connHandle));
    }
  } else if(state == GAPBOND_PAIRING_STATE_BONDED) {
    if(status == SUCCESS){
      //LCD_WRITE_STRING("Bonding success", HAL_LCD_LINE_1);
    }
  }
}

/*********************************************************************
 * @fn      BLEC_CentralPasscodeCB
 *
 * @brief   Passcode callback.
 *
 * @return  none
 */
void BLEC_CentralPasscodeCB(uint8 *deviceAddr, uint16 connectionHandle, uint8 uiInputs, uint8 uiOutputs){
  BLE_CmdRetError(BLE_Error_BleStatus, GAPBondMgr_PasscodeRsp(connectionHandle, SUCCESS, P.PASSCODE));
}


/*********************************************************************
 * @fn      BLEC_ScanAdvertDataSearch
 *
 * @brief   
 *
 * @return  true if  found
 */
bool BLEC_ScanAdvertDataPair(uint8 *pD, uint8 pDL, uint8 *pK, uint8 pKL)
{
  uint8 pKLen, pDLen;
  uint8 pKType, pDType;
  uint8 pKIdx = 0, pDIdx = 0;
  bool Match = false;
  while(pKIdx <  pKL){
    pKLen = pK[pKIdx++] - 1;
    pKType = pK[pKIdx++];
    pDIdx = 0;
    Match = false;
    while(pDIdx <  pDL){
      pDLen = pD[pDIdx++] - 1;
      pDType = pD[pDIdx++];
      if(pKType == pDType){
        Match = BLE_Array_Compare(&pD[pDIdx], pDLen, &pK[pKIdx], pKLen);
        break;
      }
      pDIdx += pDLen;
    }
    if(Match == false){
      return false;
    }
    pKIdx += pKLen;
  }
  return true;
}


/*********************************************************************
 * @fn      BLEC_EventCB
 *
 * @brief   Central event callback function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  none
 */
uint8 BLEC_EventCB(gapCentralRoleEvent_t *pEvent)
{
  switch(pEvent->gap.opcode)
  {
    case GAP_DEVICE_INIT_DONE_EVENT:
      BLE_CmdRetError(BLE_Error_BleStatus, GAPCentralRole_GetParameter(GAPCENTRALROLE_BD_ADDR, E->Addr));
      BLE_CmdRetRoleStatus();
      break;
    case GAP_DEVICE_INFO_EVENT:
       {
        switch(pEvent->deviceInfo.eventType){
          case GAP_ADRPT_ADV_IND:
            {
              E->PERIOD_SCAN_DEVICE_COUNT++; // Scanned Device Counter + 1
              bool ScanMatched;
              BLE_Array_ItemExist_Array(E->DevList, addr, pEvent->deviceInfo.addr, ScanMatched);
              if(ScanMatched == false){
                //Compare UUID
                uint8 _16BIT[] = { LO_UINT16(SIMPLEPROFILE_SERV_UUID), HI_UINT16(SIMPLEPROFILE_SERV_UUID)};
                BLE_Type_ScanAdvertData ADVERT_DATA[] = {{GAP_ADTYPE_16BIT_COMPLETE, _16BIT, BLE_Array_Len(_16BIT)}};
                uint8 TA[1 + 1 + 2];
                uint8 TAL = 0;
                BLE_ScanAdvertData_Construct(ADVERT_DATA, BLE_Array_Len(ADVERT_DATA), TA, &TAL);
                E->SCANNED_DEV_MEAN_RSSI = BLEC_ScanAdvertDataPair(pEvent->deviceInfo.pEvtData, pEvent->deviceInfo.dataLen, TA, TAL) ? pEvent->deviceInfo.rssi : 0;
              }
            }
            break;
          case GAP_ADRPT_SCAN_RSP:
            {
              if(E->SCANNED_DEV_MEAN_RSSI != 0) {
                // Filter RSSI(Average Method)
                E->SCANNED_DEV_MEAN_RSSI = ((int16)E->SCANNED_DEV_MEAN_RSSI + (int16)pEvent->deviceInfo.rssi) / 2;
                // Close Enough
                if(E->SCANNED_DEV_MEAN_RSSI  > P.SCAN_RSSI_THRESHOLD){
                  int8 Rssi = E->SCANNED_DEV_MEAN_RSSI;
                  uint8 Addr[BLEC_ADDR_LEN];
                  uint8 AddrType = pEvent->deviceInfo.addrType;
                  uint8 ScanRspData[BLE_SCAN_RSP_DATA_Max_Len];
                  uint8 ScanRspDataLen;
                  
                  Rssi = E->SCANNED_DEV_MEAN_RSSI;
                  BLE_Array_CopyFL(Addr, pEvent->deviceInfo.addr);
                  AddrType = pEvent->deviceInfo.addrType;
                  BLE_Array_Copy(ScanRspData, ScanRspDataLen, pEvent->deviceInfo.pEvtData, pEvent->deviceInfo.dataLen);

                  bool ReInsert = false; 
                  do {
                    ReInsert = false;
                    for(uint8 i = 0; i < BLEC_Dev_Max_Num; i++){ 
                      // RSP Matched
                      if(BLEC_ScanAdvertDataPair(E->DevList[i].ScanRspData, E->DevList[i].ScanRspDataLen, ScanRspData, ScanRspDataLen)) {
                        if(E->DevList[i].ConnStatus == BLEC_ConnStatus_Unknown) {   // First Device
                          E->DevList[i].ConnStatus = BLEC_ConnStatus_Scanned;
                          BLE_Array_CopyFL(E->DevList[i].addr, Addr);
                          BLE_Array_Copy(E->DevList[i].ScanRspData, E->DevList[i].ScanRspDataLen, ScanRspData, ScanRspDataLen);
                          E->DevList[i].addrType = AddrType;
                          E->DevList[i].ScanRssi = Rssi;
                          break;
                        } else if(E->DevList[i].ConnStatus == BLEC_ConnStatus_Scanned && Rssi  >= E->DevList[i].ScanRssi){ // Device Existed
                          // Swap
                          BLE_Value_Swap(E->DevList[i].ScanRssi, Rssi, int);
                          BLE_Value_Swap(E->DevList[i].addrType, AddrType, uint8);
                          BLE_Array_SwapFL(E->DevList[i].addr, Addr, uint8);
                          BLE_Array_Swap(E->DevList[i].ScanRspData, E->DevList[i].ScanRspDataLen, ScanRspData, ScanRspDataLen, uint8);
                          ReInsert = true;
                          break;
                        }
                      }
                    }
                  } while(ReInsert == true);
                  if(E->PERIOD_SCAN_DEVICE_COUNT >= P.PERIOD_SCAN_DEVICE_OVERFLOW_COUNT){
                    BLE_CmdRetError(BLE_Error_BleStatus, GAPCentralRole_CancelDiscovery());
                  }
                }
              }
            }
            break;
        }
      }
      break;
    case GAP_DEVICE_DISCOVERY_EVENT:
      {
        if(E->Scanning == true){
          uint32 CurrentTime = osal_GetSystemClock();
          if(E->PERIOD_SCAN_DEVICE_COUNT >= P.PERIOD_SCAN_DEVICE_OVERFLOW_COUNT){
            //Resample
            E->PERIOD_SCAN_DEVICE_COUNT = 0;
            BLE_CmdRetError(BLE_Error_BleStatus, GAPCentralRole_StartDiscovery(BLEC_DISCOVERY_MODE, BLEC_DISCOVERY_ACTIVE_SCAN, BLEC_DISCOVERY_WHITE_LIST));
          } else {
            E->PERIOD_SCAN_DEVICE_COUNT = 0;
            if(osal_get_timeoutEx(E->TaskId, BLEC_END_SCAN_EVT)){
              BLE_CmdRetError(BLE_Error_General, osal_stop_timerEx(E->TaskId, BLEC_END_SCAN_EVT));
            }
            E->Scanning = FALSE;
            uint8 ScannMatchedCount = 0;
            for(uint8 i = 0; i < BLEC_Dev_Max_Num; i++){
              if(E->DevList[i].ConnStatus == BLEC_ConnStatus_Scanned) {
                ScannMatchedCount++;
              }
            }
            if(ScannMatchedCount == 0){
              BLE_CmdRetError(BLE_Error_BleStatus, bleNoResources);
            } else {
              BLE_CmdRetError(BLE_Error_General, osal_start_timerEx(E->TaskId, BLEC_CONNECT_EVT, P.MULTI_CONNECT_INTERVAL));
            }
          }
        }
      }
      break;
    case GAP_LINK_ESTABLISHED_EVENT:
      {
        if(E->Connecting == true){
          uint32 CurrentTime = osal_GetSystemClock();
          if(pEvent->gap.hdr.status == SUCCESS)
          {          
            BLEC_Type_Dev* T;
            for(uint8 i = 0; i < BLEC_Dev_Max_Num; i++){
              T = &E->DevList[i];
              if(T->ConnStatus == BLEC_ConnStatus_Connecting && osal_memcmp(T->addr, pEvent->linkCmpl.devAddr, BLEC_ADDR_LEN) == true){
                T->DiscState = BLE_DISC_STATE_SVC;
                T->SvcStartHdl = 0;
                T->SvcEndHdl = 0;
                T->CharHdSC = 0;
                T->CharSCDoWrite = true;
                T->SendQueue.Len = 0;
                T->ConnHandle = pEvent->linkCmpl.connectionHandle;
                T->ConnStatus = BLEC_ConnStatus_Connected;
                uint8 uuid[ATT_BT_UUID_SIZE] = { LO_UINT16(SIMPLEPROFILE_SERV_UUID), HI_UINT16(SIMPLEPROFILE_SERV_UUID) };
                BLE_CmdRetError(BLE_Error_BleStatus, GATT_DiscPrimaryServiceByUUID(T->ConnHandle, uuid, ATT_BT_UUID_SIZE, E->TaskId));
                E->BLE_CommFunc->ForceWakeup();
                E->DevIsConn[i] = true;
                BLE_CmdRetConnStatus(E->DevIsConn, BLEC_Dev_Max_Num); 
                break;
              }
            }
          }
          BLE_CmdRetError(BLE_Error_General, osal_start_timerEx(E->TaskId, BLEC_CONNECT_EVT, P.MULTI_CONNECT_INTERVAL));
          E->Connecting = FALSE;
        } else {
          if(pEvent->gap.hdr.status == SUCCESS)
          {
            BLE_CmdRetError(BLE_Error_BleStatus, GAPCentralRole_TerminateLink(pEvent->linkCmpl.connectionHandle));
          }
        }
      }
      break;
    case GAP_LINK_TERMINATED_EVENT:
      {
        BLEC_Type_Dev* T;
        BLE_Array_ItemFind_Value(E->DevList, ConnHandle, pEvent->linkTerminate.connectionHandle, T);
        if(T != 0){
          T->ConnStatus = BLEC_ConnStatus_Unknown;
          T->ConnHandle = 0xFFFF;
          E->DevIsConn[T->Token] = false;
          //
          BLE_CmdRetError(BLE_Error_BleTerminate, pEvent->linkTerminate.reason);
          BLE_CmdRetConnStatus(E->DevIsConn, BLEC_Dev_Max_Num); 
        }
      }
      break;
    case GAP_LINK_PARAM_UPDATE_EVENT:
      {
        uint8 Send[] = { 0x01, 0x00 };
        BLE_CmdRetError(
          BLE_Error_BleStatus, 
          SimpleProfile_WriteCharValue(E->TaskId, pEvent->linkUpdate.connectionHandle, SIMPLEPROFILE_CHAR_CS_UUID_WRITE_HANDLE, Send, sizeof(Send))
        );
      }
      break;
    default:
      break;
  }
  return(true);
}

/*********************************************************************
 * @fn      BLEC_Connect
 *
 * @brief  Connect
 *
 * @param   
 *
 * @return  none
 */
void BLEC_Connect()
{
  for(uint8 i = 0; i < BLEC_Dev_Max_Num; i++){
    if(E->DevList[i].ConnStatus == BLEC_ConnStatus_Scanned) {
      E->Connecting = true;
      E->DevList[i].ConnStatus = BLEC_ConnStatus_Connecting;
      BLE_CmdRetError(BLE_Error_BleStatus, GAPCentralRole_EstablishLink(BLEC_LINK_HIGH_DUTY_CYCLE, BLEC_LINK_WHITE_LIST, E->DevList[i].addrType, E->DevList[i].addr));
      break;
    }
  }
}

/*********************************************************************
 * @fn      BLEC_GATTDiscoveryEvent
 *
 * @brief   Process GATT discovery event
 *
 * @return  none
 */
void BLEC_GATTDiscoveryEvent(gattMsgEvent_t *pMsg)
{
  BLEC_Type_Dev* T;
  BLE_Array_ItemFind_Value(E->DevList, ConnHandle, pMsg->connHandle, T);
  if(T != 0){
    if(T->DiscState == BLE_DISC_STATE_SVC)
    {
      // Service found, store handles
      if(pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP && pMsg->msg.findByTypeValueRsp.numInfo > 0) {
        T->SvcStartHdl = ATT_ATTR_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
        T->SvcEndHdl = ATT_GRP_END_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
      }
      // If procedure complete
      if((pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP && pMsg->hdr.status == bleProcedureComplete) ||(pMsg->method == ATT_ERROR_RSP)) {
        if(T->SvcStartHdl != 0){
          // Discover characteristic
          attReadByTypeReq_t req;
          T->DiscState = BLE_DISC_STATE_CHAR_SC;
          req.startHandle = T->SvcStartHdl;
          req.endHandle = T->SvcEndHdl;
          req.type.len = ATT_BT_UUID_SIZE;
          req.type.uuid[0] = LO_UINT16(SIMPLEPROFILE_CHAR_SC_UUID);
          req.type.uuid[1] = HI_UINT16(SIMPLEPROFILE_CHAR_SC_UUID);
          BLE_CmdRetError(BLE_Error_BleStatus, GATT_ReadUsingCharUUID(T->ConnHandle, &req, E->TaskId));
        }
      }
    }
    else if(T->DiscState == BLE_DISC_STATE_CHAR_SC){
      // Characteristic found, store handle
      if(pMsg->method == ATT_READ_BY_TYPE_RSP && pMsg->msg.readByTypeRsp.numPairs > 0){
        T->CharHdSC = BUILD_UINT16(pMsg->msg.readByTypeRsp.pDataList[0], pMsg->msg.readByTypeRsp.pDataList[1]);
        if(P.ENABLE_UPDATE_REQUEST == true) {
          if(SUCCESS != BLE_CmdRetError(BLE_Error_BleStatus, GAPCentralRole_UpdateLink(T->ConnHandle, P.UPDATE_MIN_CONN_INTERVAL, P.UPDATE_MAX_CONN_INTERVAL, P.UPDATE_SLAVE_LATENCY, P.UPDATE_CONN_TIMEOUT))){
            BLE_CmdRetError(BLE_Error_BleStatus, GAPCentralRole_TerminateLink(T->ConnHandle));
          }
        }
      }
      T->DiscState = BLE_DISC_STATE_IDLE;       
    }
  }
}

/*********************************************************************
 * @fn      BLEC_End_Scan_Conn
 *
 * @brief   
 *
 * @return  none
 */
uint32 BLEC_End_Scan_Conn()
{
  if(osal_get_timeoutEx(E->TaskId, BLEC_SCAN_EVT)){
    BLE_CmdRetError(BLE_Error_General, osal_stop_timerEx(E->TaskId, BLEC_SCAN_EVT));
  }
  if(osal_get_timeoutEx(E->TaskId, BLEC_END_SCAN_EVT)){
    BLE_CmdRetError(BLE_Error_General, osal_stop_timerEx(E->TaskId, BLEC_END_SCAN_EVT));
  }
  if(osal_get_timeoutEx(E->TaskId, BLEC_CONNECT_EVT)){
    BLE_CmdRetError(BLE_Error_General, osal_stop_timerEx(E->TaskId, BLEC_CONNECT_EVT));
  }

  bool NeedDelay = FALSE;
  if(E->Connecting == true){
    E->Connecting = FALSE;
    NeedDelay = true;
  }
  if(E->Scanning == true){
    E->Scanning = FALSE;
    BLE_CmdRetError(BLE_Error_BleStatus, GAPCentralRole_CancelDiscovery());
    NeedDelay = true;
  }
  //Clear Connected Device
  for(uint8 i = 0; i < BLEC_Dev_Max_Num; i++){
    if(E->DevList[i].ConnStatus == BLEC_ConnStatus_Connected){
      BLE_CmdRetError(BLE_Error_BleStatus, GAPCentralRole_TerminateLink(E->DevList[i].ConnHandle));
      NeedDelay = true;
    }
    E->DevList[i].ConnStatus = BLEC_ConnStatus_Unknown;
  }
  E->PERIOD_SCAN_DEVICE_COUNT = 0;
  return(NeedDelay == true) ? osal_GetSystemClock() + P.END_ALL_SCAN_CONN_REQ_TIME : 0;
}

/*********************************************************************
 * @fn      BLEC_BleSend
 *
 * @brief   
 *
 * @return  none
 */
bool BLEC_BleSend(uint8 DevToken, uint8* data, uint8 dataLen){
  if((DevToken < BLEC_Dev_Max_Num) &&(dataLen <= SIMPLEPROFILE_CHAR_SC_CS_LEN) &&(E->DevList[DevToken].ConnStatus == BLEC_ConnStatus_Connected)) {
    BLEC_Type_Dev* T = &E->DevList[DevToken];
    if(T->CharSCDoWrite){
      if(BLE_CmdRetError(BLE_Error_BleStatus, SimpleProfile_WriteCharValue(E->TaskId, T->ConnHandle, T->CharHdSC, data, dataLen)) == SUCCESS){
        T->CharSCDoWrite = false;
        return true;
      } else {
        return false;
      }
    } else {
      // BLE is busy, push task into queue
      if(BLE_QueueEn(&T->SendQueue, data, dataLen) == true){
        return true;
      } else {
        // Queue is full
        BLE_CmdRetError(BLE_Error_BleStatus, bleMemAllocError);
        return false;
      }
    }
  } else {
    return false;
  }
}

/*********************************************************************
 * @fn      BLEC_CmdHandle
 *
 * @brief   Handle Uart msg
 *
 * @param   msg - Data, Len - Length of Data
 *
 * @return  none
 */
void BLEC_CmdHandle(uint8* Cmd, uint8 CmdLen){
#define QM  ((Msg_Executed = true) == true)
#define MF  (Msg_Executed == false)
#define MT  (Msg_Executed == true)
  uint8 Role;
  uint8 Type;
  uint8* Ext;
  uint8 ExtLen;
  bool Msg_Executed = false;
  if(BLE_CmdParse(Cmd, CmdLen, &Role, &Type, &Ext, &ExtLen, true) == true){
    uint8 PType0, PType1;
    uint32 PValL;
    uint8* PValArr;
    uint8 PvalArrLen;
    switch(Type){
      case BLE_MsgType_Connect_EnAdvert:
        if(BLE_CmdExtParse(Ext, ExtLen, true) && QM) osal_start_timerEx(E->TaskId, BLEC_SCAN_EVT, BLEC_End_Scan_Conn());
        break;
      case BLE_MsgType_Disconnect_DisAdvert:
        if(BLE_CmdExtParse(Ext, ExtLen, true) && QM) BLEC_End_Scan_Conn();
        break;
      case BLE_MsgType_RoleStatus_Get:
        if(BLE_CmdExtParse(Ext, ExtLen, true) && QM) BLE_CmdRetRoleStatus();
        break;
      case BLE_MsgType_ConnStatus_Get:      //Check Connection 
        if(BLE_CmdExtParse(Ext, ExtLen, true) && QM) BLE_CmdRetConnStatus(E->DevIsConn, BLEC_Dev_Max_Num);
        break;
      case BLE_MsgType_Parms_Set:       //Change Parms
        if(BLE_CmdExtParse_8_8_N(Ext, ExtLen, &PType0, &PType1, &PValArr, &PvalArrLen, true)) {
          switch(PType0){
            case BLEC_SetParm_SCAN_RSP_DATA:
              if(SE(PvalArrLen, BLE_SCAN_RSP_DATA_Max_Len) && Sm(PType1, BLEC_Dev_Max_Num) && QM) BLE_Array_Copy(PS.SCAN_RSP_DATA[PType1], PS.SCAN_RSP_DATA_LEN[PType1] , PValArr, PvalArrLen);
              break;
          }
        }
        if(MF && BLE_CmdExtParse_8_N(Ext, ExtLen, &PType0, &PValArr, &PvalArrLen, true)) {
          switch(PType0){
            case BLEC_SetParm_NAME:
              if(SE(PvalArrLen, BLE_Parm_Dev_Name_Max_Len) && La(PvalArrLen, 0) && QM) BLE_Array_Copy(PS.NAME, PS.NAME_LEN, PValArr, PvalArrLen);
              break;
            case BLEC_SetParm_TRANSMIT_ENCRYPT_KEY:
              if(Eq(PvalArrLen, BLE_TRANSMIT_ENCRYPT_DATA_LEN) && QM) BLE_Array_CopyFL(PS.TRANSMIT_ENCRYPT_KEY, PValArr);
              break;
            case BLEC_SetParm_INFO:
              if(SE(PvalArrLen, BLE_Parm_Dev_Info_Max_Len) && QM) BLE_Array_Copy(PS.INFO, PS.INFO_LEN, PValArr, PvalArrLen);
              break;
          }
        }
        if(MF && BLE_CmdExtParse_8_32(Ext, ExtLen, &PType0, &PValL, true)) {
          switch(PType0){
            case BLEC_SetParm_RESET:
              if(QM) BLEC_Reset_EParms(&PS);
              break;
            case BLEC_SetParm_ENABLE_UPDATE_REQUEST:
              if(Bool(PValL) && QM) PS.ENABLE_UPDATE_REQUEST = PValL;
              break;
            case BLEC_SetParm_UPDATE_MIN_CONN_INTERVAL:
              if(PU16(PValL) && QM) PS.UPDATE_MIN_CONN_INTERVAL = PValL;
              break;
            case BLEC_SetParm_UPDATE_MAX_CONN_INTERVAL:
              if(PU16(PValL) && QM) PS.UPDATE_MAX_CONN_INTERVAL = PValL;
              break;
            case BLEC_SetParm_UPDATE_SLAVE_LATENCY:
              if(U16(PValL) && QM) PS.UPDATE_SLAVE_LATENCY = PValL;
              break;
            case BLEC_SetParm_UPDATE_CONN_TIMEOUT:
              if(PU16(PValL) && QM) PS.UPDATE_CONN_TIMEOUT = PValL;
              break;
            case BLEC_SetParm_SCAN_RSSI_THRESHOLD:
              if(Sm(PValL, 0xFF / 2) && QM) PS.SCAN_RSSI_THRESHOLD =(-1) * PValL;
              break;
            case BLEC_SetParm_SCAN_DURATION:
              if(PU16(PValL) && QM) PS.SCAN_DURATION = PValL;
              break;
            case BLEC_SetParm_SCAN_TOTAL_DURATION:
              if(PU16(PValL) && QM) PS.SCAN_TOTAL_DURATION = PValL;
              break;
            case BLEC_SetParm_MULTI_CONNECT_INTERVAL:
              if(PU16(PValL) && QM) PS.MULTI_CONNECT_INTERVAL = PValL;
              break;
            case BLEC_SetParm_PERIOD_SCAN_DEVICE_MAX_NUM:
              if(PU8(PValL) && QM) PS.PERIOD_SCAN_DEVICE_MAX_NUM = PValL;
              break;  
            case BLEC_SetParm_PERIOD_SCAN_DEVICE_OVERFLOW_COUNT:
              if(PU8(PValL) && QM) PS.PERIOD_SCAN_DEVICE_OVERFLOW_COUNT = PValL;
              break;
            case BLEC_SetParm_END_ALL_SCAN_CONN_REQ_TIME:
              if(QM) PS.END_ALL_SCAN_CONN_REQ_TIME = PValL;
              break;
            case BLEC_SetParm_PAIR_MODE:
              if(Sm(PValL, BLE_PairMode_Num) && QM) PS.PAIR_MODE =(BLE_Type_PairMode)PValL;
              break;
            case BLEC_SetParm_PASSCODE:
              if(SE(PValL, 999999) && QM) PS.PASSCODE = PValL;
              break;
            case BLEC_SetParm_ENABLE_TRANSMIT_ENCRYPT:
              if(Bool(PValL) && QM) PS.ENABLE_TRANSMIT_ENCRYPT = PValL;
              break;
            case BLEC_SetParm_WATCHDOG:
              if(SE(PValL, HAL_SYSTEM_WD_MODE_NUM) && QM) PS.WATCHDOG_MODE = PValL;
              break;
            case BLEC_SetParm_ENABLE_CMD_CHECK_BIT:
              if(Bool(PValL) && QM) PS.ENABLE_CMD_CHECK_BIT = PValL;
              break;
          }
        }
        if(MT){
          E->EParms_Save_Updated = true;
          if(osal_get_timeoutEx(E->TaskId, BLEC_PARM_SAVE_EVT) == 0){
            BLE_CmdRetError(BLE_Error_General, osal_start_timerEx(E->TaskId, BLEC_PARM_SAVE_EVT, BLEC_PARM_SAVE_DELAY));
          }
        }
        break;
      case BLE_MsgType_Hal_Set:
        if(BLE_CmdExtParse_8_8(Ext, ExtLen, &PType0, &PType1, true)) {
          switch(PType0){
            case BLE_AddOper_ReadRssi:
              if(Sm(PType1, BLEC_Dev_Max_Num) && Eq(E->DevList[PType1].ConnStatus, BLEC_ConnStatus_Connected) && QM) BLE_CmdRetError(BLE_Error_Hci, HCI_ReadRssiCmd(E->DevList[PType1].ConnHandle));
              break;
            case BLE_AddOper_ReadMac:
              if(Eq(PType1, BLE_OWN_CONN_HANDLE) && QM){
                BLE_CmdRetAddInfo(BLE_AddRet_ReadMac, PType1, E->Addr, BLEC_ADDR_LEN);
              } else if(Sm(PType1, BLEC_Dev_Max_Num) && Eq(E->DevList[PType1].ConnStatus, BLEC_ConnStatus_Connected) && QM){
                BLE_CmdRetAddInfo(BLE_AddRet_ReadMac, PType1, E->DevList[PType1].addr, BLEC_ADDR_LEN);
              }
              break;
          }
        }
        break;
      case BLE_MsgType_Send:
        if(BLE_CmdExtParse_8_N(Ext, ExtLen, &PType0, &PValArr, &PvalArrLen, true) && QM) {
          if(BLE_SendBLE(PType0, PValArr, PvalArrLen) == false){
            BLE_CmdRetTransmitDone(PType0, false);
          }
        }
        break;
      case BLE_MsgType_Reboot: //Reboot
        if(BLE_CmdExtParse(Ext, ExtLen, true) && QM) BLE_CmdRetError(BLE_Error_General, osal_start_timerEx(E->TaskId, BLEC_RESET_EVT, BLEC_End_Scan_Conn()));
        break;
#if defined(BLE_PERIPHERAL)
      case BLE_MsgType_SwitchRole:
        if(BLE_CmdExtParse(Ext, ExtLen, true) && QM) {
          E->BLE_CommFunc->SwitchRole();
          BLE_CmdRetError(BLE_Error_General, osal_start_timerEx(E->TaskId, BLEC_RESET_EVT, BLEC_End_Scan_Conn()));
        }
        break;
#endif
      case BLE_MsgType_Sleep:
        if(BLE_CmdExtParse(Ext, ExtLen, true) && QM) BLE_CmdRetError(BLE_Error_General, osal_start_timerEx(E->TaskId, BLEC_Sleep_EVT, BLEC_End_Scan_Conn()));
        break;
    }
  }
  if(MF) {
    BLE_CmdRetError(BLE_Error_CmdParse, FAILURE);
  } else {
    BLE_CmdRetCmdParsed();
  }
}

/*********************************************************************
 * @fn      BLEC_Main_Loop
 *
 * @brief   Main Loop
 *
 * @param   ......
 *
 * @return  none
 */
void BLEC_Main_Loop()
{
  WD_KICK();
}
/*********************************************************************
*********************************************************************/
#endif