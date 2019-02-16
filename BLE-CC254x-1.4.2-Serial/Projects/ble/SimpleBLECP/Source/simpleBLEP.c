/**************************************************************************************************
  Filename:       simpleBLEPeripheral.c
  Revised:        $Date: 2010-08-06 08:56:11 -0700(Fri, 06 Aug 2010) $
  Revision:       $Revision: 23333 $

  Description:    This file contains the Simple BLE Peripheral sample application
                  for use with the CC2540 Bluetooth Low Energy Protocol Stack.

  Copyright 2010 - 2013 Texas Instruments Incorporated. All rights reserved.

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
**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */

#if defined(BLE_PERIPHERAL)

#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "OnBoard.h"
#include "gatt.h"
#include "OSAL_snv.h"
#include "hci.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"
#include "simpleGATTprofile.h"
#if defined(PLUS_BROADCASTER)
  #include "peripheralBroadcaster.h"
#else
  #include "peripheral.h"
#endif
#include "gapbondmgr.h"
#include "simpleBLE.h"
#include "simpleBLEP.h"
#include "npi.h"
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
#define E      BLEP_GVar
#define PS     BLEP_GVar->EParms_Save
#define P      BLEP_GVar->EParms
   
#define BLEP_ADDR_LEN B_ADDR_LEN

#define Bool(_V_) (_V_ == 0 || _V_ == 1)
#define PU16(_V_) (0 < _V_ && _V_ <= 0xFFFF)
#define U16(_V_) (_V_ <= 0xFFFF)
#define PU8(_V_) (0 < _V_ && _V_ <= 0xFF)
#define U8(_V_) (_V_ <= 0xFF)
#define PU32(_V_) (0 < _V_ && _V_ <= 0xFFFFFFFF)
#define U32(_V_) (_V_ <= 0xFFFFFFFF)
#define Sm(_V0_, _V1_) (_V0_ < _V1_)
#define SE(_V0_, _V1_) (_V0_ <= _V1_)
#define La(_V0_, _V1_) (_V0_ > _V1_)
#define LE(_V0_, _V1_) (_V0_ >= _V1_)
#define Eq(_V0_, _V1_) (_V0_ == _V1_)

uint16 BLEP_Timer_Record = 0;
#define Timeout(_Token, _Method)  (BLEP_Timer_Record & _Token) 
                   
#define BLEP_DISCOVERABLE_MODE                  GAP_ADTYPE_FLAGS_GENERAL     // Limited discoverable mode advertises for 30.72s, and then stops, General discoverable mode advertises indefinitely
#if defined(PLUS_BROADCASTER)
  #define BLEP_ADV_IN_CONN_WAIT                    500                          // delay 500 ms
  #define BLEP_SBP_ADV_IN_CONNECTION_EVT           0x0004
#endif

#define BLEP_SBP_START_DEVICE_EVT                              (0x0001 << 0)             // Simple BLE Peripheral Task Events
#define BLEP_RESET_EVT                                         (0x0001 << 1)             // Reset Timer
#define BLEP_SEND_DONE_EVT                                     (0x0001 << 2)             // Send Done Timer
#define BLEP_Sleep_EVT                                         (0x0001 << 3)             // Sleep Timer
#define BLEP_PARM_SAVE_EVT                                     (0x0001 << 4)             // Save Parameter

#define BLEP_PARM_SAVE_DELAY                                   200
/*********************************************************************
* TYPEDEFS
*/

typedef struct{
  BLE_Type_CommFunc* BLE_CommFunc;
  
  BLEP_Type_EParms EParms;
  BLEP_Type_EParms EParms_Save;
  bool EParms_Save_Updated;
  uint8 TaskId;                                                                 // Task ID for internal task/event processing
  // GAP - Advertisement data(max size = 31 bytes, though this is
  // best kept short to conserve power while advertisting)
  uint8 advertData[BLE_SCAN_RSP_DATA_Max_Len];
  uint8 advertDataLen;
  uint8 centralAddr[BLEP_ADDR_LEN];
  uint8 addr[BLEP_ADDR_LEN];
  //bool Char7DoWrite;
  bool Connected;
  bool SendDone;
  BLE_Type_Queue SendQueue;
  //BLE Buffer
  uint8 ConnHandle;
  bool AdverEnable;
  gapRolesCBs_t PeripheralCBs;   // GAP Role Callbacks
  gapBondCBs_t BondMgrCBs; // GAP Bond Manager Callbacks
  simpleProfileCBs_t SimpleProfileCBs; // Simple GATT Profile Callbacks
  gaprole_States_t gapProfileState;
} BLEP_Type_GlobalVar;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
void BLEP_WakeupNotify();
void BLEP_Reset_EParms(BLEP_Type_EParms*);
void BLEP_Init_GlobalVar();
void BLEP_ProcessOSALMsg(osal_event_hdr_t *);
void BLEP_StateNotificationCB(gaprole_States_t);
void BLEP_ProfileChangeCB(uint8);

void BLEP_CmdHandle(uint8*, uint8);

uint32 BLEP_End_Conn(bool);
void BLEP_Set_AdvertEnable(bool);
void BLEP_CentralPasscodeCB(uint8*, uint16, uint8, uint8);
void BLEP_CentralPairStateCB(uint16, uint8, uint8);
void BLEP_RssiCB(int8);
void BLEP_ProcessGATTMsg(gattMsgEvent_t *);
bool BLEP_BleSend(uint8, uint8*, uint8);
void BLEP_Main_Loop();
/*********************************************************************
 * LOCAL Variables
 */
BLEP_Type_GlobalVar* BLEP_GVar = 0;

/*********************************************************************
 * PUBLIC FUNCTIONS
 */
   
/*********************************************************************
 * @fn      BLEP_WakeupNotify
 *
 * @brief   
 * @param   
 *
 f @return  none
 */
void BLEP_WakeupNotify(){
}

/*********************************************************************
 * @fn      BLEC_Reset_EParms
 *
 * @brief   
 * @param   
 *
 f @return  none
 */
void BLEP_Reset_EParms(BLEP_Type_EParms* EParms){
  EParms->INTER_VERSION = BLE_Parm_INTER_VERSION;
  EParms->ADVERTISING_INTERVAL = 160;
  EParms->ENABLE_DESIRED_REQUEST = TRUE;
  EParms->DESIRED_MIN_CONN_INTERVAL = 32;
  EParms->DESIRED_MAX_CONN_INTERVAL = 32;
  EParms->DESIRED_SLAVE_LATENCY = 2;
  EParms->DESIRED_CONN_TIMEOUT  = 200;
  EParms->CONN_PAUSE_PERIPHERAL = 2;
  EParms->POWER_LEVEL = LL_EXT_TX_POWER_0_DBM;
  EParms->END_ALL_SCAN_CONN_REQ_TIME = 150;  
  BLE_Array_SetStr(EParms->NAME, EParms->NAME_LEN, "BLEP");
  EParms->PASSCODE = 123456;
  EParms->PAIR_MODE = BLE_PairMode_Passcode_Initiate;
  uint8 COMPANY_ID[] = { HI_UINT16(BLE_TI_COMPANY_ID), LO_UINT16(BLE_TI_COMPANY_ID)};
  BLE_Type_ScanAdvertData SCAN_DATA[] = {
    {GAP_ADTYPE_MANUFACTURER_SPECIFIC, COMPANY_ID, BLE_Array_Len(COMPANY_ID)}
  };
  BLE_ScanAdvertData_Construct(SCAN_DATA, BLE_Array_Len(SCAN_DATA), EParms->SCAN_RSP_DATA, &EParms->SCAN_RSP_DATA_LEN);
  EParms->INFO_LEN = 0;
  EParms->AUTO_ADVERT = true;
  EParms->SEND_DONE_DELAY = 35;
  EParms->ENABLE_TRANSMIT_ENCRYPT = false;
  osal_memset(EParms->TRANSMIT_ENCRYPT_KEY, 0x00, BLE_TRANSMIT_ENCRYPT_DATA_LEN);
  EParms->WATCHDOG_MODE = HAL_SYSTEM_WD_MODE_DISABLE;
  EParms->ENABLE_CMD_CHECK_BIT = false;
}

void BLEP_Init_GlobalVar(){
  
  E =(BLEP_Type_GlobalVar*)osal_mem_alloc(sizeof(BLEP_Type_GlobalVar));
  // Others
  E->ConnHandle = 0x00;
  E->advertDataLen = 0;         // GAP - Advertisement data(max size = 31 bytes, though this is
  E->EParms_Save_Updated = false;
  E->AdverEnable = false;
  E->TaskId = 0;
  E->Connected = false;
  E->SendDone = true;
  //System
  E->PeripheralCBs.pfnStateChange = BLEP_StateNotificationCB;
  E->PeripheralCBs.pfnRssiRead = BLEP_RssiCB;

  E->BondMgrCBs.passcodeCB = BLEP_CentralPasscodeCB;
  E->BondMgrCBs.pairStateCB = BLEP_CentralPairStateCB;
  
  E->SimpleProfileCBs.pfnSimpleProfileChange = BLEP_ProfileChangeCB;
  
  E->gapProfileState = GAPROLE_INIT;
}

/*********************************************************************
 * @fn      BLEP_Set_Advert
 *
 * @brief   
 *
 * @param   
 *
 * @return  none
 */
void BLEP_Set_AdvertEnable(bool IsEnable){
  if(E->AdverEnable != IsEnable){
    E->AdverEnable = IsEnable;
    BLE_CmdRetError(BLE_Error_General, GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(E->AdverEnable), &E->AdverEnable));
  }
}


/*********************************************************************
 * @fn      BLEP_Init
 *
 * @brief   Initialization function for the Simple BLE Peripheral App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization(ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ...).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void BLEP_Init(uint8 task_id , BLE_Type_CommFunc* BLE_CommFunc)
{
  BLEP_Init_GlobalVar();
  osal_set_loop_func(BLEP_Main_Loop);
  E->BLE_CommFunc = BLE_CommFunc;
  E->BLE_CommFunc->CmdReceive = BLE_CmdGetFromExter;
  E->BLE_CommFunc->WakeupNotify = BLEP_WakeupNotify;
  E->TaskId = task_id;
  
  //Read Parms from flash
  if(osal_snv_read(BLEP_Parms_Flash_Idx, sizeof(P),(uint8*)&P) != SUCCESS ||
  P.INTER_VERSION != BLE_Parm_INTER_VERSION){      //Check whether data is valid
    BLEP_Reset_EParms(&P);
  }
  PS = P;
  E->AdverEnable = P.AUTO_ADVERT;
  
  BLE_Type_Argv BLE_Argv = {
    .Role = BLE_CMD_Msg_RelDev_P,
    .Info = P.INFO,
    .InfoLen = P.INFO_LEN,
    .EnCmdCheckBit= P.ENABLE_CMD_CHECK_BIT,
    .EnTranEncry = P.ENABLE_TRANSMIT_ENCRYPT,
    .TranEncryKey = P.TRANSMIT_ENCRYPT_KEY,
    .CmdSend = BLE_CommFunc->CmdSend,
    .BleSend = BLEP_BleSend,
    .CmdRec = BLEP_CmdHandle
  };
  BLE_Init(&BLE_Argv);
  
  // Setup the GAP
  BLE_CmdRetError(BLE_Error_BleStatus, GAP_SetParamValue(TGAP_CONN_PAUSE_PERIPHERAL, P.CONN_PAUSE_PERIPHERAL));
  
  // Setup the GAP Peripheral Role Profile
  {
    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    // Set the GAP Role Parameters
    uint16 gapRole_AdvertOffTime = 0;
    BLE_CmdRetError(BLE_Error_BleStatus, GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(gapRole_AdvertOffTime), &gapRole_AdvertOffTime));

    BLE_CmdRetError(BLE_Error_BleStatus, GAPRole_SetParameter(GAPROLE_PARAM_UPDATE_ENABLE, sizeof(P.ENABLE_DESIRED_REQUEST), &P.ENABLE_DESIRED_REQUEST));
    BLE_CmdRetError(BLE_Error_BleStatus, GAPRole_SetParameter(GAPROLE_MIN_CONN_INTERVAL, sizeof(P.DESIRED_MIN_CONN_INTERVAL), &P.DESIRED_MIN_CONN_INTERVAL));
    BLE_CmdRetError(BLE_Error_BleStatus, GAPRole_SetParameter(GAPROLE_MAX_CONN_INTERVAL, sizeof(P.DESIRED_MAX_CONN_INTERVAL), &P.DESIRED_MAX_CONN_INTERVAL));
    BLE_CmdRetError(BLE_Error_BleStatus, GAPRole_SetParameter(GAPROLE_SLAVE_LATENCY, sizeof(P.DESIRED_SLAVE_LATENCY), &P.DESIRED_SLAVE_LATENCY));
    BLE_CmdRetError(BLE_Error_BleStatus, GAPRole_SetParameter(GAPROLE_TIMEOUT_MULTIPLIER, sizeof(P.DESIRED_CONN_TIMEOUT), &P.DESIRED_CONN_TIMEOUT));
  }

  // Set the GAP Characteristics
  BLE_CmdRetError(BLE_Error_BleStatus, GGS_SetParameter(GGS_DEVICE_NAME_ATT, P.NAME_LEN, P.NAME));

  // Set advertising interval
  {
    uint16 advInt = P.ADVERTISING_INTERVAL;
    BLE_CmdRetError(BLE_Error_BleStatus, GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, advInt));
    BLE_CmdRetError(BLE_Error_BleStatus, GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, advInt));
    BLE_CmdRetError(BLE_Error_BleStatus, GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, advInt));
    BLE_CmdRetError(BLE_Error_BleStatus, GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, advInt));
  }

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
    uint8 ioCap = 
     (P.PAIR_MODE == BLE_PairMode_Passcode_Initiate || P.PAIR_MODE == BLE_PairMode_Initiate)?
      GAPBOND_IO_CAP_DISPLAY_ONLY:
      GAPBOND_IO_CAP_KEYBOARD_ONLY;
    uint8 bonding = 
     (P.PAIR_MODE == BLE_PairMode_Passcode_Initiate || P.PAIR_MODE == BLE_PairMode_Initiate)?
      false:
      true;
      
    BLE_CmdRetError(BLE_Error_BleStatus, GAPBondMgr_SetParameter(GAPBOND_DEFAULT_PASSCODE, sizeof(passkey), &passkey));
    BLE_CmdRetError(BLE_Error_BleStatus, GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(pairMode), &pairMode));
    BLE_CmdRetError(BLE_Error_BleStatus, GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(mitm), &mitm));
    BLE_CmdRetError(BLE_Error_BleStatus, GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(ioCap), &ioCap));
    BLE_CmdRetError(BLE_Error_BleStatus, GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(bonding), &bonding));
  }

  // Initialize GATT attributes
  BLE_CmdRetError(BLE_Error_BleStatus, GGS_AddService(GATT_ALL_SERVICES));            // GAP
  BLE_CmdRetError(BLE_Error_BleStatus, GATTServApp_AddService(GATT_ALL_SERVICES));    // GATT attributes
  BLE_CmdRetError(BLE_Error_BleStatus, DevInfo_AddService());                           // Device Information Service
  BLE_CmdRetError(BLE_Error_BleStatus, SimpleProfile_AddService(GATT_ALL_SERVICES));  // Simple GATT Profile
  
#if defined FEATURE_OAD
  VOID OADTarget_AddService();
#endif

  
  // Setup the SimpleProfile Characteristic Values
  // Register callback with SimpleGATTprofile
  BLE_CmdRetError(BLE_Error_BleStatus, SimpleProfile_RegisterAppCBs(&E->SimpleProfileCBs));

  BLE_CmdRetError(BLE_Error_General, osal_set_event(E->TaskId, BLEP_SBP_START_DEVICE_EVT));

  BLE_CmdRetError(BLE_Error_Hci, HCI_EXT_ClkDivOnHaltCmd(LL_EXT_DISABLE_CLK_DIVIDE_ON_HALT));
  
  BLE_CmdRetError(BLE_Error_Hci, HCI_EXT_SetTxPowerCmd(P.POWER_LEVEL));         //Set POWER
  
  HAL_SYSTEM_SET_WATCHDOG(P.WATCHDOG_MODE);
}

/*********************************************************************
 * @fn      BLEP_RssiCB
 *
 * @brief   RSSI callback.
 *
 * @param   rssi - RSSI
 *
 * @return  none
 */
void BLEP_RssiCB(int8 rssi)
{
  if(E->Connected){
    uint8 rssiPos = -1 * rssi;
    BLE_CmdRetAddInfo(BLE_AddRet_ReadRssi, E->ConnHandle, &rssiPos, 1);
  }
}
/*********************************************************************
 * @fn      BLEP_ProcessEvent
 *
 * @brief   Simple BLE Peripheral Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 BLEP_ProcessEvent(uint8 task_id, uint16 events)
{
  VOID task_id; // OSAL required parameter that isn't used in this function

  if(events & SYS_EVENT_MSG)
  {
    uint8 *pMsg;

    if((pMsg = osal_msg_receive(E->TaskId)) != NULL)
    {
      BLEP_ProcessOSALMsg((osal_event_hdr_t *)pMsg);
      // Release the OSAL message
      osal_msg_deallocate(pMsg);
    }

    // return unprocessed events
    return(events ^ SYS_EVENT_MSG);
  }

  if(events & BLEP_SBP_START_DEVICE_EVT)
  {
    // Start the Device
    BLE_CmdRetError(BLE_Error_BleStatus, GAPRole_StartDevice(&E->PeripheralCBs));

    // Start Bond Manager
    GAPBondMgr_Register(&E->BondMgrCBs);

    return(events ^ BLEP_SBP_START_DEVICE_EVT);
  }
  // *********************** Handle Own defined Timer ********************************* //
  if(events & BLEP_RESET_EVT) {
    BLEP_ProcessEvent(task_id, BLEP_PARM_SAVE_EVT);
    SystemResetSoft();
    return(events ^ BLEP_RESET_EVT);
  }
  if(events & BLEP_PARM_SAVE_EVT)
  {
    if(E->EParms_Save_Updated == true) {
      BLE_CmdRetError(BLE_Error_General, osal_snv_write(BLEP_Parms_Flash_Idx, sizeof(PS),(uint8*)&PS));
      E->EParms_Save_Updated = false;
    }
    return(events ^ BLEP_PARM_SAVE_EVT);
  }
  if(events & BLEP_SEND_DONE_EVT) {
    E->SendDone = true;
    BLE_CmdRetTransmitDone(E->ConnHandle, true);
    uint8* NewSend;
    uint8 NewSendLen;
    while(BLE_QueueDe(&E->SendQueue, &NewSend, &NewSendLen)){
      if(BLEP_BleSend(E->ConnHandle, NewSend, NewSendLen) == false){
        BLE_CmdRetTransmitDone(E->ConnHandle, false);
      } else {
        break;
      }
    }
    return(events ^ BLEP_SEND_DONE_EVT);
  }
  if(events & BLEP_Sleep_EVT) {
    E->BLE_CommFunc->ForceSleep();
    return(events ^ BLEP_Sleep_EVT);
  }
  // ****************************************************************************** //
  
#if defined(PLUS_BROADCASTER)
  if(events & BLEP_SBP_ADV_IN_CONNECTION_EVT)
  {
    uint8 turnOnAdv = TRUE;
    // Turn on advertising while in a connection
    BLE_CmdRetError(BLE_Error_BleStatus, GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(turnOnAdv), &turnOnAdv));

    return(events ^ BLEP_SBP_ADV_IN_CONNECTION_EVT);
  }
#endif // PLUS_BROADCASTER

  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      simpleBLEPeripheral_ProcessGATTMsg
 *
 * @brief   Process GATT messages
 *
 * @return  none
 */
void BLEP_ProcessGATTMsg(gattMsgEvent_t *pMsg)
{
  GATT_bm_free(&pMsg->msg, pMsg->method);
}

/*********************************************************************
 * @fn      BLEP_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
void BLEP_ProcessOSALMsg(osal_event_hdr_t *pMsg)
{
  switch(pMsg->event) {
    case GATT_MSG_EVENT:
      // Process GATT message
      BLEP_ProcessGATTMsg((gattMsgEvent_t *)pMsg);
      break;
    default:
      // do nothing
      break;
  }
}

/*********************************************************************
 * @fn      BLEP_StateNotificationCB
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
void BLEP_StateNotificationCB(gaprole_States_t newState)
{
  uint8 ConnectDisConnect = 0;
  switch(newState)
  {
    case GAPROLE_STARTED:
      {
        uint8 TA[BLE_SCAN_RSP_DATA_Max_Len];   
        uint8 TAL = 0;
        BLE_CmdRetError(BLE_Error_BleStatus, GAPRole_GetParameter(GAPROLE_BD_ADDR, E->addr));
        // use 6 bytes of device address for 8 bytes of system ID value
        TA[0] = E->addr[0];
        TA[1] = E->addr[1];
        TA[2] = E->addr[2];
        // set middle bytes to zero
        TA[4] = 0x00;
        TA[3] = 0x00;
        // shift three bytes up
        TA[7] = E->addr[5];
        TA[6] = E->addr[4];
        TA[5] = E->addr[3];
        BLE_CmdRetError(BLE_Error_BleStatus, DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, TA));
        
        //Init SCAN RSP data
        BLE_CmdRetError(BLE_Error_BleStatus, GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, P.SCAN_RSP_DATA_LEN, P.SCAN_RSP_DATA));
        //Init ADV RSP data
        uint8 FLAGS[] = { BLEP_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED };
        uint8 _16BIT[] = { LO_UINT16(SIMPLEPROFILE_SERV_UUID), HI_UINT16(SIMPLEPROFILE_SERV_UUID)};
        BLE_Type_ScanAdvertData ADVERT_DATA[] = {
          {GAP_ADTYPE_FLAGS, FLAGS, BLE_Array_Len(FLAGS)},
          {GAP_ADTYPE_16BIT_COMPLETE, _16BIT, BLE_Array_Len(_16BIT) }
        };
        BLE_ScanAdvertData_Construct(ADVERT_DATA, BLE_Array_Len(ADVERT_DATA), TA, &TAL);
        BLE_CmdRetError(BLE_Error_BleStatus, GAPRole_SetParameter(GAPROLE_ADVERT_DATA, TAL, TA));
        
        BLE_CmdRetError(BLE_Error_BleStatus, GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(E->AdverEnable), &E->AdverEnable));
        BLE_CmdRetRoleStatus();
      }
      break;
    case GAPROLE_ADVERTISING:
      {
      }
      break;
    case GAPROLE_CONNECTED:
      {     
        ConnectDisConnect = 1;
      }
      break;
    case GAPROLE_WAITING:
      {
        ConnectDisConnect = 2;
      }
      break;
    case GAPROLE_WAITING_AFTER_TIMEOUT:
      {
        ConnectDisConnect = 2;
      }
      break;
    case GAPROLE_ERROR:
      {
        BLE_CmdRetError(BLE_Error_BleTerminate, newState);
      }
      break;
    default:
      {
      }
      break;
  }
  if(ConnectDisConnect == 1){
    if(E->Connected == false){
      E->BLE_CommFunc->ForceWakeup();
      E->Connected = true;
      E->SendQueue.Len = 0;
      E->SendDone = true;
      BLE_CmdRetError(BLE_Error_BleStatus, GAPRole_GetParameter(GAPROLE_CONN_BD_ADDR, E->centralAddr));
      BLE_CmdRetConnStatus(&E->Connected, 1); 
    }
  } else if(ConnectDisConnect == 2){
    if(E->Connected == true){
      E->Connected = false;
      BLE_CmdRetError(BLE_Error_BleTerminate, newState);
      if(E->SendDone == false){
        BLE_CmdRetError(BLE_Error_General, osal_stop_timerEx(E->TaskId, BLEP_SEND_DONE_EVT));
        E->SendDone = true;
      }
      BLE_CmdRetConnStatus(&E->Connected, 1); 
    }
  }
  E->gapProfileState = newState;
}

/*********************************************************************
 * @fn      BLEP_ProfileChangeCB
 *
 * @brief   Callback from SimpleBLEProfile indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
void BLEP_ProfileChangeCB(uint8 paramID)
{
  switch(paramID)
  {
    case SIMPLEPROFILE_CHAR_SC:
      {
        uint8 *pValue = 0, *len = 0;
        if(BLE_CmdRetError(BLE_Error_BleStatus, SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR_SC, &pValue, &len)) == SUCCESS){
          BLE_BleRetToExter(E->ConnHandle, pValue, *len);
        }
      }
      break;
    default:
      // should not reach here!
      break;
  }
}

/*********************************************************************
 * @fn      pairStateCB
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
void BLEP_CentralPairStateCB(uint16 connHandle, uint8 state, uint8 status){
  if(state == GAPBOND_PAIRING_STATE_STARTED) {
    //
  } else if(state == GAPBOND_PAIRING_STATE_COMPLETE) {
    if(status == SUCCESS || status == SMP_PAIRING_FAILED_UNSPECIFIED) {
      //Paired
    } else {
      //UnPaired
      BLE_CmdRetError(BLE_Error_General, GAPRole_TerminateConnection());
    }
  } else if(state == GAPBOND_PAIRING_STATE_BONDED) {
    if(status == SUCCESS){
      //LCD_WRITE_STRING("Bonding success", HAL_LCD_LINE_1);
    } else {
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
void BLEP_CentralPasscodeCB(uint8 *deviceAddr, uint16 connectionHandle, uint8 uiInputs, uint8 uiOutputs){
  BLE_CmdRetError(BLE_Error_BleStatus, GAPBondMgr_PasscodeRsp(connectionHandle, SUCCESS, P.PASSCODE));
}

/*********************************************************************
 * @fn      BLEP_End_Conn
 *
 * @brief   
 *
 * @param   
 *
 * @return  none
 */
uint32 BLEP_End_Conn(bool Advert) {
  if(E->SendDone == false){
    BLE_CmdRetError(BLE_Error_General, osal_stop_timerEx(E->TaskId, BLEP_SEND_DONE_EVT));
    E->SendDone = true;
  }
  if(E->Connected == true){
    BLE_CmdRetError(BLE_Error_BleStatus, GAPRole_TerminateConnection());
  }
  if(Advert != E->AdverEnable){
    BLEP_Set_AdvertEnable(Advert);
  }
  return P.END_ALL_SCAN_CONN_REQ_TIME;
}

/*********************************************************************
 * @fn      BLEP_BleSend
 *
 * @brief   
 *
 * @return  none
 */
bool BLEP_BleSend(uint8 connHandle, uint8* data, uint8 dataLen){
  if(connHandle == E->ConnHandle && E->Connected) {
    if(E->SendDone){
      if(BLE_CmdRetError(BLE_Error_BleStatus, SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR_CS, data, dataLen)) == SUCCESS){
        E->SendDone = false;
        BLE_CmdRetError(BLE_Error_General, osal_start_timerEx(E->TaskId, BLEP_SEND_DONE_EVT, PS.SEND_DONE_DELAY));
        return true;
      } else {
        return false;
      }
    } else {
      // BLE is busy, push task into queue
      if(BLE_QueueEn(&E->SendQueue, data, dataLen) == true){
        return true;
      } else {
        BLE_CmdRetError(BLE_Error_BleStatus, bleMemAllocError);
        // Queue is full
        return false;
      }
    }
  } else {
    return false;
  }
}
/*********************************************************************
 * @fn      BLEP_CmdHandle
 *
 * @brief   Handle Exter msg
 *
 * @param   msg - Data, Len - Length of Data
 *
 * @return  none
 */
void BLEP_CmdHandle(uint8* Cmd, uint8 CmdLen){
#define QM   ((Msg_Executed = true) == true)
#define MF   (Msg_Executed == false)
#define MT   (Msg_Executed == true)
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
        if(BLE_CmdExtParse(Ext, ExtLen, true) && QM) BLEP_End_Conn(true);
        break;
      case BLE_MsgType_Disconnect_DisAdvert:
        if(BLE_CmdExtParse(Ext, ExtLen, true) && QM) BLEP_End_Conn(false);
        break;
      case BLE_MsgType_RoleStatus_Get:
        if(BLE_CmdExtParse(Ext, ExtLen, true) && QM) BLE_CmdRetRoleStatus();
        break;
      case BLE_MsgType_ConnStatus_Get:
        if(BLE_CmdExtParse(Ext, ExtLen, true) && QM) BLE_CmdRetConnStatus(&E->Connected, 1);
        break;
      case BLE_MsgType_Parms_Set:       //Change Parms
        {
          if(BLE_CmdExtParse_8_N(Ext, ExtLen, &PType0, &PValArr, &PvalArrLen, true)){
            switch(PType0){
              case BLEP_SetParm_NAME:
                if(SE(PvalArrLen, BLE_Parm_Dev_Name_Max_Len) && La(PvalArrLen, 0) && QM) BLE_Array_Copy(PS.NAME, PS.NAME_LEN , PValArr, PvalArrLen);
                break;
              case BLEP_SetParm_SCAN_RSP_DATA:
                if(SE(PvalArrLen, BLE_SCAN_RSP_DATA_Max_Len) && La(PvalArrLen, 0) && QM) BLE_Array_Copy(PS.SCAN_RSP_DATA, PS.SCAN_RSP_DATA_LEN , PValArr, PvalArrLen);
                break;
              case BLEP_SetParm_TRANSMIT_ENCRYPT_KEY:
                if(Eq(PvalArrLen, BLE_TRANSMIT_ENCRYPT_DATA_LEN) && QM) BLE_Array_CopyFL(PS.TRANSMIT_ENCRYPT_KEY, PValArr);
                break;
               case BLEP_SetParm_INFO:
                if(SE(PvalArrLen, BLE_Parm_Dev_Info_Max_Len) && QM) BLE_Array_Copy(PS.INFO, PS.INFO_LEN, PValArr, PvalArrLen);
                break;
            }
          }
          if(MF && BLE_CmdExtParse_8_32(Ext, ExtLen, &PType0, &PValL, true)) {
            switch(PType0){
              case BLEP_SetParm_RESET:
                if(QM) BLEP_Reset_EParms(&PS);
                break;
              case BLEP_SetParm_ADVERTISING_INTERVAL:
                if(PU16(PValL) && QM) PS.ADVERTISING_INTERVAL = PValL;
                break;
              case BLEP_SetParm_ENABLE_DESIRED_REQUEST:
                if(Bool(PValL) && QM) PS.ENABLE_DESIRED_REQUEST = PValL;
                break;
              case BLEP_SetParm_DESIRED_MIN_CONN_INTERVAL:
                if(PU16(PValL) && QM) PS.DESIRED_MIN_CONN_INTERVAL = PValL;
                break;
              case BLEP_SetParm_DESIRED_MAX_CONN_INTERVAL:
                if(PU16(PValL) && QM) PS.DESIRED_MAX_CONN_INTERVAL = PValL;
                break;
              case BLEP_SetParm_DESIRED_SLAVE_LATENCY:
                if(U16(PValL) && QM) PS.DESIRED_SLAVE_LATENCY = PValL;
                break;
              case BLEP_SetParm_DESIRED_CONN_TIMEOUT:
                if(PU16(PValL) && QM) PS.DESIRED_CONN_TIMEOUT = PValL;
                break;
              case BLEP_SetParm_CONN_PAUSE_PERIPHERAL:
                if(PU16(PValL) && QM) PS.CONN_PAUSE_PERIPHERAL = PValL;
                break;
              case BLEP_SetParm_POWER_LEVEL:
                if(Sm(PValL, LL_EXT_TX_POWER_NUM) && QM) 
                  PS.POWER_LEVEL =(BLE_Type_PairMode)PValL;
                break;
              case BLEP_SetParm_END_ALL_SCAN_CONN_REQ_TIME:
                if(PU16(PValL) && QM) PS.END_ALL_SCAN_CONN_REQ_TIME = PValL;
                break;
              case BLEP_SetParm_PAIR_MODE:
                if(Sm(PValL, BLE_PairMode_Num) && QM) PS.PAIR_MODE =(BLE_Type_PairMode)PValL;
                break;
              case BLEP_SetParm_PASSCODE:
                if(SE(PValL, 999999) && QM) PS.PASSCODE = PValL;
                break;  
              case BLEP_SetParm_AUTO_ADVERT:
                if(Bool(PValL) && QM) PS.AUTO_ADVERT = PValL;
                break;
              case BLEP_SetParm_SEND_DONE_DELAY:
                if(Bool(PValL) && QM) PS.SEND_DONE_DELAY = PValL;
                break;
              case BLEP_SetParm_ENABLE_TRANSMIT_ENCRYPT:
                if(Bool(PValL) && QM) PS.ENABLE_TRANSMIT_ENCRYPT = PValL;
                break;
              case BLEP_SetParm_WATCHDOG:
                if(SE(PValL, HAL_SYSTEM_WD_MODE_NUM) && QM) PS.WATCHDOG_MODE = PValL;
                break;
              case BLEP_SetParm_ENABLE_CMD_CHECK_BIT:
                if(Bool(PValL) && QM) PS.ENABLE_CMD_CHECK_BIT = PValL;
                break;
            }
          }
          if(MT){
            E->EParms_Save_Updated = true;
            if(osal_get_timeoutEx(E->TaskId, BLEP_PARM_SAVE_EVT) == 0){
              BLE_CmdRetError(BLE_Error_General, osal_start_timerEx(E->TaskId, BLEP_PARM_SAVE_EVT, BLEP_PARM_SAVE_DELAY));
            }
          }
        }
        break;
        
      case BLE_MsgType_Hal_Set:
        if(BLE_CmdExtParse_8_8(Ext, ExtLen, &PType0, &PType1, true)) {
          switch(PType0){
            case BLE_AddOper_ReadRssi:
              if(Eq(PType1, E->ConnHandle) &&(E->Connected) && QM) BLE_CmdRetError(BLE_Error_Hci, HCI_ReadRssiCmd(GAPRole_GetConnHandle()));
              break;
            case BLE_AddOper_ReadMac:
              if(Eq(PType1, BLE_OWN_CONN_HANDLE) && QM){
                BLE_CmdRetAddInfo(BLE_AddRet_ReadMac, PType1, E->addr, BLEP_ADDR_LEN);
              } else if(Eq(PType1, E->ConnHandle) &&(E->Connected)  && QM){
                BLE_CmdRetAddInfo(BLE_AddRet_ReadMac, PType1, E->centralAddr, BLEP_ADDR_LEN);
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
      case BLE_MsgType_Reboot:
        if(BLE_CmdExtParse(Ext, ExtLen, true) && QM) BLE_CmdRetError(BLE_Error_General, osal_start_timerEx(E->TaskId, BLEP_RESET_EVT, BLEP_End_Conn(false)));
        break;
#if defined(BLE_CENTRAL)
      case BLE_MsgType_SwitchRole:
        if(BLE_CmdExtParse(Ext, ExtLen, true) && QM) {
          E->BLE_CommFunc->SwitchRole();
          BLE_CmdRetError(BLE_Error_General, osal_start_timerEx(E->TaskId, BLEP_RESET_EVT, BLEP_End_Conn(false)));
        }
        break;
#endif
      case BLE_MsgType_Sleep:
        if(BLE_CmdExtParse(Ext, ExtLen, true) && QM) BLE_CmdRetError(BLE_Error_General, osal_start_timerEx(E->TaskId, BLEP_Sleep_EVT, BLEP_End_Conn(E->AdverEnable)));
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
 * @fn      BLEP_Main_Loop
 *
 * @brief   Main Loop
 *
 * @param   ......
 *
 * @return  none
 */
void BLEP_Main_Loop() {
  WD_KICK();
}

/*********************************************************************
*********************************************************************/

#endif