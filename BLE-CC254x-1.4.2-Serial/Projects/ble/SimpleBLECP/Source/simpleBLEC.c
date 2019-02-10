/**************************************************************************************************
  Filename:       BLEC_Central.c
  Revised:        $Date: 2011-06-20 11:57:59 -0700 (Mon, 20 Jun 2011) $
  Revision:       $Revision: 28 $

  Description:    This file contains the Simple BLE Central sample application 
                  for use with the CC2540 Bluetooth Low Energy Protocol Stack.

  Copyright 2010 Texas Instruments Incorporated. All rights reserved.

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
#if defined ( BLE_CENTRAL )
   
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
#define P      BLEC_GVar->EParms                                          \


#define QM    ((Msg_Executed = true) == true)
#define MF    (Msg_Executed == false)
#define MT    (Msg_Executed == true)
#define Bool(_V_)  (_V_ == 0 || _V_ == 1)
#define PU16(_V_)  (0 < _V_ && _V_ <= 0xFFFF)
#define U16(_V_)  (_V_ <= 0xFFFF)
#define PU8(_V_)  (0 < _V_ && _V_ <= 0xFF)
#define U8(_V_)  (_V_ <= 0xFF)
#define PU32(_V_)  (0 < _V_ && _V_ <= 0xFFFFFFFF)
#define U32(_V_)  (_V_ <= 0xFFFFFFFF)
#define Sm(_V0_, _V1_)  (_V0_ < _V1_)
#define SE(_V0_, _V1_)  (_V0_ <= _V1_)
#define Eq(_V0_, _V1_)  (_V0_ == _V1_)
   
#define BLEC_ADDR_LEN B_ADDR_LEN

#define BLEC_START_DEVICE_EVT                   (0x0001 << 0)
#define BLEC_RESET_EVT                          (0x0001 << 1)             // Reset Timer
#define BLEC_SCAN_EVT                           (0x0001 << 2)             // Scan Timer
#define BLEC_END_SCAN_EVT                       (0x0001 << 3)             // End Scan Timer
#define BLEC_CONNECT_EVT                        (0x0001 << 4)             // Connect Timer
#define BLEC_Uart_TIMEOUT_EVT                   (0x0001 << 5)             // Uart Timeout Timer
#define BLEC_Sleep_EVT                          (0x0001 << 6)             // Sleep

#define BLEC_DISCOVERY_MODE                DEVDISC_MODE_ALL     // Discovey mode (limited, general, all)

#define BLEC_DISCOVERY_ACTIVE_SCAN         TRUE         // TRUE to use active scan

#define BLEC_DISCOVERY_WHITE_LIST          FALSE        // TRUE to use white list during discovery

#define BLEC_LINK_HIGH_DUTY_CYCLE          FALSE        // TRUE to use high scan duty cycle when creating link

#define BLEC_LINK_WHITE_LIST               FALSE        // TRUE to use white list when creating link


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
  bool StartDiscovery;  // Value read/write toggle
  BLEC_Type_ConnStatus ConnStatus;
  uint8 addrType;
  uint8 addr[BLEC_ADDR_LEN];
  uint8 name[BLE_Parm_Dev_Name_Len];
  uint8 nameLen;
  int8 ScanRssi;
  uint8 Token;
} BLEC_Type_Dev;

typedef struct{
  BLE_Type_CommFunc* BLE_CommFunc;
  BLEC_Type_EParms EParms;
  BLEC_Type_EParms EParms_Save;
  bool EParms_Save_Updated;
  uint16 PERIOD_SCAN_DEVICE_COUNT;                                              // Current Scan Device
  uint8 TaskId;                                                                 // Task ID for internal task/event processing
  bool Scanning;                                                               // Scanning state
  bool Connecting;   
  BLEC_Type_Dev DevList[BLEC_Dev_Limit_Num];
  bool DevIsConn[BLEC_Dev_Limit_Num]; 
  int8 SCANNED_DEVICE_RSSI; 
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
void BLEC_FindConnHandle(BLEC_Type_Dev**, uint16);
void BLEC_ProcessGATTMsg( gattMsgEvent_t * );
void BLEC_RssiCB( uint16 connHandle, int8 );
uint8 BLEC_EventCB( gapCentralRoleEvent_t * );
void BLEC_ProcessOSALMsg( osal_event_hdr_t * );
void BLEC_GATTDiscoveryEvent( gattMsgEvent_t * );
bool BLEC_FindInfoInAdvertData( uint8 type, uint8 *, uint8, uint8*, uint8* );
void BLEC_Connect();
void BLEC_CmdHandle(uint8*, uint8);
void BLEC_CentralPasscodeCB( uint8*, uint16, uint8, uint8 );
void BLEC_CentralPairStateCB( uint16, uint8, uint8 );
uint32 BLEC_End_Scan_Conn();
bool BLEC_BleSend(uint8, uint8*, uint8);
void BLEC_Cmd_RECEIVE( uint8* , uint8 );
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
  BLE_CmdRetRoleStatus();
  BLE_CmdRetConnStatus(E->DevIsConn, BLEC_Dev_Limit_Num);
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
  EParms->ENABLE_UPDATE_REQUEST = TRUE;
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
  uint8 NameIdx = 0;
  EParms->NAME[NameIdx++] = 'B';
  EParms->NAME[NameIdx++] = 'L';
  EParms->NAME[NameIdx++] = 'E';
  EParms->NAME[NameIdx++] = 'C';
  EParms->NAME_LEN = NameIdx;
  EParms->PAIR_MODE = BLE_PairMode_Passcode_WaitForReq;
  EParms->PASSCODE = 123456;
  osal_memset(EParms->INFO, 0x00, BLE_Parm_Dev_Info_Len);
  EParms->INFO_LEN = 0;
  
  EParms->ENABLE_TRANSMIT_ENCRYPT = false;
  osal_memset(EParms->TRANSMIT_ENCRYPT_KEY, 0x00, BLE_TRANSMIT_ENCRYPT_DATA_LEN);
  EParms->WATCHDOG_MODE = HAL_SYSTEM_WD_MODE_DISABLE;
  EParms->ENABLE_CMD_CHECK_BIT = false;
  for(uint8 i = 0; i < BLEC_Dev_Limit_Num; i++){
    NameIdx = 0;
    EParms->SCAN_DEV_NAME[i][NameIdx++] = 'B';
    EParms->SCAN_DEV_NAME[i][NameIdx++] = 'L';
    EParms->SCAN_DEV_NAME[i][NameIdx++] = 'E';
    EParms->SCAN_DEV_NAME[i][NameIdx++] = 'P';
    EParms->SCAN_DEV_NAME_LEN[i] = NameIdx;
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
  
  E = (BLEC_Type_GlobalVar*)osal_mem_alloc(sizeof(BLEC_Type_GlobalVar));
  // Others
  E->EParms_Save_Updated = false;
  E->PERIOD_SCAN_DEVICE_COUNT = 0;
  E->TaskId = 0;
  E->Scanning = FALSE;
  E->Connecting = FALSE;
  E->SCANNED_DEVICE_RSSI = 0; 

  //System
  E->RoleCB.rssiCB = BLEC_RssiCB;
  E->RoleCB.eventCB = BLEC_EventCB;
  for(uint8 i = 0; i < BLEC_Dev_Limit_Num; i++){
    E->DevList[i].Token = i;
    E->DevList[i].ConnStatus = BLEC_ConnStatus_Unknown;
    E->DevList[i].ConnHandle = 0xFFFF;
  }
  osal_memset(E->DevIsConn, false, BLEC_Dev_Limit_Num);
  E->BondCB.passcodeCB = BLEC_CentralPasscodeCB;
  E->BondCB.pairStateCB = BLEC_CentralPairStateCB;
}

/*********************************************************************
 * @fn      BLEC_FindConnHandle
 *
 * @brief   
 *
 * @param   
 *
 f @return  none
 */
void BLEC_FindConnHandle(BLEC_Type_Dev** Dest, uint16 Val){
  for(uint8 i = 0; i < BLEC_Dev_Limit_Num; i++){
    if(E->DevList[i].ConnHandle ==  Val) {
      *Dest = &E->DevList[i];
      return;
    }
  }
  *Dest = 0;
}

/*********************************************************************
 * @fn      BLEC_Init
 *
 * @brief   Initialization function for the Simple BLE Central App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notification).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 f @return  none
 */
void BLEC_Init( uint8 task_id , BLE_Type_CommFunc* BLE_CommFunc)
{
  BLEC_Init_GlobalVar();
  osal_set_loop_func(BLEC_Main_Loop);
  E->BLE_CommFunc = BLE_CommFunc;
  E->BLE_CommFunc->CmdReceive = BLEC_Cmd_RECEIVE;
  E->BLE_CommFunc->WakeupNotify = BLEC_WakeupNotify;
  
  E->TaskId = task_id;
  
  //Read Parms from flash
  if(osal_snv_read(BLEC_Parms_Flash_Idx, sizeof(P), (uint8*)&P) != SUCCESS ||
  P.INTER_VERSION != BLE_Parm_INTER_VERSION){      //Check whether data is valid
    BLEC_Reset_EParms(&P);
  }
  PS = P;
  for(uint8 i = 0; i < BLEC_Dev_Limit_Num; i++){
    BLE_Copy_Array(E->DevList[i].name, E->DevList[i].nameLen, P.SCAN_DEV_NAME[i], P.SCAN_DEV_NAME_LEN[i]);
  }

  // Setup Central Profile
  {
    uint8 PERIOD_SCAN_DEVICE_MAX_NUM = P.PERIOD_SCAN_DEVICE_MAX_NUM;
    GAPCentralRole_SetParameter ( GAPCENTRALROLE_MAX_SCAN_RES, sizeof( uint8 ), &PERIOD_SCAN_DEVICE_MAX_NUM );
  }
  
  // Setup GAP
  GAP_SetParamValue( TGAP_GEN_DISC_SCAN, P.SCAN_DURATION );
  GAP_SetParamValue( TGAP_LIM_DISC_SCAN, P.SCAN_DURATION );
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, P.NAME_LEN, (uint8*)P.NAME);

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
    uint8 ioCap = (P.PAIR_MODE == BLE_PairMode_Passcode_Initiate || P.PAIR_MODE == BLE_PairMode_Initiate)?
      GAPBOND_IO_CAP_DISPLAY_ONLY:
      GAPBOND_IO_CAP_KEYBOARD_ONLY;    
    uint8 bonding = (P.PAIR_MODE == BLE_PairMode_Passcode_Initiate || P.PAIR_MODE == BLE_PairMode_Initiate)?
      false:
      true;
    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof( passkey ), &passkey );
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof( pairMode ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof( mitm ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof( ioCap ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof( bonding ), &bonding );
  }

  
  // Initialize GATT Client
  VOID GATT_InitClient();

  // Register to receive incoming ATT Indications/Notifications
  GATT_RegisterForInd( E->TaskId );

  // Initialize GATT attributes
  GGS_AddService( GATT_ALL_SERVICES );         // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES ); // GATT attributes

  // Setup a delayed profile startup
  osal_set_event( E->TaskId, BLEC_START_DEVICE_EVT );
   
  HCI_EXT_ClkDivOnHaltCmd( LL_EXT_DISABLE_CLK_DIVIDE_ON_HALT );
  
  HAL_SYSTEM_SET_WATCHDOG(P.WATCHDOG_MODE);
  
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
uint16 BLEC_ProcessEvent( uint8 task_id, uint16 events )
{
  VOID task_id; // OSAL required parameter that isn't used in this function
  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;
    if ( (pMsg = osal_msg_receive( E->TaskId )) != NULL )
    {
      BLEC_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );
      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }
    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }
  if ( events & BLEC_START_DEVICE_EVT )
  {
    // Start the Device
    VOID GAPCentralRole_StartDevice( (gapCentralRoleCB_t *) &E->RoleCB );
    // Register with bond manager after starting device
    GAPBondMgr_Register( (gapBondCBs_t *) &E->BondCB );
    return ( events ^ BLEC_START_DEVICE_EVT );
  }
  // *********************** Handle Own defined Timer ********************************* //
  if ( events & BLEC_RESET_EVT )
  {
    if(E->EParms_Save_Updated == TRUE) {
      osal_snv_write(BLEC_Parms_Flash_Idx, sizeof(BLEC_Type_EParms), (uint8*)&PS);
    }
    SystemResetSoft();
    return ( events ^ BLEC_RESET_EVT );
  }
  
  if ( events & BLEC_SCAN_EVT )
  {
    E->Scanning = TRUE;
    GAPCentralRole_StartDiscovery(BLEC_DISCOVERY_MODE, BLEC_DISCOVERY_ACTIVE_SCAN, BLEC_DISCOVERY_WHITE_LIST);
    osal_start_timerEx( E->TaskId, BLEC_END_SCAN_EVT, P.SCAN_TOTAL_DURATION);
    return ( events ^ BLEC_SCAN_EVT );
  }
  if ( events & BLEC_END_SCAN_EVT )
  {
    GAPCentralRole_CancelDiscovery();
    E->PERIOD_SCAN_DEVICE_COUNT = 0;
    return ( events ^ BLEC_END_SCAN_EVT );
  }
  
  if ( events & BLEC_CONNECT_EVT )
  {
    BLEC_Connect();
    return ( events ^ BLEC_CONNECT_EVT );
  }
  
  if ( events & BLEC_Uart_TIMEOUT_EVT )
  {
    BLE_UartRecTimeout();
    return ( events ^ BLEC_Uart_TIMEOUT_EVT );
  }
  
  if ( events & BLEC_Sleep_EVT ) {
    E->BLE_CommFunc->ForceSleep();
    return ( events ^ BLEC_Sleep_EVT );
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
void BLEC_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
    case GATT_MSG_EVENT:
      BLEC_ProcessGATTMsg( (gattMsgEvent_t *) pMsg );
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
void BLEC_ProcessGATTMsg( gattMsgEvent_t *pMsg )
{
  BLEC_Type_Dev* T;
  BLEC_FindConnHandle(&T, pMsg->connHandle);
  if(T != 0){
    if((pMsg->method == ATT_READ_RSP) ||
      ((pMsg->method == ATT_ERROR_RSP) &&
      (pMsg->msg.errorRsp.reqOpcode == ATT_READ_REQ))){
      if(pMsg->method == ATT_ERROR_RSP ) {
        //uint8 status = pMsg->msg.errorRsp.errCode;
        ////LCD_WRITE_STRING_VALUE( "Read Error", status, 10, HAL_LCD_LINE_1 );
      }
      else{
        // After a successful read, display the read value
        //uint8 valueRead = pMsg->msg.readRsp.value[0];
        //LCD_WRITE_STRING_VALUE( "Read rsp:", valueRead, 10, HAL_LCD_LINE_1 );
      }
    }
    else if ((pMsg->method == ATT_WRITE_RSP) ||
      ((pMsg->method == ATT_ERROR_RSP) &&
      (pMsg->msg.errorRsp.reqOpcode == ATT_WRITE_REQ))){
      if ( pMsg->method == ATT_ERROR_RSP == ATT_ERROR_RSP) {
        //uint8 status = pMsg->msg.errorRsp.errCode;
        if(T->CharSCDoWrite == false) {
          BLE_CmdRetTransmitDone(T->Token, false);
        }
      } else {
        // After a succesful write, display the value that was written and increment value
        if(T->CharSCDoWrite == false) {
          BLE_CmdRetTransmitDone(T->Token, true);
        }
      }
      T->CharSCDoWrite = TRUE;
    } else if ( T->DiscState != BLE_DISC_STATE_IDLE ) {
      BLEC_GATTDiscoveryEvent( pMsg );
    } else if ( ( pMsg->method == ATT_HANDLE_VALUE_NOTI ) ){    //通知 
      if( pMsg->msg.handleValueNoti.handle == SIMPLEPROFILE_CHAR_CS_UUID_READ_HANDLE)   //CHAR7的通知  串口打印
      {
        BLE_BleRetToUart(T->Token, pMsg->msg.handleValueNoti.pValue, pMsg->msg.handleValueNoti.len);
      }
    }
  }
  GATT_bm_free( &pMsg->msg, pMsg->method );
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
void BLEC_RssiCB( uint16 connHandle, int8 rssi )
{
  BLEC_Type_Dev* T;
  BLEC_FindConnHandle(&T, connHandle);
  if(T != 0){
    BLE_CmdRetHal(BLE_HalRet_ReadRssi, T->Token, -rssi);
  }
}

/*********************************************************************
 * @fn      pairStateCB
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
void BLEC_CentralPairStateCB( uint16 connHandle, uint8 state, uint8 status ){
  if ( state == GAPBOND_PAIRING_STATE_STARTED ) {
    //LCD_WRITE_STRING( "Pairing started", HAL_LCD_LINE_1 );
  } else if ( state == GAPBOND_PAIRING_STATE_COMPLETE ) {
    if ( status == SUCCESS || status == SMP_PAIRING_FAILED_UNSPECIFIED ) {
      //LCD_WRITE_STRING( "Pairing success", HAL_LCD_LINE_1 );
    } else {
      GAPCentralRole_TerminateLink(connHandle);
    }
  } else if ( state == GAPBOND_PAIRING_STATE_BONDED ) {
    if ( status == SUCCESS ){
      //LCD_WRITE_STRING( "Bonding success", HAL_LCD_LINE_1 );
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
void BLEC_CentralPasscodeCB( uint8 *deviceAddr, uint16 connectionHandle, uint8 uiInputs, uint8 uiOutputs ){
  GAPBondMgr_PasscodeRsp( connectionHandle, SUCCESS, P.PASSCODE );
}


/*********************************************************************
 * @fn      BLEC_FindInfoInAdvertData
 *
 * @brief   
 *
 * @return  TRUE if  found
 */
bool BLEC_FindInfoInAdvertData( uint8 type, uint8 *pData, uint8 pDataLen, uint8* pInfoIdx, uint8* pInfoLen )
{
  uint8 adLen;
  uint8 adType;
  uint8 pDataIdx = 0;
  // While end of data not reached
  while ( pDataIdx < pDataLen )
  {
    // Get length of next AD item
    adLen = pData[pDataIdx++];
    if ( adLen > 0 )
    {
      adType = pData[pDataIdx++];     
      if ( adType == type )
      {
        (*pInfoIdx) = pDataIdx;
        (*pInfoLen) = adLen - 1;
        return true;
      }
      else
      {
        pDataIdx += adLen;
      }
    }
  }
  // Match not found
  return FALSE;
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
uint8 BLEC_EventCB( gapCentralRoleEvent_t *pEvent )
{
  switch ( pEvent->gap.opcode )
  {
    case GAP_DEVICE_INIT_DONE_EVENT:
      BLE_CmdRetRoleStatus();
      BLE_CmdRetConnStatus(E->DevIsConn, BLEC_Dev_Limit_Num);
      break;
    case GAP_DEVICE_INFO_EVENT:
       {
        switch(pEvent->deviceInfo.eventType){
          case GAP_ADRPT_ADV_IND:
            {
              E->SCANNED_DEVICE_RSSI = pEvent->deviceInfo.rssi;
              E->PERIOD_SCAN_DEVICE_COUNT++; // Scanned Device Counter + 1
              //Compare UUID
              uint8 InfoIdx, InfoLen;
              if((BLEC_FindInfoInAdvertData(GAP_ADTYPE_16BIT_MORE, pEvent->deviceInfo.pEvtData, pEvent->deviceInfo.dataLen, &InfoIdx, &InfoLen ) == true) ||
              (BLEC_FindInfoInAdvertData(GAP_ADTYPE_16BIT_COMPLETE, pEvent->deviceInfo.pEvtData, pEvent->deviceInfo.dataLen, &InfoIdx, &InfoLen ) == true)){
                // Loop all UUID
                while(InfoLen >= 2){
                  if(SIMPLEPROFILE_SERV_UUID == BUILD_UINT16(pEvent->deviceInfo.pEvtData[InfoIdx + 1], pEvent->deviceInfo.pEvtData[InfoIdx])){
                    E->SCANNED_DEVICE_RSSI = 0;
                    break;
                  }
                  InfoIdx += 2;
                  InfoLen -= 2;
                }
              }
            }
            break;
          case GAP_ADRPT_SCAN_RSP:
            {
              if(E->SCANNED_DEVICE_RSSI != 0) {
                // Filter RSSI (Average Method)
                E->SCANNED_DEVICE_RSSI = (E->SCANNED_DEVICE_RSSI + pEvent->deviceInfo.rssi) / 2;
                // Close Enough
                if(E->SCANNED_DEVICE_RSSI  > P.SCAN_RSSI_THRESHOLD){
                  uint8 InfoIdx, InfoLen;
                  // Find Name
                  if(BLEC_FindInfoInAdvertData(GAP_ADTYPE_LOCAL_NAME_COMPLETE, pEvent->deviceInfo.pEvtData, pEvent->deviceInfo.dataLen, &InfoIdx, &InfoLen) == true){
                    if(InfoLen <= BLE_Parm_Dev_Name_Len){
                      uint8 NameLen = 0;
                      uint8 Name[BLE_Parm_Dev_Name_Len] = { 0 };
                      NameLen = InfoLen;
                      osal_memcpy(Name, &pEvent->deviceInfo.pEvtData[InfoIdx], NameLen);
                      
                      int8 Rssi = 0, Rssi_Swap = 0;
                      uint8 Addr[BLEC_ADDR_LEN] = { 0 }, Addr_Swap[BLEC_ADDR_LEN] = { 0 };
                      uint8 AddrType = 0, AddrType_Swap = 0;
                      
                      Rssi = E->SCANNED_DEVICE_RSSI ;
                      osal_memcpy(Addr, pEvent->deviceInfo.addr, BLEC_ADDR_LEN);
                      AddrType = pEvent->deviceInfo.addrType;
                      
                      bool ReInsert = false; 
                      do {
                        ReInsert = false;
                        for(uint8 i = 0; i < BLEC_Dev_Limit_Num; i++){ 
                          // Name Matched
                          if((NameLen == E->DevList[i].nameLen) && osal_memcmp(Name, &E->DevList[i].name, NameLen) == true) {
                            if(E->DevList[i].ConnStatus == BLEC_ConnStatus_Unknown) {   // First Device
                              E->DevList[i].ConnStatus = BLEC_ConnStatus_Scanned;
                              osal_memcpy(E->DevList[i].addr, Addr, BLEC_ADDR_LEN);
                              E->DevList[i].addrType = AddrType;
                              E->DevList[i].ScanRssi = Rssi;
                              break;
                            } else if(E->DevList[i].ConnStatus == BLEC_ConnStatus_Scanned && Rssi  >= E->DevList[i].ScanRssi){ // Device Existed
                              Rssi_Swap = E->DevList[i].ScanRssi;
                              osal_memcpy(Addr_Swap, E->DevList[i].addr, BLEC_ADDR_LEN);
                              AddrType_Swap = E->DevList[i].addrType;
                              
                              E->DevList[i].ConnStatus = BLEC_ConnStatus_Scanned;
                              osal_memcpy(E->DevList[i].addr, Addr, BLEC_ADDR_LEN);
                              E->DevList[i].addrType = AddrType;
                              E->DevList[i].ScanRssi = Rssi;
                              
                              Rssi = Rssi_Swap;
                              osal_memcpy(Addr, Addr_Swap, BLEC_ADDR_LEN);
                              AddrType = AddrType_Swap;
                              ReInsert = true;
                              break;
                            }
                          }
                        }
                      } while(ReInsert == true);
                      if(E->PERIOD_SCAN_DEVICE_COUNT >= P.PERIOD_SCAN_DEVICE_OVERFLOW_COUNT){
                        GAPCentralRole_CancelDiscovery();
                      }
                    }
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
        if(E->Scanning == TRUE){
          uint32 CurrentTime = osal_GetSystemClock();
          if(E->PERIOD_SCAN_DEVICE_COUNT >= P.PERIOD_SCAN_DEVICE_OVERFLOW_COUNT){
            //Resample
            E->PERIOD_SCAN_DEVICE_COUNT = 0;
            GAPCentralRole_StartDiscovery(BLEC_DISCOVERY_MODE, BLEC_DISCOVERY_ACTIVE_SCAN, BLEC_DISCOVERY_WHITE_LIST);
          } else {
            E->PERIOD_SCAN_DEVICE_COUNT = 0;
            osal_stop_timerEx( E->TaskId, BLEC_END_SCAN_EVT);
            E->Scanning = FALSE;

            osal_start_timerEx( E->TaskId, BLEC_CONNECT_EVT, P.MULTI_CONNECT_INTERVAL);
            BLE_CmdRetConnStatus(E->DevIsConn, BLEC_Dev_Limit_Num);
          }
        }
      }
      break;
    case GAP_LINK_ESTABLISHED_EVENT:
      {
        if(E->Connecting == TRUE){
          uint32 CurrentTime = osal_GetSystemClock();
          if ( pEvent->gap.hdr.status == SUCCESS )
          {          
            BLEC_Type_Dev* T;
            for(uint8 i = 0; i < BLEC_Dev_Limit_Num; i++){
              T = &E->DevList[i];
              if(T->ConnStatus == BLEC_ConnStatus_Connecting && osal_memcmp(T->addr, pEvent->linkCmpl.devAddr, BLEC_ADDR_LEN) == true){
                T->DiscState = BLE_DISC_STATE_SVC;
                T->SvcStartHdl = 0;
                T->SvcEndHdl = 0;
                T->CharHdSC = 0;
                T->CharSCDoWrite = TRUE;
                T->StartDiscovery = TRUE;
                T->ConnHandle = pEvent->linkCmpl.connectionHandle;
                T->ConnStatus = BLEC_ConnStatus_Connected;
                uint8 uuid[ATT_BT_UUID_SIZE] = { LO_UINT16(SIMPLEPROFILE_SERV_UUID), HI_UINT16(SIMPLEPROFILE_SERV_UUID) };
                if(GATT_DiscPrimaryServiceByUUID( T->ConnHandle, uuid, ATT_BT_UUID_SIZE, E->TaskId ) != SUCCESS){
                  GAPCentralRole_TerminateLink(pEvent->linkCmpl.connectionHandle);
                }
                E->BLE_CommFunc->ForceWakeup();
                E->DevIsConn[i] = true;
                BLE_CmdRetConnStatus(E->DevIsConn, BLEC_Dev_Limit_Num); 
                break;
              }
            }
          }
          osal_start_timerEx( E->TaskId, BLEC_CONNECT_EVT, P.MULTI_CONNECT_INTERVAL);
          E->Connecting = FALSE;
        } else {
          if ( pEvent->gap.hdr.status == SUCCESS )
          {  
            GAPCentralRole_TerminateLink(pEvent->linkCmpl.connectionHandle);
          }
        }
      }
      break;
    case GAP_LINK_TERMINATED_EVENT:
      {
        BLEC_Type_Dev* T;
        BLEC_FindConnHandle(&T, pEvent->linkTerminate.connectionHandle);
        if(T != 0){
          T->ConnStatus = BLEC_ConnStatus_Unknown;
          T->ConnHandle = 0xFFFF;
          E->DevIsConn[T->Token] = false;
          BLE_CmdRetConnStatus(E->DevIsConn, BLEC_Dev_Limit_Num); 
        }
      }
      break;
    case GAP_LINK_PARAM_UPDATE_EVENT:
      {
        uint8 Send[] = { 0x01, 0x00 };
        if(SimpleProfile_WriteCharValue(E->TaskId, pEvent->linkUpdate.connectionHandle, SIMPLEPROFILE_CHAR_CS_UUID_WRITE_HANDLE, Send, sizeof(Send)) != SUCCESS) {
          GAPCentralRole_TerminateLink(pEvent->linkUpdate.connectionHandle);
        }
      }
      break;
      
    default:
      break;
  }
  return  ( TRUE );
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
  for(uint8 i = 0; i < BLEC_Dev_Limit_Num; i++){
    if(E->DevList[i].ConnStatus == BLEC_ConnStatus_Scanned) {
      E->Connecting = TRUE;
      E->DevList[i].ConnStatus = BLEC_ConnStatus_Connecting;
      GAPCentralRole_EstablishLink(BLEC_LINK_HIGH_DUTY_CYCLE, BLEC_LINK_WHITE_LIST, E->DevList[i].addrType, E->DevList[i].addr);
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
void BLEC_GATTDiscoveryEvent( gattMsgEvent_t *pMsg )
{
  BLEC_Type_Dev* T;
  BLEC_FindConnHandle(&T, pMsg->connHandle);
  if(T != 0){
    if(T->DiscState == BLE_DISC_STATE_SVC )
    {
      // Service found, store handles
      if ( pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP &&
           pMsg->msg.findByTypeValueRsp.numInfo > 0 )
      {
        T->SvcStartHdl = ATT_ATTR_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
        T->SvcEndHdl = ATT_GRP_END_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
      }
      // If procedure complete
      if ( ( pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP  && 
             pMsg->hdr.status == bleProcedureComplete ) ||
           ( pMsg->method == ATT_ERROR_RSP ) )
      {
        if ( T->SvcStartHdl != 0 )
        {
          // Discover characteristic
          attReadByTypeReq_t req;
          T->DiscState = BLE_DISC_STATE_CHAR_SC;
          req.startHandle = T->SvcStartHdl;
          req.endHandle = T->SvcEndHdl;
          req.type.len = ATT_BT_UUID_SIZE;
          req.type.uuid[0] = LO_UINT16(SIMPLEPROFILE_CHAR_SC_UUID);
          req.type.uuid[1] = HI_UINT16(SIMPLEPROFILE_CHAR_SC_UUID);
          GATT_ReadUsingCharUUID( T->ConnHandle, &req, E->TaskId );
        }
      }
    }
    else if(T->DiscState == BLE_DISC_STATE_CHAR_SC )
    {
      // Characteristic found, store handle
      if ( pMsg->method == ATT_READ_BY_TYPE_RSP && pMsg->msg.readByTypeRsp.numPairs > 0 ) {
        T->CharHdSC = BUILD_UINT16( pMsg->msg.readByTypeRsp.pDataList[0], pMsg->msg.readByTypeRsp.pDataList[1] );
        if(P.ENABLE_UPDATE_REQUEST == TRUE) {
          if(SUCCESS != GAPCentralRole_UpdateLink( T->ConnHandle,
          P.UPDATE_MIN_CONN_INTERVAL,
          P.UPDATE_MAX_CONN_INTERVAL,
          P.UPDATE_SLAVE_LATENCY,
          P.UPDATE_CONN_TIMEOUT )){
            GAPCentralRole_TerminateLink(T->ConnHandle);
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
uint32 BLEC_End_Scan_Conn( )
{
  BLE_CmdRetConnStatus(E->DevIsConn, BLEC_Dev_Limit_Num); 
  osal_stop_timerEx( E->TaskId, BLEC_SCAN_EVT);
  osal_stop_timerEx( E->TaskId, BLEC_END_SCAN_EVT);
  osal_stop_timerEx( E->TaskId, BLEC_CONNECT_EVT);

  bool NeedDelay = FALSE;
  if(E->Connecting == TRUE){
    E->Connecting = FALSE;
    NeedDelay = TRUE;
  }
  if(E->Scanning == TRUE){
    E->Scanning = FALSE;
    GAPCentralRole_CancelDiscovery();
    NeedDelay = TRUE;
  }
  //Clear Connected Device
  for(uint8 i = 0; i < BLEC_Dev_Limit_Num; i++){
    if(E->DevList[i].ConnStatus == BLEC_ConnStatus_Connected){
      GAPCentralRole_TerminateLink(E->DevList[i].ConnHandle);
      NeedDelay = TRUE;
    }
    E->DevList[i].ConnStatus = BLEC_ConnStatus_Unknown;
  }
  
  E->PERIOD_SCAN_DEVICE_COUNT = 0;
  return (NeedDelay == TRUE) ? osal_GetSystemClock() + P.END_ALL_SCAN_CONN_REQ_TIME : 0;
}

/*********************************************************************
 * @fn      BLEC_BleSend
 *
 * @brief   
 *
 * @return  none
 */
bool BLEC_BleSend(uint8 DevToken, uint8* data, uint8 dataLen){
  if((DevToken < BLEC_Dev_Limit_Num) && (dataLen <= SIMPLEPROFILE_CHAR_SC_CS_LEN)) {
    BLEC_Type_Dev* T = &E->DevList[DevToken];
    if((T->ConnStatus == BLEC_ConnStatus_Connected) && 
       (T->CharSCDoWrite) && 
       (SimpleProfile_WriteCharValue(E->TaskId, T->ConnHandle, T->CharHdSC, data, dataLen) == SUCCESS)){
       return true;
    }
  }
  return false;
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
  uint8 Role;
  uint8 Type;
  uint8* Ext;
  uint8 ExtLen;
  if(!BLE_CmdParse(Cmd, CmdLen, &Role, &Type, &Ext, &ExtLen)){
    BLE_CmdRetCmdInvalid();
    return;
  }
  uint8 PType0, PType1;
  uint32 PValL;
  uint8* PValArr;
  uint8 PvalArrLen;
  bool Msg_Executed = false;
  switch(Type){
    case BLE_MsgType_Connect_EnAdvert:
      if(BLE_CmdExtParse(Ext, ExtLen) && QM) osal_start_timerEx( E->TaskId, BLEC_SCAN_EVT, BLEC_End_Scan_Conn());
      break;
    case BLE_MsgType_Disconnect_DisAdvert:
      if(BLE_CmdExtParse(Ext, ExtLen) && QM) BLEC_End_Scan_Conn();
      break;
    case BLE_MsgType_RoleStatus_Get:
      if(BLE_CmdExtParse(Ext, ExtLen) && QM) BLE_CmdRetRoleStatus();
      break;
    case BLE_MsgType_ConnStatus_Get:      //Check Connection 
      if(BLE_CmdExtParse(Ext, ExtLen) && QM) BLE_CmdRetConnStatus(E->DevIsConn, BLEC_Dev_Limit_Num);
      break;
    case BLE_MsgType_Parms_Set:       //Change Parms
      if(BLE_CmdExtParse_8_8_N(Ext, ExtLen, &PType0, &PType1, &PValArr, &PvalArrLen)) {
        switch(PType0){
          case BLEC_SetParm_SCAN_DEV_NAME:
            if(SE(PvalArrLen, BLE_Parm_Dev_Name_Len) && Sm(PType1, BLEC_Dev_Limit_Num) && QM) BLE_Copy_Array(PS.SCAN_DEV_NAME[PType1], PS.SCAN_DEV_NAME_LEN[PType1] , PValArr, PvalArrLen);
            break;
        }
      }
      if(MF && BLE_CmdExtParse_8_N(Ext, ExtLen, &PType0, &PValArr, &PvalArrLen)) {
        switch(PType0){
          case BLEC_SetParm_NAME:
            if(SE(PvalArrLen, BLE_Parm_Dev_Name_Len) && QM) BLE_Copy_Array(PS.NAME, PS.NAME_LEN, PValArr, PvalArrLen);
            break;
          case BLEC_SetParm_TRANSMIT_ENCRYPT_KEY:
            if(Eq(PvalArrLen, BLE_TRANSMIT_ENCRYPT_DATA_LEN) && QM) osal_memcpy(PS.TRANSMIT_ENCRYPT_KEY, PValArr, PvalArrLen);
            break;
          case BLEC_SetParm_INFO:
            if(SE(PvalArrLen, BLE_Parm_Dev_Info_Len) && QM) BLE_Copy_Array(PS.INFO, PS.INFO_LEN, PValArr, PvalArrLen);
            break;
        }
      }
      if(MF && BLE_CmdExtParse_8_32(Ext, ExtLen, &PType0, &PValL)) {
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
            if(U8(PValL) && QM) PS.UPDATE_SLAVE_LATENCY = PValL;
            break;
          case BLEC_SetParm_UPDATE_CONN_TIMEOUT:
            if(PU16(PValL) && QM) PS.UPDATE_CONN_TIMEOUT = PValL;
            break;
          case BLEC_SetParm_SCAN_RSSI_THRESHOLD:
            if(Sm(PValL, 0xFF / 2) && QM) PS.SCAN_RSSI_THRESHOLD = (-1) * PValL;
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
            if(Sm(PValL, BLE_PairMode_Num) && QM) PS.PAIR_MODE = (BLE_Type_PairMode)PValL;
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
      };
      if(MT){
        E->EParms_Save_Updated = true;
        BLE_CmdRetParmSetd();
      }
      break;
    case BLE_MsgType_Hal_Set:
      if(BLE_CmdExtParse_8_8(Ext, ExtLen, &PType0, &PType1)) {
        switch(PType0){
          case BLE_HalSet_ReadRssi:
            if(Sm(PType1, BLEC_Dev_Limit_Num) && Eq(E->DevList[PType1].ConnStatus, BLEC_ConnStatus_Connected) && Eq(HCI_ReadRssiCmd(E->DevList[PType1].ConnHandle), SUCCESS) && QM);
            break;
        }
      }
      break;
    case BLE_MsgType_Send:
      if(BLE_CmdExtParse_8_N(Ext, ExtLen, &PType0, &PValArr, &PvalArrLen) && QM) {
        if(BLE_SendBLE(PType0, PValArr, PvalArrLen)){
          E->DevList[PType0].CharSCDoWrite = FALSE;
        } else {
          BLE_CmdRetTransmitDone(PType0, false);
        }
      }
      break;
    case BLE_MsgType_Reboot: //Reboot
      if(BLE_CmdExtParse(Ext, ExtLen) && QM) osal_start_timerEx(E->TaskId, BLEC_RESET_EVT, BLEC_End_Scan_Conn());
      break;
    case BLE_MsgType_SwitchRole:
      if(BLE_CmdExtParse(Ext, ExtLen) && QM) {
        E->BLE_CommFunc->SwitchRole();
        osal_start_timerEx( E->TaskId, BLEC_RESET_EVT, BLEC_End_Scan_Conn());
      }
      break;
    case BLE_MsgType_Sleep:
      if(BLE_CmdExtParse(Ext, ExtLen) && QM) osal_start_timerEx( E->TaskId, BLEC_Sleep_EVT, BLEC_End_Scan_Conn());
      break;
  }
  if(Msg_Executed == false) {
    BLE_CmdRetCmdInvalid();
  }
}

/*********************************************************************
 * @fn      BLEC_Cmd_RECEIVE
 *
 * @brief   Uart Cakkback
 *
 * @param   ......
 *
 * @return  none
 */
void BLEC_Cmd_RECEIVE( uint8* Data, uint8 DataLen )
{
  osal_stop_timerEx( E->TaskId, BLEC_Uart_TIMEOUT_EVT);
  BLE_CmdGetFromUart(Data, DataLen);
  osal_start_timerEx( E->TaskId, BLEC_Uart_TIMEOUT_EVT, BLE_ReceiveBufferTimeout);
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