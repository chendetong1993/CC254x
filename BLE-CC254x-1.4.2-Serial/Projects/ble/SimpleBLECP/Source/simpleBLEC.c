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
#include "hal_led.h"
#include "hal_key.h"
#include "hal_adc.h"
/*********************************************************************
 * MACROS
 */

// Length of bd addr as a string
#define Array_Len(__Array__) (sizeof(__Array__) / sizeof(__Array__[0]))
#define Is_Bool(__VAR__) (__VAR__ == true || __VAR__ == false)

/*********************************************************************
 * CONSTANTS
 */
// Simple BLE Central Task Events
#define BLEC_ADDR_LEN B_ADDR_LEN

#define BLEC_START_DEVICE_EVT                   (0x0001 << 0)
#define BLEC_RESET_EVT                          (0x0001 << 1)             // Reset Timer
#define BLEC_SCAN_EVT                           (0x0001 << 2)             // Scan Timer
#define BLEC_END_SCAN_EVT                       (0x0001 << 3)             // End Scan Timer
#define BLEC_CONNECT_EVT                        (0x0001 << 4)             // Connect Timer
#define BLEC_UART_TIMEOUT_EVT                   (0x0001 << 5)             // Uart Timeout Timer
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
  uint8 name[BLE_Parm_Name_Len];
  uint8 nameLen;
  int8 ScanRssi;
  
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
  BLEC_Type_Dev DevList[BLEC_DEVICE_LIMIT_NUM];
  int8 SCANNED_DEVICE_RSSI; 
  uint8 UARTReceiveBuffer[BLE_CMD_Msg_Max_Len];
  uint8 BLEReceiveBuffer[BLE_CMD_Msg_Max_Len];
  uint8 SendBuffer[BLE_CMD_Msg_Max_Len];
  uint8 UARTReceiveBufferLen;
  uint32 UARTReceiveBuffer_LastTime;
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
void BLEC_ProcessGATTMsg( gattMsgEvent_t * );
void BLEC_RssiCB( uint16 connHandle, int8 );
uint8 BLEC_EventCB( gapCentralRoleEvent_t * );
void BLEC_ProcessOSALMsg( osal_event_hdr_t * );
void BLEC_GATTDiscoveryEvent( gattMsgEvent_t * );
bool BLEC_FindInfoInAdvertData( uint8 type, uint8 *, uint8, uint8*, uint8* );
void BLEC_Connect();
void BLEC_SerialReturnResult(BLE_Type_MsgType, uint8*, uint8);
bool BLEC_Check_SerailMsgValid(uint8*, uint8);
void BLEC_U32_To_U8Array(uint32, uint8*, uint8*);
void BLEC_U8Array_To_U32(uint8*, uint8, uint32*);
void BLEC_MsgRead(uint8*, uint8, uint8*, uint8*, uint32*, void (*)(uint8*, uint8));
void BLEC_UARTMsgHandle(uint8*, uint8);
void BLEC_CentralPasscodeCB( uint8*, uint16, uint8, uint8 );
void BLEC_CentralPairStateCB( uint16, uint8, uint8 );
uint32 BLEC_End_Scan_Conn();
void BLEC_SerialReturnRoleStatus(BLE_Type_PowerStatus);
void BLEC_SerialReturnConnStatus();
void BLEC_UART_RECEIVE( uint8* , uint8 );
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
  HalLedExitSleep();
  HalKeyExitSleep();
  BLEC_SerialReturnRoleStatus(BLE_PowerStatus_Awake);
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
  EParms->VERSION = 0;
  EParms->ENABLE_TRANSMIT_ENCRYPT = true;
  osal_memset(EParms->TRANSMIT_ENCRYPT_KEY, 0x00, BLE_TRANSMIT_ENCRYPT_DATA_LEN);
  EParms->WATCHDOG_MODE = HAL_SYSTEM_WD_MODE_DISABLE;
  EParms->ENABLE_CMD_CHECK_BIT = false;
  for(uint8 i = 0; i < BLEC_DEVICE_LIMIT_NUM; i++){
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
  
  BLEC_GVar = (BLEC_Type_GlobalVar*)osal_mem_alloc(sizeof(BLEC_Type_GlobalVar));
  // Others
  BLEC_GVar->EParms_Save_Updated = false;
  BLEC_GVar->PERIOD_SCAN_DEVICE_COUNT = 0;
  BLEC_GVar->TaskId = 0;
  BLEC_GVar->Scanning = FALSE;
  BLEC_GVar->Connecting = FALSE;
  BLEC_GVar->SCANNED_DEVICE_RSSI = 0; 

  //UART Buffer
  BLEC_GVar->UARTReceiveBufferLen = 0;
  BLEC_GVar->UARTReceiveBuffer_LastTime = 0;

  //System
  BLEC_GVar->RoleCB.rssiCB = BLEC_RssiCB;
  BLEC_GVar->RoleCB.eventCB = BLEC_EventCB;

  BLEC_GVar->BondCB.passcodeCB = BLEC_CentralPasscodeCB;
  BLEC_GVar->BondCB.pairStateCB = BLEC_CentralPairStateCB;
}

/*********************************************************************
 * @fn      BLEC_U32_To_U8Array
 *
 * @brief   
 *
 * @param   
 *
 * @return  none
 */
void BLEC_U32_To_U8Array(uint32 source, uint8* dest, uint8* destLen){
  uint8 Offset = 32, OffsetStep = 8, Val = 0; 
  uint8 Count = 0, FindHead = false;
  do {
    Offset -= OffsetStep;
    if(((Val = (source >> Offset) & 0xFF) != 0) || (FindHead == true)){
      FindHead = true;
      dest[(Count++)] = Val;
    }
  } while(Offset >= OffsetStep);
  (*destLen) = Count;
  if((*destLen) == 0){
    dest[(*destLen)++] = 0;
  }
}

/*********************************************************************
 * @fn      BLEC_U8Array_To_U32
 *
 * @brief   
 *
 * @param   
 *
 * @return  none
 */
void BLEC_U8Array_To_U32(uint8* source, uint8 sourceLen, uint32* dest){
  (*dest) = 0;
  for(uint8 i = 0; i < sourceLen; i++){
    (*dest)  = ((*dest) << 8) + source[i];
  }
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
  BLEC_GVar->BLE_CommFunc = BLE_CommFunc;
  BLEC_GVar->BLE_CommFunc->UartReceive = BLEC_UART_RECEIVE;
  BLEC_GVar->BLE_CommFunc->WakeupNotify = BLEC_WakeupNotify;
  
  BLEC_GVar->TaskId = task_id;
  
  //Read Parms from flash
  if(osal_snv_read(BLEC_Parms_Flash_Idx, sizeof(BLEC_GVar->EParms), (uint8*)&BLEC_GVar->EParms) != SUCCESS ||
  BLEC_GVar->EParms.INTER_VERSION != BLE_Parm_INTER_VERSION){      //Check whether data is valid
    BLEC_Reset_EParms(&BLEC_GVar->EParms);
  }
  BLEC_GVar->EParms_Save = BLEC_GVar->EParms;
  for(uint8 i = 0; i < BLEC_DEVICE_LIMIT_NUM; i++){
    BLEC_GVar->DevList[i].ConnStatus = BLEC_ConnStatus_Unknown;
    osal_memcpy(BLEC_GVar->DevList[i].name, BLEC_GVar->EParms.SCAN_DEV_NAME[i], BLEC_GVar->EParms.SCAN_DEV_NAME_LEN[i]);
    BLEC_GVar->DevList[i].nameLen = BLEC_GVar->EParms.SCAN_DEV_NAME_LEN[i];
  }

  // Setup Central Profile
  {
    uint8 PERIOD_SCAN_DEVICE_MAX_NUM = BLEC_GVar->EParms.PERIOD_SCAN_DEVICE_MAX_NUM;
    GAPCentralRole_SetParameter ( GAPCENTRALROLE_MAX_SCAN_RES, sizeof( uint8 ), &PERIOD_SCAN_DEVICE_MAX_NUM );
  }
  
  // Setup GAP
  GAP_SetParamValue( TGAP_GEN_DISC_SCAN, BLEC_GVar->EParms.SCAN_DURATION );
  GAP_SetParamValue( TGAP_LIM_DISC_SCAN, BLEC_GVar->EParms.SCAN_DURATION );
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, BLEC_GVar->EParms.NAME_LEN, (uint8*)BLEC_GVar->EParms.NAME);

  // Setup the GAP Bond Manager
  {
    uint32 passkey = 0;
    
    uint8 pairMode = 
      (BLEC_GVar->EParms.PAIR_MODE == BLE_PairMode_Passcode_Initiate || BLEC_GVar->EParms.PAIR_MODE == BLE_PairMode_Initiate)?
      GAPBOND_PAIRING_MODE_INITIATE:
      GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    uint8 mitm = 
      (BLEC_GVar->EParms.PAIR_MODE == BLE_PairMode_Passcode_Initiate || BLEC_GVar->EParms.PAIR_MODE == BLE_PairMode_Passcode_WaitForReq)?
      true:
      false;
    uint8 ioCap = (BLEC_GVar->EParms.PAIR_MODE == BLE_PairMode_Passcode_Initiate || BLEC_GVar->EParms.PAIR_MODE == BLE_PairMode_Initiate)?
      GAPBOND_IO_CAP_DISPLAY_ONLY:
      GAPBOND_IO_CAP_KEYBOARD_ONLY;    
    uint8 bonding = (BLEC_GVar->EParms.PAIR_MODE == BLE_PairMode_Passcode_Initiate || BLEC_GVar->EParms.PAIR_MODE == BLE_PairMode_Initiate)?
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
  GATT_RegisterForInd( BLEC_GVar->TaskId );

  // Initialize GATT attributes
  GGS_AddService( GATT_ALL_SERVICES );         // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES ); // GATT attributes

  // Setup a delayed profile startup
  osal_set_event( BLEC_GVar->TaskId, BLEC_START_DEVICE_EVT );
   
  HCI_EXT_ClkDivOnHaltCmd( LL_EXT_DISABLE_CLK_DIVIDE_ON_HALT );
  
  HAL_SYSTEM_SET_WATCHDOG(BLEC_GVar->EParms.WATCHDOG_MODE);
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
    if ( (pMsg = osal_msg_receive( BLEC_GVar->TaskId )) != NULL )
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
    VOID GAPCentralRole_StartDevice( (gapCentralRoleCB_t *) &BLEC_GVar->RoleCB );
    // Register with bond manager after starting device
    GAPBondMgr_Register( (gapBondCBs_t *) &BLEC_GVar->BondCB );
    return ( events ^ BLEC_START_DEVICE_EVT );
  }

  // *********************** Handle Own defined Timer ********************************* //
  if ( events & BLEC_RESET_EVT )
  {
    if(BLEC_GVar->EParms_Save_Updated == TRUE) {
      osal_snv_write(BLEC_Parms_Flash_Idx, sizeof(BLEC_Type_EParms), (uint8*)&BLEC_GVar->EParms_Save);
    }
    SystemResetSoft();
    return ( events ^ BLEC_RESET_EVT );
  }
  
  if ( events & BLEC_SCAN_EVT )
  {
    BLEC_GVar->Scanning = TRUE;
    GAPCentralRole_StartDiscovery(BLEC_DISCOVERY_MODE, BLEC_DISCOVERY_ACTIVE_SCAN, BLEC_DISCOVERY_WHITE_LIST);
    osal_start_timerEx( BLEC_GVar->TaskId, BLEC_END_SCAN_EVT, BLEC_GVar->EParms.SCAN_TOTAL_DURATION);
    return ( events ^ BLEC_SCAN_EVT );
  }
  
  if ( events & BLEC_END_SCAN_EVT )
  {
    GAPCentralRole_CancelDiscovery();
    BLEC_GVar->PERIOD_SCAN_DEVICE_COUNT = 0;
    return ( events ^ BLEC_END_SCAN_EVT );
  }
  
  if ( events & BLEC_CONNECT_EVT )
  {
    BLEC_Connect();
    return ( events ^ BLEC_CONNECT_EVT );
  }
  
  if ( events & BLEC_UART_TIMEOUT_EVT )
  {
    if(BLEC_GVar->UARTReceiveBufferLen != 0){
       BLEC_GVar->UARTReceiveBufferLen = 0;
       BLEC_SerialReturnResult(BLE_MsgType_CMD_Invalid, 0, 0);
    }
    return ( events ^ BLEC_UART_TIMEOUT_EVT );
  }
  
  if ( events & BLEC_Sleep_EVT ) {
    BLEC_GVar->BLE_CommFunc->ForceSleep();
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
  for(uint8 i = 0; i < BLEC_DEVICE_LIMIT_NUM; i++){
    if(BLEC_GVar->DevList[i].ConnStatus != BLEC_ConnStatus_Connected) {
      continue;
    }
    if(BLEC_GVar->DevList[i].ConnHandle == pMsg->connHandle) {
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
          if(BLEC_GVar->DevList[i].CharSCDoWrite == false) {
            BLEC_SerialReturnResult(BLE_MsgType_UnSended, &i, sizeof(i));
          }
        } else {
          // After a succesful write, display the value that was written and increment value
          if(BLEC_GVar->DevList[i].CharSCDoWrite == false) {
            BLEC_SerialReturnResult(BLE_MsgType_Sended, &i, sizeof(i));
          }
        }
        BLEC_GVar->DevList[i].CharSCDoWrite = TRUE;
      } else if ( BLEC_GVar->DevList[i].DiscState != BLE_DISC_STATE_IDLE ) {
        BLEC_GATTDiscoveryEvent( pMsg );
      } else if ( ( pMsg->method == ATT_HANDLE_VALUE_NOTI ) ){    //通知 
        if( pMsg->msg.handleValueNoti.handle == SIMPLEPROFILE_CHAR_CS_UUID_READ_HANDLE)   //CHAR7的通知  串口打印
        {
          bStatus_t status = FAILURE;
          
          uint8 *Msg = BLEC_GVar->BLEReceiveBuffer;
          uint8 *MsgLen = &Msg[0];
          uint8 *MsgFrom = &Msg[1];
          uint8 *MsgContent = &Msg[2];
          (*MsgFrom) = i;
          
          osal_memcpy( MsgContent, pMsg->msg.handleValueNoti.pValue, pMsg->msg.handleValueNoti.len );
          (*MsgLen) = pMsg->msg.handleValueNoti.len;
          
          if(BLEC_GVar->EParms.ENABLE_TRANSMIT_ENCRYPT == true) {
            if((*MsgLen) == BLE_TRANSMIT_ENCRYPT_DATA_LEN && LL_EXT_Decrypt(BLEC_GVar->EParms.TRANSMIT_ENCRYPT_KEY, MsgContent, MsgContent) == SUCCESS) {
              MsgLen = &MsgContent[BLE_TRANSMIT_ENCRYPT_DATA_LEN - 1];
              if((*MsgLen) < BLE_TRANSMIT_ENCRYPT_DATA_LEN){
                status = SUCCESS;
              }
            }
          } else {
            status = SUCCESS;
          }
          if(status == SUCCESS){
            BLEC_SerialReturnResult(BLE_MsgType_Received, MsgFrom, (*MsgLen) + 1);
          } else {
            BLEC_SerialReturnResult(BLE_MsgType_UnReceived, MsgFrom, 1);
          }
        }
      }
      break;
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
  for(uint8 i = 0; i < BLEC_DEVICE_LIMIT_NUM; i++){
    if(BLEC_GVar->DevList[i].ConnStatus != BLEC_ConnStatus_Connected) {
      continue;
    }
    if(BLEC_GVar->DevList[i].ConnHandle == connHandle){
      if(BLEC_GVar->DevList[i].ScanRssi != i){
        BLEC_GVar->DevList[i].ScanRssi = rssi;
        uint8 Ext[] = {BLE_HalInfoReturned_ReadRssiStatus, i, -rssi};
        BLEC_SerialReturnResult(BLE_MsgType_HalInfo_Returned, Ext, sizeof(Ext) ); 
      }
      break;
    }
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
  GAPBondMgr_PasscodeRsp( connectionHandle, SUCCESS, BLEC_GVar->EParms.PASSCODE );
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
      BLEC_SerialReturnRoleStatus(BLE_PowerStatus_Awake);
      break;
    case GAP_DEVICE_INFO_EVENT:
       {
        switch(pEvent->deviceInfo.eventType){
          case GAP_ADRPT_ADV_IND:
            {
              BLEC_GVar->SCANNED_DEVICE_RSSI = pEvent->deviceInfo.rssi;
              BLEC_GVar->PERIOD_SCAN_DEVICE_COUNT++; // Scanned Device Counter + 1
              //Compare UUID
              uint8 InfoIdx, InfoLen;
              if((BLEC_FindInfoInAdvertData(GAP_ADTYPE_16BIT_MORE, pEvent->deviceInfo.pEvtData, pEvent->deviceInfo.dataLen, &InfoIdx, &InfoLen ) == true) ||
              (BLEC_FindInfoInAdvertData(GAP_ADTYPE_16BIT_COMPLETE, pEvent->deviceInfo.pEvtData, pEvent->deviceInfo.dataLen, &InfoIdx, &InfoLen ) == true)){
                // Loop all UUID
                while(InfoLen >= 2){
                  if(SIMPLEPROFILE_SERV_UUID == BUILD_UINT16(pEvent->deviceInfo.pEvtData[InfoIdx + 1], pEvent->deviceInfo.pEvtData[InfoIdx])){
                    BLEC_GVar->SCANNED_DEVICE_RSSI = 0;
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
              if(BLEC_GVar->SCANNED_DEVICE_RSSI != 0) {
                // Filter RSSI (Average Method)
                BLEC_GVar->SCANNED_DEVICE_RSSI = (BLEC_GVar->SCANNED_DEVICE_RSSI + pEvent->deviceInfo.rssi) / 2;
                // Close Enough
                if(BLEC_GVar->SCANNED_DEVICE_RSSI  > BLEC_GVar->EParms.SCAN_RSSI_THRESHOLD){
                  uint8 InfoIdx, InfoLen;
                  // Find Name
                  if(BLEC_FindInfoInAdvertData(GAP_ADTYPE_LOCAL_NAME_COMPLETE, pEvent->deviceInfo.pEvtData, pEvent->deviceInfo.dataLen, &InfoIdx, &InfoLen) == true){
                    if(InfoLen <= BLE_Parm_Name_Len){
                      uint8 NameLen = 0;
                      uint8 Name[BLE_Parm_Name_Len] = { 0 };
                      NameLen = InfoLen;
                      osal_memcpy(Name, &pEvent->deviceInfo.pEvtData[InfoIdx], NameLen);
                      
                      int8 Rssi = 0, Rssi_Swap = 0;
                      uint8 Addr[BLEC_ADDR_LEN] = { 0 }, Addr_Swap[BLEC_ADDR_LEN] = { 0 };
                      uint8 AddrType = 0, AddrType_Swap = 0;
                      
                      Rssi = BLEC_GVar->SCANNED_DEVICE_RSSI ;
                      osal_memcpy(Addr, pEvent->deviceInfo.addr, BLEC_ADDR_LEN);
                      AddrType = pEvent->deviceInfo.addrType;
                      
                      bool ReInsert = false; 
                      do {
                        ReInsert = false;
                        for(uint8 i = 0; i < BLEC_DEVICE_LIMIT_NUM; i++){ 
                          // Name Matched
                          if((NameLen == BLEC_GVar->DevList[i].nameLen) && osal_memcmp(Name, &BLEC_GVar->DevList[i].name, NameLen) == true) {
                            if(BLEC_GVar->DevList[i].ConnStatus == BLEC_ConnStatus_Unknown) {   // First Device
                              BLEC_GVar->DevList[i].ConnStatus = BLEC_ConnStatus_Scanned;
                              osal_memcpy(BLEC_GVar->DevList[i].addr, Addr, BLEC_ADDR_LEN);
                              BLEC_GVar->DevList[i].addrType = AddrType;
                              BLEC_GVar->DevList[i].ScanRssi = Rssi;
                              break;
                            } else if(BLEC_GVar->DevList[i].ConnStatus == BLEC_ConnStatus_Scanned && Rssi  >= BLEC_GVar->DevList[i].ScanRssi){ // Device Existed
                              Rssi_Swap = BLEC_GVar->DevList[i].ScanRssi;
                              osal_memcpy(Addr_Swap, BLEC_GVar->DevList[i].addr, BLEC_ADDR_LEN);
                              AddrType_Swap = BLEC_GVar->DevList[i].addrType;
                              
                              BLEC_GVar->DevList[i].ConnStatus = BLEC_ConnStatus_Scanned;
                              osal_memcpy(BLEC_GVar->DevList[i].addr, Addr, BLEC_ADDR_LEN);
                              BLEC_GVar->DevList[i].addrType = AddrType;
                              BLEC_GVar->DevList[i].ScanRssi = Rssi;
                              
                              Rssi = Rssi_Swap;
                              osal_memcpy(Addr, Addr_Swap, BLEC_ADDR_LEN);
                              AddrType = AddrType_Swap;
                              ReInsert = true;
                              break;
                            }
                          }
                        }
                      } while(ReInsert == true);
                      if(BLEC_GVar->PERIOD_SCAN_DEVICE_COUNT >= BLEC_GVar->EParms.PERIOD_SCAN_DEVICE_OVERFLOW_COUNT){
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
        if(BLEC_GVar->Scanning == TRUE){
          uint32 CurrentTime = osal_GetSystemClock();
          if(BLEC_GVar->PERIOD_SCAN_DEVICE_COUNT >= BLEC_GVar->EParms.PERIOD_SCAN_DEVICE_OVERFLOW_COUNT){
            //Resample
            BLEC_GVar->PERIOD_SCAN_DEVICE_COUNT = 0;
            GAPCentralRole_StartDiscovery(BLEC_DISCOVERY_MODE, BLEC_DISCOVERY_ACTIVE_SCAN, BLEC_DISCOVERY_WHITE_LIST);
          } else {
            BLEC_GVar->PERIOD_SCAN_DEVICE_COUNT = 0;
            osal_stop_timerEx( BLEC_GVar->TaskId, BLEC_END_SCAN_EVT);
            BLEC_GVar->Scanning = FALSE;

            osal_start_timerEx( BLEC_GVar->TaskId, BLEC_CONNECT_EVT, BLEC_GVar->EParms.MULTI_CONNECT_INTERVAL);
            BLEC_SerialReturnConnStatus();
          }
        }
      }
      break;
    case GAP_LINK_ESTABLISHED_EVENT:
      {
        if(BLEC_GVar->Connecting == TRUE){
          uint32 CurrentTime = osal_GetSystemClock();
          if ( pEvent->gap.hdr.status == SUCCESS )
          {          
            for(uint8 i = 0; i < BLEC_DEVICE_LIMIT_NUM; i++){
              BLEC_Type_Dev* T = &BLEC_GVar->DevList[i];
              if(T->ConnStatus == BLEC_ConnStatus_Connecting &&
              osal_memcmp(T->addr, pEvent->linkCmpl.devAddr, BLEC_ADDR_LEN) == true){
                T->ScanRssi = 0;
                T->DiscState = BLE_DISC_STATE_SVC;
                T->SvcStartHdl = 0;
                T->SvcEndHdl = 0;
                T->CharHdSC = 0;
                T->CharSCDoWrite = TRUE;
                T->StartDiscovery = TRUE;
                T->ConnHandle = pEvent->linkCmpl.connectionHandle;
                T->ConnStatus = BLEC_ConnStatus_Connected;

                uint8 uuid[ATT_BT_UUID_SIZE] = { LO_UINT16(SIMPLEPROFILE_SERV_UUID), HI_UINT16(SIMPLEPROFILE_SERV_UUID) };
                if(GATT_DiscPrimaryServiceByUUID( T->ConnHandle, uuid, ATT_BT_UUID_SIZE, BLEC_GVar->TaskId ) != SUCCESS){
                  GAPCentralRole_TerminateLink(pEvent->linkCmpl.connectionHandle);
                }
                BLEC_GVar->BLE_CommFunc->ForceWakeup();
                BLEC_SerialReturnConnStatus();
                break;
              }
            }
          }
          osal_start_timerEx( BLEC_GVar->TaskId, BLEC_CONNECT_EVT, BLEC_GVar->EParms.MULTI_CONNECT_INTERVAL);
          BLEC_GVar->Connecting = FALSE;
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
        BLEC_Type_Dev * T;
        for(uint8 i = 0; i < BLEC_DEVICE_LIMIT_NUM; i++){
          T = &BLEC_GVar->DevList[i];
          if(T->ConnHandle == pEvent->linkTerminate.connectionHandle){
            T->ConnStatus = BLEC_ConnStatus_Unknown;
            GAPCentralRole_CancelRssi(T->ConnHandle);
            BLEC_SerialReturnConnStatus();
            break;
          }
        }
      }
      break;
    case GAP_LINK_PARAM_UPDATE_EVENT:
      {
        uint8 Send[] = { 0x01, 0x00 };
        if(SimpleProfile_WriteCharValue(BLEC_GVar->TaskId, pEvent->linkUpdate.connectionHandle, SIMPLEPROFILE_CHAR_CS_UUID_WRITE_HANDLE, sizeof(Send), Send) != SUCCESS) {
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
  for(uint8 i = 0; i < BLEC_DEVICE_LIMIT_NUM; i++){
    if(BLEC_GVar->DevList[i].ConnStatus == BLEC_ConnStatus_Scanned) {
      BLEC_GVar->Connecting = TRUE;
      BLEC_GVar->DevList[i].ConnStatus = BLEC_ConnStatus_Connecting;
      GAPCentralRole_EstablishLink(
        BLEC_LINK_HIGH_DUTY_CYCLE,
        BLEC_LINK_WHITE_LIST,
        BLEC_GVar->DevList[i].addrType,
        BLEC_GVar->DevList[i].addr);
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
  for(uint8 i = 0; i < BLEC_DEVICE_LIMIT_NUM; i++){
    if(BLEC_GVar->DevList[i].ConnStatus != BLEC_ConnStatus_Connected){
     continue;
    }
    if(BLEC_GVar->DevList[i].ConnHandle == pMsg->connHandle){
      if(BLEC_GVar->DevList[i].DiscState == BLE_DISC_STATE_SVC )
      {
        // Service found, store handles
        if ( pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP &&
             pMsg->msg.findByTypeValueRsp.numInfo > 0 )
        {
          BLEC_GVar->DevList[i].SvcStartHdl = ATT_ATTR_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
          BLEC_GVar->DevList[i].SvcEndHdl = ATT_GRP_END_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
        }
        // If procedure complete
        if ( ( pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP  && 
               pMsg->hdr.status == bleProcedureComplete ) ||
             ( pMsg->method == ATT_ERROR_RSP ) )
        {
          if ( BLEC_GVar->DevList[i].SvcStartHdl != 0 )
          {
            // Discover characteristic
            attReadByTypeReq_t req;
            BLEC_GVar->DevList[i].DiscState = BLE_DISC_STATE_CHAR_SC;
            req.startHandle = BLEC_GVar->DevList[i].SvcStartHdl;
            req.endHandle = BLEC_GVar->DevList[i].SvcEndHdl;
            req.type.len = ATT_BT_UUID_SIZE;
            req.type.uuid[0] = LO_UINT16(SIMPLEPROFILE_CHAR_SC_UUID);
            req.type.uuid[1] = HI_UINT16(SIMPLEPROFILE_CHAR_SC_UUID);
            GATT_ReadUsingCharUUID( BLEC_GVar->DevList[i].ConnHandle, &req, BLEC_GVar->TaskId );
          }
        }
      }
      else if(BLEC_GVar->DevList[i].DiscState == BLE_DISC_STATE_CHAR_SC )
      {
        // Characteristic found, store handle
        if ( pMsg->method == ATT_READ_BY_TYPE_RSP && 
             pMsg->msg.readByTypeRsp.numPairs > 0 )
        {
          BLEC_GVar->DevList[i].CharHdSC = BUILD_UINT16( pMsg->msg.readByTypeRsp.pDataList[0], pMsg->msg.readByTypeRsp.pDataList[1] );
          if(BLEC_GVar->EParms.ENABLE_UPDATE_REQUEST == TRUE) {
            if(SUCCESS != GAPCentralRole_UpdateLink( BLEC_GVar->DevList[i].ConnHandle,
            BLEC_GVar->EParms.UPDATE_MIN_CONN_INTERVAL,
            BLEC_GVar->EParms.UPDATE_MAX_CONN_INTERVAL,
            BLEC_GVar->EParms.UPDATE_SLAVE_LATENCY,
            BLEC_GVar->EParms.UPDATE_CONN_TIMEOUT )){
              GAPCentralRole_TerminateLink(BLEC_GVar->DevList[i].ConnHandle);
            }
          }
        }
        BLEC_GVar->DevList[i].DiscState = BLE_DISC_STATE_IDLE;       
      }
      break;
    }
  }
}


/*********************************************************************
 * @fn      BLEC_SerialReturnRoleStatus
 *
 * @brief   
 *
 * @param   
 *
 * @return  none
 */
void BLEC_SerialReturnRoleStatus(BLE_Type_PowerStatus SleepAwake)
{
  uint8 Data[5], DatLen = 0;
  Data[0] = SleepAwake;
  BLEC_U32_To_U8Array(BLEC_GVar->EParms.VERSION, &Data[1], &DatLen);
  BLEC_SerialReturnResult(BLE_MsgType_Device_Inited, Data, DatLen + 1);
}

/*********************************************************************
 * @fn      BLEC_SerialReturnConnStatus
 *
 * @brief   
 *
 * @param   
 *
 * @return  none
 */
void BLEC_SerialReturnConnStatus()
{
  uint8 t[BLEC_DEVICE_LIMIT_NUM + 1];
  uint8 Len = 1;
  for(uint8 i = 0; i < BLEC_DEVICE_LIMIT_NUM; i++){
    if(BLEC_GVar->DevList[i].ConnStatus == BLEC_ConnStatus_Connected){
      t[Len++] = i;
    }
  }
  t[0] = Len - 1;
  BLEC_SerialReturnResult(BLE_MsgType_ConnStatus_Returned, t, Len); 
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
  osal_stop_timerEx( BLEC_GVar->TaskId, BLEC_SCAN_EVT);
  osal_stop_timerEx( BLEC_GVar->TaskId, BLEC_END_SCAN_EVT);
  osal_stop_timerEx( BLEC_GVar->TaskId, BLEC_CONNECT_EVT);
  

  bool NeedDelay = FALSE;
  if(BLEC_GVar->Connecting == TRUE){
    BLEC_GVar->Connecting = FALSE;
    NeedDelay = TRUE;
  }
  if(BLEC_GVar->Scanning == TRUE){
    BLEC_GVar->Scanning = FALSE;
    GAPCentralRole_CancelDiscovery();
    NeedDelay = TRUE;
  }
  //Clear Connected Device
  for(uint8 i = 0; i < BLEC_DEVICE_LIMIT_NUM; i++){
    if(BLEC_GVar->DevList[i].ConnStatus == BLEC_ConnStatus_Connected){
      GAPCentralRole_TerminateLink(BLEC_GVar->DevList[i].ConnHandle);
      NeedDelay = TRUE;
    }
    BLEC_GVar->DevList[i].ConnStatus = BLEC_ConnStatus_Unknown;
  }
  
  BLEC_GVar->PERIOD_SCAN_DEVICE_COUNT = 0;
  return (NeedDelay == TRUE) ? osal_GetSystemClock() + BLEC_GVar->EParms.END_ALL_SCAN_CONN_REQ_TIME : 0;
}

/*********************************************************************
 * @fn      BLEC_SerialReturnResult
 *
 * @brief   Send Result via UART
 *
 * @param   Check whether data is valid
 *
 * @return  none
 */
void BLEC_SerialReturnResult(BLE_Type_MsgType MsgType, uint8* ExtData, uint8 ExtDataLen){
  BLEC_GVar->SendBuffer[BLE_CMD_Msg_LenIdx] = BLE_CMD_Msg_Len(ExtDataLen);
  BLEC_GVar->SendBuffer[BLE_CMD_Msg_RelDevIdx] = BLE_CMD_Msg_RelDev_C;
  BLEC_GVar->SendBuffer[BLE_CMD_Msg_TypeIdx] = MsgType;
  osal_memcpy(&BLEC_GVar->SendBuffer[BLE_CMD_Msg_ExtHeadIdx], ExtData, ExtDataLen);
  //Calculate Sum
  BLEC_GVar->SendBuffer[BLE_CMD_Msg_CheckSumIdx(ExtDataLen)] = 0;
  if(BLEC_GVar->EParms_Save.ENABLE_CMD_CHECK_BIT){
    for(uint8 i = BLE_CMD_Msg_CheckSumHeadIdx, end = BLE_CMD_Msg_CheckSumEndIdx(ExtDataLen); i <= end; i++){
      BLEC_GVar->SendBuffer[BLE_CMD_Msg_CheckSumIdx(ExtDataLen)] += BLEC_GVar->SendBuffer[i];
    }
  }
  BLEC_GVar->BLE_CommFunc->UartSend(BLEC_GVar->SendBuffer, BLE_CMD_Msg_Len(ExtDataLen));
}

/*********************************************************************
 * @fn      BLEC_Check_SerailMsgValid
 *
 * @brief   
 *
 * @param   Check whether serial data is valid
 *
 * @return  none
 */
bool BLEC_Check_SerailMsgValid(uint8* msg, uint8 Len){
  //Check Length
  if(msg[BLE_CMD_Msg_LenIdx] != Len || Len < BLE_CMD_Msg_Len(0)){
    return false;
  }

  // Check Related Device
  if(msg[BLE_CMD_Msg_RelDevIdx] != BLE_CMD_Msg_RelDev_C){
    return false;
  }
  
  uint8 ExtLen = BLE_CMD_Msg_ExtLen(Len);
  //Check Msg Type
  /*
  if(msg[BLE_CMD_Msg_TypeIdx] != BLE_MsgType_Connect_EnAdvert &&
    msg[BLE_CMD_Msg_TypeIdx] != BLE_MsgType_Disconnect_DisAdvert &&
    msg[BLE_CMD_Msg_TypeIdx] != BLE_MsgType_ConnStatus_Get &&
    (msg[BLE_CMD_Msg_TypeIdx] != BLE_MsgType_Parms_Set) &&
    (msg[BLE_CMD_Msg_TypeIdx] != BLE_MsgType_Hal_Set) &&
    (msg[BLE_CMD_Msg_TypeIdx] != BLE_MsgType_Send) &&
    msg[BLE_CMD_Msg_TypeIdx] != BLE_MsgType_Reboot &&
    msg[BLE_CMD_Msg_TypeIdx] != BLE_MsgType_SwitchRole &&
    msg[BLE_CMD_Msg_TypeIdx] != BLE_MsgType_Sleep){
    return false;
  }
  */
  if(BLEC_GVar->EParms_Save.ENABLE_CMD_CHECK_BIT){
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
 * @fn      BLEC_UARTMsgHandle
 *
 * @brief   Handle UART msg
 *
 * @param   msg - Data, Len - Length of Data
 *
 * @return  none
 */
void BLEC_UARTMsgHandle(uint8* Msg, uint8 Len){
  bool Msg_Executed = false;
  if(BLEC_Check_SerailMsgValid(Msg, Len) == true){         // CMD Valid
    uint8 ExtLen = BLE_CMD_Msg_ExtLen(Len);
    switch(Msg[BLE_CMD_Msg_TypeIdx]){
      case BLE_MsgType_Connect_EnAdvert:
        if(ExtLen == 0){
          osal_start_timerEx( BLEC_GVar->TaskId, BLEC_SCAN_EVT, BLEC_End_Scan_Conn());
          BLEC_SerialReturnConnStatus();
          Msg_Executed = true;
        }
        break;
      case BLE_MsgType_Disconnect_DisAdvert:
        if(ExtLen == 0){
          BLEC_End_Scan_Conn();
          BLEC_SerialReturnConnStatus();
          Msg_Executed = true;
        }
        break;
      case BLE_MsgType_RoleStatus_Get:
        if(ExtLen == 0){
          BLEC_SerialReturnRoleStatus(BLE_PowerStatus_Awake);
          Msg_Executed = true;
        }
        break;
      case BLE_MsgType_ConnStatus_Get:      //Check Connection 
        if(ExtLen == 0){
          BLEC_SerialReturnConnStatus();
          Msg_Executed = true;
        }
        break;
      case BLE_MsgType_Parms_Set:       //Change Parms
        {
          if(ExtLen > 2 ){
            uint8 ParmType = Msg[BLE_CMD_Msg_ExtHeadIdx];
            uint8* ParmVal = &Msg[BLE_CMD_Msg_ExtHeadIdx + 1];
            uint8 ParmValLen = ExtLen - 1;
            if(ParmType == BLEC_SetParm_SCAN_DEV_NAME){
              //Check Parms
              uint8 PeripheralNameLen = ParmValLen - 1;
              uint8 PeripheralIdx = ParmVal[0];
              if((BLE_Parm_Name_Len >= PeripheralNameLen) && (PeripheralIdx < BLEC_DEVICE_LIMIT_NUM)){
                uint8* PeripheralName = &ParmVal[1];
                osal_memcpy(BLEC_GVar->EParms_Save.SCAN_DEV_NAME[PeripheralIdx], PeripheralName, PeripheralNameLen);
                BLEC_GVar->EParms_Save.NAME_LEN = PeripheralNameLen;
                Msg_Executed = true;
              }
            } else if(ParmType == BLEC_SetParm_NAME){
              //Check Parms
              if(BLE_Parm_Name_Len >= ParmValLen){
                osal_memcpy(BLEC_GVar->EParms_Save.NAME, ParmVal, ParmValLen);
                BLEC_GVar->EParms_Save.NAME_LEN = ParmValLen;
                Msg_Executed = true;
              }
            } else if(ParmType == BLEC_SetParm_TRANSMIT_ENCRYPT_KEY) {
              //Check Parms
              if(ParmValLen == BLE_TRANSMIT_ENCRYPT_DATA_LEN){
                osal_memcpy(BLEC_GVar->EParms_Save.TRANSMIT_ENCRYPT_KEY, ParmVal, ParmValLen);
                Msg_Executed = true;
              }
            } else {
              if(ParmValLen <= 4){
                uint32 NewData = 0;
                BLEC_U8Array_To_U32(ParmVal, ParmValLen, &NewData);
                switch(ParmType){
                  case BLEC_SetParm_RESET:
                    BLEC_Reset_EParms(&BLEC_GVar->EParms_Save);
                    Msg_Executed = true;
                    break;
                  case BLEC_SetParm_VERSION:
                    BLEC_GVar->EParms_Save.VERSION = NewData;
                    Msg_Executed = true;
                    break;
                  case BLEC_SetParm_ENABLE_UPDATE_REQUEST:
                    if(Is_Bool(NewData)){
                      BLEC_GVar->EParms_Save.ENABLE_UPDATE_REQUEST = NewData;
                      Msg_Executed = true;
                    }
                    break;
                  case BLEC_SetParm_UPDATE_MIN_CONN_INTERVAL:
                    if(NewData > 0){
                      BLEC_GVar->EParms_Save.UPDATE_MIN_CONN_INTERVAL = NewData;
                      Msg_Executed = true;
                    }
                    break;
                  case BLEC_SetParm_UPDATE_MAX_CONN_INTERVAL:
                    if(NewData > 0){
                      BLEC_GVar->EParms_Save.UPDATE_MAX_CONN_INTERVAL = NewData;
                      Msg_Executed = true;
                    }
                    break;
                  case BLEC_SetParm_UPDATE_SLAVE_LATENCY:
                    BLEC_GVar->EParms_Save.UPDATE_SLAVE_LATENCY = NewData;
                    Msg_Executed = true;
                    break;
                  case BLEC_SetParm_UPDATE_CONN_TIMEOUT:
                    if(NewData > 0){
                      BLEC_GVar->EParms_Save.UPDATE_CONN_TIMEOUT = NewData;
                      Msg_Executed = true;
                    }
                    break;
                  case BLEC_SetParm_SCAN_RSSI_THRESHOLD:
                    if(NewData > 0){
                      BLEC_GVar->EParms_Save.SCAN_RSSI_THRESHOLD = (-1) * NewData;
                      Msg_Executed = true;
                    }
                    break;
                  case BLEC_SetParm_SCAN_DURATION:
                    if(NewData > 0){
                      BLEC_GVar->EParms_Save.SCAN_DURATION = NewData;
                      Msg_Executed = true;
                    }
                    break;
                  case BLEC_SetParm_SCAN_TOTAL_DURATION:
                    if(NewData > 0){
                      BLEC_GVar->EParms_Save.SCAN_TOTAL_DURATION = NewData;
                      Msg_Executed = true;
                    }
                    break;
                  case BLEC_SetParm_MULTI_CONNECT_INTERVAL:
                    if(NewData > 0){
                      BLEC_GVar->EParms_Save.MULTI_CONNECT_INTERVAL = NewData;
                      Msg_Executed = true;
                    }
                    break;
                  case BLEC_SetParm_PERIOD_SCAN_DEVICE_MAX_NUM:
                    if(NewData > 0){
                      BLEC_GVar->EParms_Save.PERIOD_SCAN_DEVICE_MAX_NUM = NewData;
                      Msg_Executed = true;
                    }
                    break;  
                  case BLEC_SetParm_PERIOD_SCAN_DEVICE_OVERFLOW_COUNT:
                    if(NewData > 0){
                      BLEC_GVar->EParms_Save.PERIOD_SCAN_DEVICE_OVERFLOW_COUNT = NewData;
                      Msg_Executed = true;
                    }
                    break;
                  case BLEC_SetParm_END_ALL_SCAN_CONN_REQ_TIME:
                    BLEC_GVar->EParms_Save.END_ALL_SCAN_CONN_REQ_TIME = NewData;
                    Msg_Executed = true;
                    break;
                  case BLEC_SetParm_PAIR_MODE:
                    if(NewData == BLE_PairMode_Initiate || NewData == BLE_PairMode_WaitForReq || 
                       NewData == BLE_PairMode_Passcode_Initiate || NewData == BLE_PairMode_Passcode_WaitForReq){
                      BLEC_GVar->EParms_Save.PAIR_MODE = (BLE_Type_PairMode)NewData;
                      Msg_Executed = true;
                    }
                    break;
                  case BLEC_SetParm_PASSCODE:
                    if(NewData <= 999999){
                      BLEC_GVar->EParms_Save.PASSCODE = NewData;
                      Msg_Executed = true;
                    }
                    break;
                  case BLEC_SetParm_ENABLE_TRANSMIT_ENCRYPT:
                    if(Is_Bool(NewData)){
                      BLEC_GVar->EParms_Save.ENABLE_TRANSMIT_ENCRYPT = NewData;
                      Msg_Executed = true;
                    }
                    break;
                  case BLEC_SetParm_WATCHDOG:
                    if(NewData < HAL_SYSTEM_WD_MODE_NUM){
                      BLEC_GVar->EParms_Save.WATCHDOG_MODE = NewData;
                      Msg_Executed = true;
                    }
                    break;
                  case BLEC_SetParm_ENABLE_CMD_CHECK_BIT:
                    if(Is_Bool(NewData)){
                      BLEC_GVar->EParms_Save.ENABLE_CMD_CHECK_BIT = NewData;
                      Msg_Executed = true;
                    }
                    break;
                }
              }
            }
            if(Msg_Executed == true){
              BLEC_GVar->EParms_Save_Updated = true;
              BLEC_SerialReturnResult(BLE_MsgType_Parms_Setd, 0, 0);
            }
          }
        }
        break;
      case BLE_MsgType_Hal_Set:
        {
          if(ExtLen > 2 && ExtLen <= 6){
            uint8 ParmType = Msg[BLE_CMD_Msg_ExtHeadIdx];
            uint8 ParmRelChannel =  Msg[BLE_CMD_Msg_ExtHeadIdx + 1];
            if(ExtLen == 2){
              switch(ParmType){
                case BLE_SetHal_ReadAdcStatus:
                {
                    uint8 Channel = ParmRelChannel >> 4;
                    uint8 Resolution = ParmRelChannel & 0x0F;
                    if((Channel < HAL_ADC_CHANNEL_NUM && Resolution < HAL_ADC_RESOLUTION_NUM)) {
                      HalAdcSetReference(HAL_ADC_REF_AVDD);
                      uint8 Ext[] = {BLE_HalInfoReturned_ReadAdcStatus, ParmRelChannel, (uint8)(HalAdcRead(Channel, Resolution) / 2048 * 255) };
                      BLEC_SerialReturnResult(BLE_MsgType_HalInfo_Returned, Ext, sizeof(Ext) );
                      Msg_Executed = true;
                    }
                  }
                  break;
                case BLE_SetHal_ReadKeyStatus:
                  if((ParmRelChannel < HAL_KEY_CHANNEL_NUM)) {
                    uint8 Ext[] = {BLE_HalInfoReturned_ReadKeyStatus, ParmRelChannel, HalKeyGet(ParmRelChannel)};
                    BLEC_SerialReturnResult(BLE_MsgType_HalInfo_Returned, Ext, sizeof(Ext) );
                    Msg_Executed = true;
                  }
                  break;
                case BLE_SetHal_ReadRssiStatus:
                  if((ParmRelChannel < BLEC_DEVICE_LIMIT_NUM)){
                    BLEC_Type_Dev* T = &BLEC_GVar->DevList[ParmRelChannel];
                    if((T->ConnStatus == BLEC_ConnStatus_Connected) && (HCI_ReadRssiCmd(T->ConnHandle) == SUCCESS)) {
                      T->ScanRssi = 0;
                      Msg_Executed = true;
                    }
                  }
                  break;
              }
            } else if(ExtLen <= 6){
              uint32 ParmVal = 0;
              BLEC_U8Array_To_U32(&Msg[BLE_CMD_Msg_ExtHeadIdx + 2], ExtLen - 2, &ParmVal);
              switch(ParmType){
                case BLE_SetHal_SetLedStatus:
                  if((ParmRelChannel < HAL_LED_CHANNEL_NUM) && (Is_Bool(ParmVal))) {
                    uint8 Ext[] = {BLE_HalInfoReturned_SetLedStatus, ParmRelChannel, HalLedSet(ParmRelChannel, ParmVal)};
                    BLEC_SerialReturnResult(BLE_MsgType_HalInfo_Returned, Ext, sizeof(Ext) );
                    Msg_Executed = true;
                  }
                  break;
              }
            }
          }
        }
        break;
      case BLE_MsgType_Send:
        {
          if(ExtLen >= 2){
            uint8 DeviceToken = Msg[BLE_CMD_Msg_ExtHeadIdx];
            if((DeviceToken < BLEC_DEVICE_LIMIT_NUM)) {
              bStatus_t status = FAILURE;
              BLEC_Type_Dev* T = &BLEC_GVar->DevList[DeviceToken];
              if((T->ConnStatus == BLEC_ConnStatus_Connected) && (T->CharSCDoWrite == true)){
                uint8 TransmitDataLen = ExtLen - 1;
                uint8* TransmitData = &Msg[BLE_CMD_Msg_ExtHeadIdx + 1];
                if(BLEC_GVar->EParms.ENABLE_TRANSMIT_ENCRYPT == true) {
                  if(TransmitDataLen < BLE_TRANSMIT_ENCRYPT_DATA_LEN){
                    osal_memcpy(BLEC_GVar->SendBuffer, TransmitData, TransmitDataLen);
                    BLEC_GVar->SendBuffer[BLE_TRANSMIT_ENCRYPT_DATA_LEN - 1] = TransmitDataLen;
                    if(LL_Encrypt( BLEC_GVar->EParms.TRANSMIT_ENCRYPT_KEY,  BLEC_GVar->SendBuffer, BLEC_GVar->SendBuffer ) == SUCCESS){
                      status = SimpleProfile_WriteCharValue(
                        BLEC_GVar->TaskId,
                        T->ConnHandle,
                        T->CharHdSC,
                        BLE_TRANSMIT_ENCRYPT_DATA_LEN,
                        BLEC_GVar->SendBuffer);
                    }
                  }
                } else {
                  status = SimpleProfile_WriteCharValue(
                    BLEC_GVar->TaskId,
                    T->ConnHandle,
                    T->CharHdSC,
                    TransmitDataLen,
                    TransmitData);
                }
              }
              if(status == SUCCESS){
                BLEC_GVar->DevList[DeviceToken].CharSCDoWrite = FALSE;
              } else {
                BLEC_SerialReturnResult(BLE_MsgType_UnSended, &DeviceToken, sizeof(DeviceToken));
              }
              Msg_Executed = true;
            }
          }
        }
        break;
      case BLE_MsgType_Reboot: //Reboot
        if(ExtLen == 0){
          osal_start_timerEx( BLEC_GVar->TaskId, BLEC_RESET_EVT, BLEC_End_Scan_Conn());
          Msg_Executed = true;
        }
        break;
      case BLE_MsgType_SwitchRole:
        if(ExtLen == 0){
          BLEC_GVar->BLE_CommFunc->SwitchRole();
          osal_start_timerEx( BLEC_GVar->TaskId, BLEC_RESET_EVT, BLEC_End_Scan_Conn());
          Msg_Executed = true;
        }
        break;
      case BLE_MsgType_Sleep:
        if(ExtLen == 0){
          BLEC_SerialReturnRoleStatus(BLE_PowerStatus_Sleep);
          HalLedEnterSleep();
          HalKeyEnterSleep();
          osal_start_timerEx( BLEC_GVar->TaskId, BLEC_Sleep_EVT, BLEC_End_Scan_Conn());
          Msg_Executed = true;
        }
        break;
    }
  }
  if(Msg_Executed == false) {
    BLEC_SerialReturnResult(BLE_MsgType_CMD_Invalid, 0, 0); 
  }
}


/*********************************************************************
 * @fn      BLEC_MsgRead
 *
 * @brief   Handle disconnected msg
 *
 * @param   ......
 *
 * @return  none
 */
void BLEC_MsgRead(uint8* RecData, uint8 RecLen, uint8* CurrBuffer, uint8* CurrLen, uint32* LastTime, void (*Method)(uint8*, uint8)){
   for(uint8 i = 0; i < RecLen; i++){
     // Data Exceed Max Length
     if((*CurrLen) == 0 && RecData[i] > BLE_CMD_Msg_Max_Len){
       continue;
     }
     CurrBuffer[(*CurrLen)++] = RecData[i];
     if(CurrBuffer[0] == (*CurrLen)){
        Method(CurrBuffer, (*CurrLen));
        (*CurrLen) = 0;
     }
   }
   *LastTime = osal_GetSystemClock();
}

/*********************************************************************
 * @fn      BLEC_UART_RECEIVE
 *
 * @brief   UART Cakkback
 *
 * @param   ......
 *
 * @return  none
 */
void BLEC_UART_RECEIVE( uint8* Data, uint8 DataLen )
{
  osal_stop_timerEx( BLEC_GVar->TaskId, BLEC_UART_TIMEOUT_EVT);
  BLEC_MsgRead(
    Data,
    DataLen,
    BLEC_GVar->UARTReceiveBuffer,
    &BLEC_GVar->UARTReceiveBufferLen,
    &BLEC_GVar->UARTReceiveBuffer_LastTime,
    BLEC_UARTMsgHandle
  );
  osal_start_timerEx( BLEC_GVar->TaskId, BLEC_UART_TIMEOUT_EVT, BLE_ReceiveBufferTimeout);
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
