/**************************************************************************************************
  Filename:       simpleBLEPeripheral.c
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
#if defined ( PLUS_BROADCASTER )
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

#define Array_Len(__Array__) (sizeof(__Array__) / sizeof(__Array__[0]))
#define Is_Bool(__VAR__) (__VAR__ == true || __VAR__ == false)

/*********************************************************************
 * CONSTANTS
 */

#define BLEP_DISCOVERABLE_MODE                  GAP_ADTYPE_FLAGS_GENERAL     // Limited discoverable mode advertises for 30.72s, and then stops, General discoverable mode advertises indefinitely
#if defined ( PLUS_BROADCASTER )
  #define BLEP_ADV_IN_CONN_WAIT                    500                          // delay 500 ms
  #define BLEP_SBP_ADV_IN_CONNECTION_EVT           0x0004
#endif

#define BLEP_SBP_START_DEVICE_EVT                               (0x0001 << 0)             // Simple BLE Peripheral Task Events
#define BLEP_RESET_EVT                                          (0x0001 << 1)             // Reset Timer
#define BLEP_SEND_DONE_EVT                                      (0x0001 << 2)             // Send Done Timer
#define BLEP_UART_TIMEOUT_EVT                                   (0x0001 << 3)             // Uart Timeout Timer
#define BLEP_Sleep_EVT                                          (0x0001 << 4)             // Sleep Timer

#if defined FEATURE_OAD
#include "OAD.h"
#include "OAD_target.h"
#endif
#include "hal_led.h"
#include "hal_key.h"
#include "hal_adc.h"
/*********************************************************************
* TYPEDEFS
*/

typedef struct {
  uint8 DataType;
  uint8* Data;
  uint8 DataLen;
} BLEP_Type_ScanAdvertData;

typedef struct{
  BLE_Type_CommFunc* BLE_CommFunc;
  
  BLEP_Type_EParms EParms;
  BLEP_Type_EParms EParms_Save;
  bool EParms_Save_Updated;
  uint8 TaskId;                                                                 // Task ID for internal task/event processing
  // GAP - SCAN RSP data (max size = 31 bytes)
  uint8 scanRspData[31];
  uint8 scanRspDataLen;
  // GAP - Advertisement data (max size = 31 bytes, though this is
  // best kept short to conserve power while advertisting)
  uint8 advertData[31];
  uint8 advertDataLen;
  //bool Char7DoWrite;
  bool Connected;
  //UART Buffer
  uint8 UARTReceiveBuffer[BLE_CMD_Msg_Max_Len];
  uint8 BLEReceiveBuffer[BLE_CMD_Msg_Max_Len];
  uint8 SendBuffer[BLE_CMD_Msg_Max_Len];
  uint8 UARTReceiveBufferLen;
  uint32 UARTReceiveBuffer_LastTime;
  bool SendDone;
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
void BLEP_ScanAdvertData_Construct(BLEP_Type_ScanAdvertData*, uint8, uint8*, uint8*);
void BLEP_ProcessOSALMsg( osal_event_hdr_t * );
void BLEP_StateNotificationCB( gaprole_States_t  );
void BLEP_ProfileChangeCB( uint8  );

void BLEP_MsgRead(uint8*, uint8, uint8*, uint8*, uint32*, void (*)(uint8*, uint8));
void BLEP_UARTMsgHandle(uint8*, uint8);

void BLEP_U32_To_U8Array(uint32, uint8*, uint8*);
void BLEP_U8Array_To_U32(uint8*, uint8, uint32*);
void BLEP_SerialReturnResult(BLE_Type_MsgType, uint8*, uint8);
bool BLEP_Check_SerailMsgValid(uint8*, uint8);
uint32 BLEP_End_Conn_Advert();
void BLEP_Set_AdvertEnable(bool);
void BLEP_CentralPasscodeCB( uint8*, uint16, uint8, uint8 );
void BLEP_CentralPairStateCB( uint16, uint8, uint8 );
void BLEP_SerialReturnRoleStatus(BLE_Type_PowerStatus);
void BLEP_SerialReturnConnStatus();
void BLEP_RssiCB(int8);
void BLEP_ProcessGATTMsg( gattMsgEvent_t * );
void BLEP_UART_RECEIVE( uint8* , uint8 );
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
  HalLedExitSleep();
  HalKeyExitSleep();
  BLEP_SerialReturnRoleStatus(BLE_PowerStatus_Awake);
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
  uint8 NameIdx = 0;
  EParms->NAME[NameIdx++] = 'B';
  EParms->NAME[NameIdx++] = 'L';
  EParms->NAME[NameIdx++] = 'E';
  EParms->NAME[NameIdx++] = 'P';
  EParms->NAME_LEN = NameIdx;  
  EParms->PASSCODE = 123456;
  EParms->PAIR_MODE = BLE_PairMode_Passcode_Initiate;
  EParms->VERSION = 0;
  EParms->AUTO_ADVERT = true;
  EParms->SEND_DONE_DELAY = 35;
  EParms->ENABLE_TRANSMIT_ENCRYPT = true;
  osal_memset(EParms->TRANSMIT_ENCRYPT_KEY, 0x00, BLE_TRANSMIT_ENCRYPT_DATA_LEN);
  EParms->WATCHDOG_MODE = HAL_SYSTEM_WD_MODE_DISABLE;
  EParms->ENABLE_CMD_CHECK_BIT = false;
}

void BLEP_Init_GlobalVar(){
  
  BLEP_GVar = (BLEP_Type_GlobalVar*)osal_mem_alloc(sizeof(BLEP_Type_GlobalVar));
  // Others
  BLEP_GVar->ConnHandle = 0x00;
  BLEP_GVar->scanRspDataLen = 0;        // GAP - SCAN RSP data (max size = 31 bytes)
  BLEP_GVar->advertDataLen = 0;         // GAP - Advertisement data (max size = 31 bytes, though this is
  BLEP_GVar->EParms_Save_Updated = false;
  BLEP_GVar->AdverEnable = false;
  BLEP_GVar->TaskId = 0;
  BLEP_GVar->Connected = false;
  BLEP_GVar->SendDone = true;
  //UART Buffer
  BLEP_GVar->UARTReceiveBufferLen = 0;
  BLEP_GVar->UARTReceiveBuffer_LastTime = 0;

  //BLE Buffer

  //System
  BLEP_GVar->PeripheralCBs.pfnStateChange = BLEP_StateNotificationCB;
  BLEP_GVar->PeripheralCBs.pfnRssiRead = BLEP_RssiCB;

  BLEP_GVar->BondMgrCBs.passcodeCB = BLEP_CentralPasscodeCB;
  BLEP_GVar->BondMgrCBs.pairStateCB = BLEP_CentralPairStateCB;
  
  BLEP_GVar->SimpleProfileCBs.pfnSimpleProfileChange = BLEP_ProfileChangeCB;
  
  BLEP_GVar->gapProfileState = GAPROLE_INIT;
}

/*********************************************************************
 * @fn      BLEP_ScanAdvertData_Construct
 *
 * @brief   
 *
 * @param   
 *
 * @return  none
 */
void BLEP_ScanAdvertData_Construct(BLEP_Type_ScanAdvertData* Data, uint8 DataLen, uint8* Dest, uint8 *DestLen)
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
 * @fn      BLEP_Set_Advert
 *
 * @brief   
 *
 * @param   
 *
 * @return  none
 */
void BLEP_Set_AdvertEnable(bool IsEnable){
  if(BLEP_GVar->AdverEnable != IsEnable){
    BLEP_GVar->AdverEnable = IsEnable;
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( BLEP_GVar->AdverEnable ), &BLEP_GVar->AdverEnable );
  }
}


/*********************************************************************
 * @fn      BLEP_Init
 *
 * @brief   Initialization function for the Simple BLE Peripheral App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void BLEP_Init( uint8 task_id , BLE_Type_CommFunc* BLE_CommFunc)
{
  BLEP_Init_GlobalVar();
  osal_set_loop_func(BLEP_Main_Loop);
  BLEP_GVar->BLE_CommFunc = BLE_CommFunc;
  BLEP_GVar->BLE_CommFunc->UartReceive = BLEP_UART_RECEIVE;
  BLEP_GVar->BLE_CommFunc->WakeupNotify = BLEP_WakeupNotify;
    
  BLEP_GVar->TaskId = task_id;
  
  //Read Parms from flash
  if(osal_snv_read(BLEP_Parms_Flash_Idx, sizeof(BLEP_GVar->EParms), (uint8*)&BLEP_GVar->EParms) != SUCCESS ||
  BLEP_GVar->EParms.INTER_VERSION != BLE_Parm_INTER_VERSION){      //Check whether data is valid
    BLEP_Reset_EParms(&BLEP_GVar->EParms);
  }
  BLEP_GVar->EParms_Save = BLEP_GVar->EParms;
  BLEP_GVar->AdverEnable = BLEP_GVar->EParms.AUTO_ADVERT;
  
  //Init SCAN RSP data
  {
    // GAP - SCAN RSP data (max size = 31 bytes)
    uint8 CONN_INTERVAL_RANGE[] = {
      LO_UINT16( BLEP_GVar->EParms.DESIRED_MIN_CONN_INTERVAL ),   // 100ms
      HI_UINT16( BLEP_GVar->EParms.DESIRED_MIN_CONN_INTERVAL ),
      LO_UINT16( BLEP_GVar->EParms.DESIRED_MAX_CONN_INTERVAL ),   // 1s
      HI_UINT16( BLEP_GVar->EParms.DESIRED_MAX_CONN_INTERVAL )};
    uint8 POWER_LEVEL[] = { 0 };
    BLEP_Type_ScanAdvertData SCAN_DATA[] = {
      {GAP_ADTYPE_LOCAL_NAME_COMPLETE, BLEP_GVar->EParms.NAME, BLEP_GVar->EParms.NAME_LEN},
      {GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE, CONN_INTERVAL_RANGE, Array_Len(CONN_INTERVAL_RANGE) },
      {GAP_ADTYPE_POWER_LEVEL, POWER_LEVEL, Array_Len(POWER_LEVEL) }
    };
    BLEP_ScanAdvertData_Construct(SCAN_DATA, Array_Len(SCAN_DATA), BLEP_GVar->scanRspData, &BLEP_GVar->scanRspDataLen);
  }  
  //Init ADV RSP data
  {
    // GAP - SCAN RSP data (max size = 31 bytes)
    uint8 FLAGS[] = { BLEP_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED };
    uint8 _16BIT_MORE[] = {
      LO_UINT16( SIMPLEPROFILE_SERV_UUID ),
      HI_UINT16( SIMPLEPROFILE_SERV_UUID )
    };
    BLEP_Type_ScanAdvertData ADVERT_DATA[] = {
      {GAP_ADTYPE_FLAGS, FLAGS, Array_Len(FLAGS)},
      {GAP_ADTYPE_16BIT_MORE, _16BIT_MORE, Array_Len(_16BIT_MORE) }
    };
    BLEP_ScanAdvertData_Construct(ADVERT_DATA, Array_Len(ADVERT_DATA), BLEP_GVar->advertData, &BLEP_GVar->advertDataLen);
  }
  // Setup the GAP
  VOID GAP_SetParamValue( TGAP_CONN_PAUSE_PERIPHERAL, BLEP_GVar->EParms.CONN_PAUSE_PERIPHERAL );
  
  // Setup the GAP Peripheral Role Profile
  {


    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16 gapRole_AdvertOffTime = 0;

    // Set the GAP Role Parameters
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( BLEP_GVar->AdverEnable ), &BLEP_GVar->AdverEnable );
    GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( gapRole_AdvertOffTime ), &gapRole_AdvertOffTime );

    GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, BLEP_GVar->scanRspDataLen, BLEP_GVar->scanRspData );
    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, BLEP_GVar->advertDataLen, BLEP_GVar->advertData );
    
    GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( BLEP_GVar->EParms.ENABLE_DESIRED_REQUEST ), &BLEP_GVar->EParms.ENABLE_DESIRED_REQUEST );
    GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( BLEP_GVar->EParms.DESIRED_MIN_CONN_INTERVAL ), &BLEP_GVar->EParms.DESIRED_MIN_CONN_INTERVAL );
    GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( BLEP_GVar->EParms.DESIRED_MAX_CONN_INTERVAL ), &BLEP_GVar->EParms.DESIRED_MAX_CONN_INTERVAL );
    GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( BLEP_GVar->EParms.DESIRED_SLAVE_LATENCY ), &BLEP_GVar->EParms.DESIRED_SLAVE_LATENCY );
    GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( BLEP_GVar->EParms.DESIRED_CONN_TIMEOUT ), &BLEP_GVar->EParms.DESIRED_CONN_TIMEOUT );
  }

  // Set the GAP Characteristics
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, BLEP_GVar->EParms.NAME_LEN, BLEP_GVar->EParms.NAME );

  // Set advertising interval
  {
    uint16 advInt = BLEP_GVar->EParms.ADVERTISING_INTERVAL;

    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
  }

  // Setup the GAP Bond Manager
  {
    uint32 passkey = 0;
    uint8 pairMode = 
      (BLEP_GVar->EParms.PAIR_MODE == BLE_PairMode_Passcode_Initiate || BLEP_GVar->EParms.PAIR_MODE == BLE_PairMode_Initiate)?
      GAPBOND_PAIRING_MODE_INITIATE:
      GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    uint8 mitm = 
      (BLEP_GVar->EParms.PAIR_MODE == BLE_PairMode_Passcode_Initiate || BLEP_GVar->EParms.PAIR_MODE == BLE_PairMode_Passcode_WaitForReq)?
      true:
      false;
    uint8 ioCap = 
      (BLEP_GVar->EParms.PAIR_MODE == BLE_PairMode_Passcode_Initiate || BLEP_GVar->EParms.PAIR_MODE == BLE_PairMode_Initiate)?
      GAPBOND_IO_CAP_DISPLAY_ONLY:
      GAPBOND_IO_CAP_KEYBOARD_ONLY;    
    uint8 bonding = 
      (BLEP_GVar->EParms.PAIR_MODE == BLE_PairMode_Passcode_Initiate || BLEP_GVar->EParms.PAIR_MODE == BLE_PairMode_Initiate)?
      false:
      true;
      
    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof ( passkey ), &passkey );
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof ( pairMode ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof ( mitm ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof ( ioCap ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof ( bonding ), &bonding );
  }

  // Initialize GATT attributes
  GGS_AddService( GATT_ALL_SERVICES );            // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES );    // GATT attributes
  DevInfo_AddService();                           // Device Information Service
  SimpleProfile_AddService( GATT_ALL_SERVICES );  // Simple GATT Profile
  
#if defined FEATURE_OAD
  VOID OADTarget_AddService();
#endif

  
  // Setup the SimpleProfile Characteristic Values
  /*
  {
    uint8 charValue6[SIMPLEPROFILE_CHAR_SC_CS_LEN] = { 0 };
    uint8 charValue7[SIMPLEPROFILE_CHAR_SC_CS_LEN] = { 0 };
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR_SC, SIMPLEPROFILE_CHAR_SC_CS_LEN, charValue6 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR_CS, SIMPLEPROFILE_CHAR_SC_CS_LEN, charValue7 );
  }
  */
  // Register callback with SimpleGATTprofile
  VOID SimpleProfile_RegisterAppCBs( &BLEP_GVar->SimpleProfileCBs );

  osal_set_event( BLEP_GVar->TaskId, BLEP_SBP_START_DEVICE_EVT );

  HCI_EXT_ClkDivOnHaltCmd( LL_EXT_DISABLE_CLK_DIVIDE_ON_HALT );
  
  //Set POWER
  HCI_EXT_SetTxPowerCmd(BLEP_GVar->EParms.POWER_LEVEL);
  
  HAL_SYSTEM_SET_WATCHDOG(BLEP_GVar->EParms.WATCHDOG_MODE);
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
void BLEP_RssiCB( int8 rssi )
{
  if(BLEP_GVar->Connected == true){
    uint8 Ext[] = {BLE_HalInfoReturned_ReadRssiStatus, BLEP_GVar->ConnHandle, -rssi};
    BLEP_SerialReturnResult(BLE_MsgType_HalInfo_Returned, Ext, sizeof(Ext) );
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
uint16 BLEP_ProcessEvent( uint8 task_id, uint16 events )
{
  VOID task_id; // OSAL required parameter that isn't used in this function

  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( BLEP_GVar->TaskId )) != NULL )
    {
      BLEP_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & BLEP_SBP_START_DEVICE_EVT )
  {
    // Start the Device
    VOID GAPRole_StartDevice( &BLEP_GVar->PeripheralCBs );

    // Start Bond Manager
    VOID GAPBondMgr_Register( &BLEP_GVar->BondMgrCBs );

    return ( events ^ BLEP_SBP_START_DEVICE_EVT );
  }
  // *********************** Handle Own defined Timer ********************************* //
  if ( events & BLEP_RESET_EVT ) {
    if(BLEP_GVar->EParms_Save_Updated == TRUE) {
      osal_snv_write(BLEP_Parms_Flash_Idx, sizeof(BLEP_Type_EParms), (uint8*)&BLEP_GVar->EParms_Save);
    }
    SystemResetSoft();
    return ( events ^ BLEP_RESET_EVT );
  }
  if ( events & BLEP_SEND_DONE_EVT ) {
    BLEP_GVar->SendDone = true;
    BLEP_SerialReturnResult(BLE_MsgType_Sended, &BLEP_GVar->ConnHandle, sizeof(BLEP_GVar->ConnHandle));
    return ( events ^ BLEP_SEND_DONE_EVT );
  }
  if ( events & BLEP_UART_TIMEOUT_EVT ) {
    if(BLEP_GVar->UARTReceiveBufferLen != 0){
       BLEP_GVar->UARTReceiveBufferLen = 0;
       BLEP_SerialReturnResult(BLE_MsgType_CMD_Invalid, 0, 0);
    }
    return ( events ^ BLEP_UART_TIMEOUT_EVT );
  }
  if ( events & BLEP_Sleep_EVT ) {
    BLEP_GVar->BLE_CommFunc->ForceSleep();
    return ( events ^ BLEP_Sleep_EVT );
  }
  // ****************************************************************************** //
  
#if defined ( PLUS_BROADCASTER )
  if ( events & BLEP_SBP_ADV_IN_CONNECTION_EVT )
  {
    uint8 turnOnAdv = TRUE;
    // Turn on advertising while in a connection
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( turnOnAdv ), &turnOnAdv );

    return (events ^ BLEP_SBP_ADV_IN_CONNECTION_EVT);
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
void BLEP_ProcessGATTMsg( gattMsgEvent_t *pMsg )
{
  GATT_bm_free( &pMsg->msg, pMsg->method );
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
void BLEP_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event ) {
    case GATT_MSG_EVENT:
      // Process GATT message
      BLEP_ProcessGATTMsg( (gattMsgEvent_t *)pMsg );
      break;
      
    default:
      // do nothing
      break;
  }
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
void BLEP_U32_To_U8Array(uint32 source, uint8* dest, uint8* destLen){
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
void BLEP_U8Array_To_U32(uint8* source, uint8 sourceLen, uint32* dest){
  (*dest) = 0;
  for(uint8 i = 0; i < sourceLen; i++){
    (*dest)  = ((*dest) << 8) + source[i];
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
void BLEP_StateNotificationCB( gaprole_States_t newState )
{
  uint8 ConnectDisConnect = 0;
  switch ( newState )
  {
    case GAPROLE_STARTED:
      {
        uint8 ownAddress[B_ADDR_LEN];
        uint8 systemId[DEVINFO_SYSTEM_ID_LEN];

        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);

        // use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = ownAddress[0];
        systemId[1] = ownAddress[1];
        systemId[2] = ownAddress[2];

        // set middle bytes to zero
        systemId[4] = 0x00;
        systemId[3] = 0x00;

        // shift three bytes up
        systemId[7] = ownAddress[5];
        systemId[6] = ownAddress[4];
        systemId[5] = ownAddress[3];

        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);
        
        BLEP_SerialReturnRoleStatus(BLE_PowerStatus_Awake);
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
        ConnectDisConnect = 2;
      }
      break;
    default:
      {
      }
      break;

  }
  if(ConnectDisConnect == 1){
    BLEP_GVar->BLE_CommFunc->ForceWakeup();
    BLEP_GVar->Connected = true;
    BLEP_GVar->SendDone = true;
    osal_stop_timerEx( BLEP_GVar->TaskId, BLEP_SEND_DONE_EVT);
    BLEP_SerialReturnConnStatus();
  } else if(ConnectDisConnect == 2){
    BLEP_GVar->Connected = false;
    BLEP_GVar->SendDone = true;
    osal_stop_timerEx( BLEP_GVar->TaskId, BLEP_SEND_DONE_EVT);
    BLEP_SerialReturnConnStatus();
  }
  BLEP_GVar->gapProfileState = newState;
  
  VOID BLEP_GVar->gapProfileState;     // added to prevent compiler warning with
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
void BLEP_ProfileChangeCB( uint8 paramID )
{
  switch( paramID )
  {
    case SIMPLEPROFILE_CHAR_SC:
      {
        bStatus_t status = FAILURE;
        
        uint8 *Msg = BLEP_GVar->BLEReceiveBuffer;
        uint8 *MsgLen = &Msg[0];
        uint8 *MsgFrom = &Msg[1];
        uint8 *MsgContent = &Msg[2];
        (*MsgFrom) = BLEP_GVar->ConnHandle;
        if(SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR_SC, MsgLen, MsgContent) == SUCCESS){
          if(BLEP_GVar->EParms.ENABLE_TRANSMIT_ENCRYPT == true) {
            if((*MsgLen) == BLE_TRANSMIT_ENCRYPT_DATA_LEN && LL_EXT_Decrypt(BLEP_GVar->EParms.TRANSMIT_ENCRYPT_KEY, MsgContent, MsgContent) == SUCCESS) {
              MsgLen = &MsgContent[BLE_TRANSMIT_ENCRYPT_DATA_LEN - 1];
              if((*MsgLen) < BLE_TRANSMIT_ENCRYPT_DATA_LEN){
                status = SUCCESS;
              }
            }
          } else {
            status = SUCCESS;
          }
        }
        if(status == SUCCESS){
          BLEP_SerialReturnResult(BLE_MsgType_Received, MsgFrom, (*MsgLen) + 1);
        } else {
          BLEP_SerialReturnResult(BLE_MsgType_UnReceived, MsgFrom, 1);
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
void BLEP_CentralPairStateCB( uint16 connHandle, uint8 state, uint8 status ){
  if ( state == GAPBOND_PAIRING_STATE_STARTED ) {
    //
  } else if ( state == GAPBOND_PAIRING_STATE_COMPLETE ) {
    if ( status == SUCCESS || status == SMP_PAIRING_FAILED_UNSPECIFIED ) {
      //Paired
    } else {
      //UnPaired
      GAPRole_TerminateConnection();
    }
  } else if ( state == GAPBOND_PAIRING_STATE_BONDED ) {
    if ( status == SUCCESS ){
      //LCD_WRITE_STRING( "Bonding success", HAL_LCD_LINE_1 );
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
void BLEP_CentralPasscodeCB( uint8 *deviceAddr, uint16 connectionHandle, uint8 uiInputs, uint8 uiOutputs ){
  GAPBondMgr_PasscodeRsp( connectionHandle, SUCCESS, BLEP_GVar->EParms.PASSCODE );
}

/*********************************************************************
 * @fn      BLEP_SerialReturnResult
 *
 * @brief   Send result via UART
 *
 * @param   MsgType - Msg Type
 *
 * @return  none
 */
void BLEP_SerialReturnResult(BLE_Type_MsgType MsgType, uint8* ExtData, uint8 ExtDataLen){
  BLEP_GVar->SendBuffer[BLE_CMD_Msg_LenIdx] = BLE_CMD_Msg_Len(ExtDataLen);
  BLEP_GVar->SendBuffer[BLE_CMD_Msg_RelDevIdx] = BLE_CMD_Msg_RelDev_P;
  BLEP_GVar->SendBuffer[BLE_CMD_Msg_TypeIdx] = MsgType;
  osal_memcpy(&BLEP_GVar->SendBuffer[BLE_CMD_Msg_ExtHeadIdx], ExtData, ExtDataLen);
  //Calculate Sum
  BLEP_GVar->SendBuffer[BLE_CMD_Msg_CheckSumIdx(ExtDataLen)] = 0;
  if(BLEP_GVar->EParms_Save.ENABLE_CMD_CHECK_BIT){
  for(uint8 i = BLE_CMD_Msg_CheckSumHeadIdx, end = BLE_CMD_Msg_CheckSumEndIdx(ExtDataLen); i <= end; i++){
      BLEP_GVar->SendBuffer[BLE_CMD_Msg_CheckSumIdx(ExtDataLen)] += BLEP_GVar->SendBuffer[i];
    }
  }
  BLEP_GVar->BLE_CommFunc->UartSend(BLEP_GVar->SendBuffer, BLE_CMD_Msg_Len(ExtDataLen));
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
bool BLEP_Check_SerailMsgValid(uint8* msg, uint8 Len){
  //Check Length
  if(msg[BLE_CMD_Msg_LenIdx] != Len || Len < BLE_CMD_Msg_Len(0)){
    return false;
  }
  
  // Check Related Device
  if(msg[BLE_CMD_Msg_RelDevIdx] != BLE_CMD_Msg_RelDev_P){
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
  if(BLEP_GVar->EParms_Save.ENABLE_CMD_CHECK_BIT){
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
 * @fn      BLEC_SerialReturnConnStatus
 *
 * @brief   R
 *
 * @param   
 *
 * @return  none
 */
void BLEP_SerialReturnConnStatus()
{
  uint8 Ext[] = { BLEP_GVar->Connected, BLEP_GVar->ConnHandle };
  uint8 ExtLen = (BLEP_GVar->Connected == true) ? 2 : 1;
  BLEP_SerialReturnResult(BLE_MsgType_ConnStatus_Returned, Ext, ExtLen);
}

/*********************************************************************
 * @fn      BLEP_SerialReturnRoleStatus
 *
 * @brief   
 *
 * @param   
 *
 * @return  none
 */
void BLEP_SerialReturnRoleStatus(BLE_Type_PowerStatus SleepAwake)
{
  uint8 Data[5], DatLen = 0;
  Data[0] = SleepAwake;
  BLEP_U32_To_U8Array(BLEP_GVar->EParms.VERSION, &Data[1], &DatLen);
  BLEP_SerialReturnResult(BLE_MsgType_Device_Inited, Data, DatLen + 1);
}

/*********************************************************************
 * @fn      BLEP_End_Conn_Advert
 *
 * @brief   
 *
 * @param   
 *
 * @return  none
 */
uint32 BLEP_End_Conn_Advert() {
  osal_stop_timerEx( BLEP_GVar->TaskId, BLEP_SEND_DONE_EVT);
  GAPRole_TerminateConnection();
  return BLEP_GVar->EParms.END_ALL_SCAN_CONN_REQ_TIME;
}

/*********************************************************************
 * @fn      BLEP_UARTMsgHandle
 *
 * @brief   Handle UART msg
 *
 * @param   msg - Data, Len - Length of Data
 *
 * @return  none
 */
void BLEP_UARTMsgHandle(uint8* Msg, uint8 Len){
  bool Msg_Executed = false;
  if(BLEP_Check_SerailMsgValid(Msg, Len) == true){
      uint8 ExtLen = BLE_CMD_Msg_ExtLen(Len);
      switch(Msg[BLE_CMD_Msg_TypeIdx]){
        case BLE_MsgType_Connect_EnAdvert:
          if(ExtLen == 0){
            BLEP_End_Conn_Advert();
            BLEP_Set_AdvertEnable(TRUE);
            BLEP_SerialReturnConnStatus();
            Msg_Executed = true;
          }
          break;
        case BLE_MsgType_Disconnect_DisAdvert:
          if(ExtLen == 0){
            BLEP_End_Conn_Advert();
            BLEP_Set_AdvertEnable(FALSE);
            BLEP_SerialReturnConnStatus();
            Msg_Executed = true;
          }
          break;
        case BLE_MsgType_RoleStatus_Get:
          if(ExtLen == 0){
            BLEP_SerialReturnRoleStatus(BLE_PowerStatus_Awake);
            Msg_Executed = true;
          }
          break;
        case BLE_MsgType_ConnStatus_Get:        //Check Connection
          if(ExtLen == 0){
            BLEP_SerialReturnConnStatus();
            Msg_Executed = true;
          }
          break;
        case BLE_MsgType_Parms_Set:       //Change Parms
          {
            if(ExtLen > 2 ){
              if(BLEP_GVar->EParms_Save.INTER_VERSION == BLE_Parm_INTER_VERSION){
                uint8 ParmType = Msg[BLE_CMD_Msg_ExtHeadIdx];
                uint8* ParmVal = &Msg[BLE_CMD_Msg_ExtHeadIdx + 1];
                uint8 ParmValLen = ExtLen - 1;
                
                if(Msg[BLE_CMD_Msg_ExtHeadIdx] == BLEP_SetParm_NAME){
                  //Check Parms
                  if(BLE_Parm_Name_Len >= ParmValLen){
                    osal_memcpy(BLEP_GVar->EParms_Save.NAME, ParmVal, ParmValLen);
                    BLEP_GVar->EParms_Save.NAME_LEN = ParmValLen;
                    Msg_Executed = true;
                  }
                } else if(ParmType == BLEC_SetParm_TRANSMIT_ENCRYPT_KEY) {
                  //Check Parms
                  if(ParmValLen == sizeof(BLEP_GVar->EParms_Save.TRANSMIT_ENCRYPT_KEY)){
                    osal_memcpy(BLEP_GVar->EParms_Save.TRANSMIT_ENCRYPT_KEY, ParmVal, ParmValLen);
                    Msg_Executed = true;
                  }
                } else {
                 if(ParmValLen <= 4){
                  uint32 NewData = 0;
                  BLEP_U8Array_To_U32(ParmVal, ParmValLen, &NewData);
                  switch(ParmType){
                    case BLEP_SetParm_RESET:
                      BLEP_Reset_EParms(&BLEP_GVar->EParms_Save);
                      Msg_Executed = true;
                      break;
                    case BLEP_SetParm_VERSION:
                      BLEP_GVar->EParms_Save.VERSION = NewData;
                      Msg_Executed = true;
                      break;
                    case BLEP_SetParm_ADVERTISING_INTERVAL:
                      if(NewData > 0){
                        BLEP_GVar->EParms_Save.ADVERTISING_INTERVAL = NewData;
                        Msg_Executed = true;
                      }
                      break;
                    case BLEP_SetParm_ENABLE_DESIRED_REQUEST:
                      if(NewData == true || NewData == false){
                        BLEP_GVar->EParms_Save.ENABLE_DESIRED_REQUEST = NewData;
                        Msg_Executed = true;
                      }
                      break;
                    case BLEP_SetParm_DESIRED_MIN_CONN_INTERVAL:
                      if(NewData > 0){
                        BLEP_GVar->EParms_Save.DESIRED_MIN_CONN_INTERVAL = NewData;
                        Msg_Executed = true;
                      }
                      break;
                    case BLEP_SetParm_DESIRED_MAX_CONN_INTERVAL:
                      if(NewData > 0){
                        BLEP_GVar->EParms_Save.DESIRED_MAX_CONN_INTERVAL = NewData;
                        Msg_Executed = true;
                      }
                      break;
                    case BLEP_SetParm_DESIRED_SLAVE_LATENCY:
                      BLEP_GVar->EParms_Save.DESIRED_SLAVE_LATENCY = NewData;
                      Msg_Executed = true;
                      break;
                    case BLEP_SetParm_DESIRED_CONN_TIMEOUT:
                      if(NewData > 0){
                        BLEP_GVar->EParms_Save.DESIRED_CONN_TIMEOUT = NewData;
                        Msg_Executed = true;
                      }
                      break;
                    case BLEP_SetParm_CONN_PAUSE_PERIPHERAL:
                      if(NewData > 0){
                        BLEP_GVar->EParms_Save.CONN_PAUSE_PERIPHERAL = NewData;
                        Msg_Executed = true;
                      }
                      break;
                    case BLEP_SetParm_POWER_LEVEL:
                      if(NewData == LL_EXT_TX_POWER_MINUS_23_DBM || NewData == LL_EXT_TX_POWER_MINUS_6_DBM || 
                         NewData == LL_EXT_TX_POWER_0_DBM || NewData == LL_EXT_TX_POWER_4_DBM){
                        BLEP_GVar->EParms_Save.POWER_LEVEL = NewData;
                        Msg_Executed = true;
                      }
                      break;
                    case BLEP_SetParm_END_ALL_SCAN_CONN_REQ_TIME:
                      if(NewData > 0){
                        BLEP_GVar->EParms_Save.END_ALL_SCAN_CONN_REQ_TIME = NewData;
                        Msg_Executed = true;
                      }
                      break;
                    case BLEP_SetParm_PAIR_MODE:
                      if(NewData == BLE_PairMode_Initiate || NewData == BLE_PairMode_WaitForReq || 
                         NewData == BLE_PairMode_Passcode_Initiate || NewData == BLE_PairMode_Passcode_WaitForReq){
                        BLEP_GVar->EParms_Save.PAIR_MODE = (BLE_Type_PairMode)NewData;
                        Msg_Executed = true;
                      }
                      break;
                    case BLEP_SetParm_PASSCODE:
                      if(NewData <= 999999){
                        BLEP_GVar->EParms_Save.PASSCODE = NewData;
                        Msg_Executed = true;
                      }
                      break;  
                    case BLEP_SetParm_AUTO_ADVERT:
                      if(Is_Bool(NewData)){
                        BLEP_GVar->EParms_Save.AUTO_ADVERT = NewData;
                        Msg_Executed = true;
                      }
                      break;
                    case BLEP_SetParm_SEND_DONE_DELAY:
                      if(Is_Bool(NewData)){
                        BLEP_GVar->EParms_Save.SEND_DONE_DELAY = NewData;
                        Msg_Executed = true;
                      }
                      break;
                    case BLEP_SetParm_ENABLE_TRANSMIT_ENCRYPT:
                      if(Is_Bool(NewData)){
                        BLEP_GVar->EParms_Save.ENABLE_TRANSMIT_ENCRYPT = NewData;
                        Msg_Executed = true;
                      }
                      break;
                    case BLEP_SetParm_WATCHDOG:
                      if(NewData < HAL_SYSTEM_WD_MODE_NUM){
                        BLEP_GVar->EParms_Save.WATCHDOG_MODE = NewData;
                        Msg_Executed = true;
                      }
                      break;
                    case BLEP_SetParm_ENABLE_CMD_CHECK_BIT:
                      if(Is_Bool(NewData)){
                        BLEP_GVar->EParms_Save.ENABLE_CMD_CHECK_BIT = NewData;
                        Msg_Executed = true;
                      }
                      break;
                  }
                }
              }
            }
            }
            if(Msg_Executed == true){
              BLEP_GVar->EParms_Save_Updated = true;
              BLEP_SerialReturnResult(BLE_MsgType_Parms_Setd, 0, 0);
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
                        BLEP_SerialReturnResult(BLE_MsgType_HalInfo_Returned, Ext, sizeof(Ext) );
                        Msg_Executed = true;
                      }
                    }
                    break;
                  case BLE_SetHal_ReadKeyStatus:
                    if((ParmRelChannel < HAL_KEY_CHANNEL_NUM)) {
                      uint8 Ext[] = {BLE_HalInfoReturned_ReadKeyStatus, ParmRelChannel, HalKeyGet(ParmRelChannel)};
                      BLEP_SerialReturnResult(BLE_MsgType_HalInfo_Returned, Ext, sizeof(Ext) );
                      Msg_Executed = true;
                    }
                    break;
                  case BLE_SetHal_ReadRssiStatus:
                    if((ParmRelChannel == BLEP_GVar->ConnHandle) && (BLEP_GVar->Connected == true) && (HCI_ReadRssiCmd(GAPRole_GetConnHandle()) == SUCCESS)) {
                      Msg_Executed = true;
                    }
                    break;
                }
              } else if(ExtLen <= 6){
                uint32 ParmVal = 0;
                BLEP_U8Array_To_U32(&Msg[BLE_CMD_Msg_ExtHeadIdx + 2], ExtLen - 2, &ParmVal);
                switch(ParmType){
                  case BLE_SetHal_SetLedStatus:
                    if((ParmRelChannel < HAL_LED_CHANNEL_NUM) && (Is_Bool(ParmVal))) {
                      uint8 Ext[] = {BLE_HalInfoReturned_SetLedStatus, ParmRelChannel, HalLedSet(ParmRelChannel, ParmVal)};
                      BLEP_SerialReturnResult(BLE_MsgType_HalInfo_Returned, Ext, sizeof(Ext) );
                      Msg_Executed = true;
                    }
                    break;
                }
              }
            }
          }
          break;
        case BLE_MsgType_Send:
          if(ExtLen >= 2){
            uint8 DeviceToken = Msg[BLE_CMD_Msg_ExtHeadIdx];
            if((DeviceToken == BLEP_GVar->ConnHandle)) {
              bStatus_t status = FAILURE;
              if((BLEP_GVar->Connected == true) && (BLEP_GVar->SendDone == true)){
                uint8 TransmitDataLen = ExtLen - 1;
                uint8* TransmitData = &Msg[BLE_CMD_Msg_ExtHeadIdx + 1];
                if(BLEP_GVar->EParms.ENABLE_TRANSMIT_ENCRYPT == true) {
                  if(TransmitDataLen < BLE_TRANSMIT_ENCRYPT_DATA_LEN){
                    osal_memcpy(BLEP_GVar->SendBuffer, TransmitData, TransmitDataLen);
                    BLEP_GVar->SendBuffer[BLE_TRANSMIT_ENCRYPT_DATA_LEN - 1] = TransmitDataLen;
                    if(LL_Encrypt( BLEP_GVar->EParms.TRANSMIT_ENCRYPT_KEY,  BLEP_GVar->SendBuffer, BLEP_GVar->SendBuffer ) == SUCCESS){
                      status = SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR_CS, BLE_TRANSMIT_ENCRYPT_DATA_LEN, BLEP_GVar->SendBuffer ); 
                    }
                  }
                } else {
                  status = SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR_CS, TransmitDataLen, TransmitData ); 
                }
              }
              if(status == SUCCESS){
                //Transmit Successfully
                BLEP_GVar->SendDone = false;
                osal_start_timerEx( BLEP_GVar->TaskId, BLEP_SEND_DONE_EVT, BLEP_GVar->EParms_Save.SEND_DONE_DELAY);
              } else {
                //Transmit Unsuccessfully
                BLEP_SerialReturnResult(BLE_MsgType_UnSended, &BLEP_GVar->ConnHandle, sizeof(BLEP_GVar->ConnHandle)); //Transmit Unsuccessfully
              }
              Msg_Executed = true;
            }
          }
          break;
      case BLE_MsgType_Reboot: //Reboot
          if(ExtLen == 0){
            osal_start_timerEx( BLEP_GVar->TaskId, BLEP_RESET_EVT, BLEP_End_Conn_Advert());
            Msg_Executed = true;
          }
          break;
        case BLE_MsgType_SwitchRole:
          if(ExtLen == 0){
            BLEP_GVar->BLE_CommFunc->SwitchRole();
            osal_start_timerEx( BLEP_GVar->TaskId, BLEP_RESET_EVT, BLEP_End_Conn_Advert());
            Msg_Executed = true;
          }
          break;
        case BLE_MsgType_Sleep:
          if(ExtLen == 0){
            BLEP_SerialReturnRoleStatus(BLE_PowerStatus_Sleep);
            HalLedEnterSleep();
            HalKeyEnterSleep();
            osal_start_timerEx( BLEP_GVar->TaskId, BLEP_Sleep_EVT, BLEP_End_Conn_Advert());
            Msg_Executed = true;
          }
          break;
      }
  }
  if(Msg_Executed == false) {
    BLEP_SerialReturnResult(BLE_MsgType_CMD_Invalid, 0, 0); //CMD Invalid
  }
}


/*********************************************************************
 * @fn      BLEP_MsgRead
 *
 * @brief   Handle disconnected msg
 *
 * @param   ......
 *
 * @return  none
 */
void BLEP_MsgRead(uint8* RecData, uint8 RecLen, uint8* CurrBuffer, uint8* CurrLen, uint32* LastTime, void (*Method)(uint8*, uint8)){
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
 * @fn      BLEP_UART_RECEIVE
 *
 * @brief   UART Cakkback
 *
 * @param   ......
 *
 * @return  none
 */
void BLEP_UART_RECEIVE( uint8* Data, uint8 DataLen )
{
  osal_stop_timerEx( BLEP_GVar->TaskId, BLEP_UART_TIMEOUT_EVT);
  BLEP_MsgRead(
    Data,
    DataLen,
    BLEP_GVar->UARTReceiveBuffer,
    &BLEP_GVar->UARTReceiveBufferLen,
    &BLEP_GVar->UARTReceiveBuffer_LastTime,
    BLEP_UARTMsgHandle
  );
  osal_start_timerEx( BLEP_GVar->TaskId, BLEP_UART_TIMEOUT_EVT, BLE_ReceiveBufferTimeout);
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
void BLEP_Main_Loop()
{
  WD_KICK();
}

/*********************************************************************
*********************************************************************/
