/**************************************************************************************************
  Filename:       OSAL_simpleBLECentral.c
  Revised:        $Date: 2011-03-03 15:46:41 -0800 (Thu, 03 Mar 2011) $
  Revision:       $Revision: 12 $

  Description:    OSAL task initalization for Simple BLE Central app.


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

/**************************************************************************************************
 *                                            INCLUDES
 **************************************************************************************************/
#if !(defined(BLE_PERIPHERAL)) && !(defined ( BLE_CENTRAL ))
  BLE_PERIPHERAL BLE_CENTRAL Not Defined;
#endif
   
#if !(defined(POWER_SAVING))
  POWER_SAVING Not Defined;
#endif
    
#if (defined(DMA_PM) && (DMA_PM != 0)) || (!defined(DMA_PM))
  PM Not Defined;
#endif

#include "hal_types.h"
#include "OSAL.h"
#include "OSAL_Tasks.h"

#if defined ( APP_UART_MODULE )
  #include "app_UartModule.h"
#elif defined ( APP_RSSI_CHECK )
  #include "app_RssiCheck.h"
#endif

/* HAL */
#include "hal_drivers.h"

/* LL */
#include "ll.h"

/* HCI */
#include "hci_tl.h"

#if defined ( OSAL_CBTIMER_NUM_TASKS )
  #include "osal_cbTimer.h"
#endif

/* L2CAP */
#include "l2cap.h"

/* gap */
#include "gap.h"
#include "gapgattserver.h"
#include "gapbondmgr.h"

/* GATT */
#include "gatt.h"

#include "gattservapp.h"

/* Profiles */

#if defined ( BLE_CENTRAL )
  #include "central.h"
#endif
   
#if defined ( BLE_PERIPHERAL )
#if defined ( PLUS_BROADCASTER )
  #include "peripheralBroadcaster.h"
#else
  #include "peripheral.h"
#endif
#endif


#include "OSAL_snv.h"
   
#include "simpleBLE.h"

#if defined ( BLE_CENTRAL )
  #include "simpleBLEC.h"
#endif

#if defined ( BLE_PERIPHERAL )
  #include "simpleBLEP.h"
#endif

#include "hal_interrupt.h"
#include "OSAL_PwrMgr.h"

/*********************************************************************
 * Const
 */
   
/*********************************************************************
 * GLOBAL VARIABLES
 */
uint16 BLE_SleepManager_TaskId;
uint16 BLE_ProcessEvent(uint8, uint16);
uint16 BLE_Role_ProcessEvent(uint8, uint16);
void BLE_UART_Receive( uint8, uint8 );
void BLE_UART_Send( uint8*, uint8 );
#if defined(BLE_PERIPHERAL) && defined(BLE_CENTRAL)
void BLE_SwitchRole();
#endif
void BLE_Power_ForceSleep();
void BLE_Power_ForceWakeup();
bool BLE_Info_Set(uint8*, uint8);
void BLE_Info_Get(uint8**, uint8*);



BLE_Type_CommFunc BLE_CommFunc = {
#if defined(BLE_PERIPHERAL) && defined(BLE_CENTRAL)
  .SwitchRole = BLE_SwitchRole,
#endif
  .WakeupNotify = 0,
  .ForceSleep = BLE_Power_ForceSleep,
  .ForceWakeup = BLE_Power_ForceWakeup,
  .CmdReceive = 0,
  .CmdSend = APP_BleCmd_Send
};


// The order in this table must be identical to the task initialization calls below in osalInitTask.
const pTaskEventHandlerFn tasksArr[] =
{
  LL_ProcessEvent,
  Hal_ProcessEvent,
  HCI_ProcessEvent,
#if defined ( OSAL_CBTIMER_NUM_TASKS )
  OSAL_CBTIMER_PROCESS_EVENT( osal_CbTimerProcessEvent ),
#endif
  L2CAP_ProcessEvent,
  GAP_ProcessEvent,
  GATT_ProcessEvent,
  SM_ProcessEvent,
  BLE_Role_ProcessEvent,                                             // task 8
  GAPBondMgr_ProcessEvent,
  GATTServApp_ProcessEvent,
  APP_ProcessEvent,
  BLE_ProcessEvent,
};

const uint8 tasksCnt = sizeof( tasksArr ) / sizeof( tasksArr[0] );
uint16 *tasksEvents;

BLE_Type_Editable_CP BLE_Editable_CP;
bool BLE_IsSleep = false;

/*********************************************************************
 * FUNCTIONS
 *********************************************************************/

/*********************************************************************
 * @fn      BLE_Role_ProcessEvent
 *
 * @brief   
 *
 * @param   void
 *
 * @return  none
 */
uint16 BLE_Role_ProcessEvent( uint8 task_id, uint16 events ){
  if(BLE_Editable_CP.CP == BLEMode_Central){
#if defined ( BLE_CENTRAL )
    return GAPCentralRole_ProcessEvent(task_id, events);
#endif
  } else if(BLE_Editable_CP.CP == BLEMode_Peripheral){
#if defined ( BLE_PERIPHERAL )
    return GAPRole_ProcessEvent(task_id, events);
#endif
  }
  return 0;
}

/*********************************************************************
 * @fn      BLE_ProcessEvent
 *
 * @brief   
 *
 * @param   void
 *
 * @return  none
 */
uint16 BLE_ProcessEvent( uint8 task_id, uint16 events ){
  if(BLE_Editable_CP.CP == BLEMode_Central){
#if defined ( BLE_CENTRAL )
    return BLEC_ProcessEvent(task_id, events);
#endif
  } else if(BLE_Editable_CP.CP == BLEMode_Peripheral){
#if defined ( BLE_PERIPHERAL )
    return BLEP_ProcessEvent(task_id, events);
#endif
  }
  return 0;
}


/*********************************************************************
 * @fn      BLE_Info_Get
 *
 * @brief   
 *
 * @param   void
 *
 * @return  none
 */
void BLE_Info_Get(uint8** Info, uint8* InfoLen){
  *Info = BLE_Editable_CP.Info;
  *InfoLen = BLE_Editable_CP.InfoLen;
}

/*********************************************************************
 * @fn      BLE_Info_Set
 *
 * @brief   
 *
 * @param   void
 *
 * @return  none
 */
bool BLE_Info_Set(uint8* Info, uint8 InfoLen){
  if(InfoLen <= BLE_Info_Max_Len){
    BLE_Array_Copy(BLE_Editable_CP.Info, BLE_Editable_CP.InfoLen, Info, InfoLen);
    return BLE_CmdRetError(BLE_Error_General, osal_snv_write(BLE_CP_Flash_Idx, sizeof(BLE_Editable_CP), (uint8*)&BLE_Editable_CP)) == SUCCESS;
  }
  return false;
}

/*********************************************************************
 * @fn      BLE_SwitchRole
 *
 * @brief   
 *
 * @param   void
 *
 * @return  none
 */
#if defined(BLE_PERIPHERAL) && defined(BLE_CENTRAL)
void BLE_SwitchRole(){
  if(BLE_Editable_CP.CP == BLEMode_Central){
    BLE_Editable_CP.CP = BLEMode_Peripheral;
    BLE_CmdRetError(BLE_Error_General, osal_snv_write(BLE_CP_Flash_Idx, sizeof(BLE_Editable_CP), (uint8*)&BLE_Editable_CP));
    BLE_Editable_CP.CP = BLEMode_Central;
  } else if(BLE_Editable_CP.CP == BLEMode_Peripheral){
    BLE_Editable_CP.CP = BLEMode_Central;
    BLE_CmdRetError(BLE_Error_General, osal_snv_write(BLE_CP_Flash_Idx, sizeof(BLE_Editable_CP), (uint8*)&BLE_Editable_CP));
    BLE_Editable_CP.CP = BLEMode_Peripheral;
  }
}
#endif
/*********************************************************************
 * @fn      BLE_Power_ForceWakeup
 *
 * @brief   Force enter into PM0
 *
 * @param   void
 *
 * @return  none
 */
void BLE_Power_ForceWakeup(){
  if(BLE_IsSleep == true){
    BLE_IsSleep = false;
    HalInterruptSet(APP_WAKEUP_INTERRUPT_CHANNEL, 0);
    osal_pwrmgr_prevent_sleep(true);
    CLEAR_SLEEP_MODE();         //退出 休眠 ，进入工作状态
    HAL_BOARD_INIT();           //切换到外部32M 晶振 并且 等待稳定
    BLE_CmdRetError(BLE_Error_Hci, HCI_EXT_ClkDivOnHaltCmd( HCI_EXT_DISABLE_CLK_DIVIDE_ON_HALT ));
    BLE_CmdRetError(BLE_Error_Hci, HCI_EXT_HaltDuringRfCmd( HCI_EXT_HALT_DURING_RF_DISABLE ));
    
    APP_Enter_Wakeup();
    BLE_CommFunc.WakeupNotify();
  }
}

/*********************************************************************
 * @fn      BLE_Power_ForceSleep
 *
 * @brief   Force enter into PM3
 *
 * @param   void
 *
 * @return  none
 */
void BLE_Power_ForceSleep(void)
{
  if(BLE_IsSleep == false){
    BLE_IsSleep = true;
    APP_Enter_Sleep();
    
    BLE_CmdRetError(BLE_Error_Hci, HCI_EXT_ClkDivOnHaltCmd( HCI_EXT_ENABLE_CLK_DIVIDE_ON_HALT ));
    BLE_CmdRetError(BLE_Error_Hci, HCI_EXT_HaltDuringRfCmd( HCI_EXT_HALT_DURING_RF_ENABLE ));
    osal_pwrmgr_prevent_sleep(false);
    
    HalInterruptSet(APP_WAKEUP_INTERRUPT_CHANNEL, BLE_Power_ForceWakeup);
  }
}

/*********************************************************************
 * @fn      osalInitTasks
 *
 * @brief   This function invokes the initialization function for each task.
 *
 * @param   void
 *
 * @return  none
 */
void osalInitTasks( void )
{
  uint8 taskID = 0;
  
  //Read From flash check Central or Pheripheral
  if(osal_snv_read(BLE_CP_Flash_Idx, sizeof(BLE_Editable_CP), (uint8*)&BLE_Editable_CP) != SUCCESS ||  BLE_Editable_CP.INTER_VERSION != BLE_Editable_CP.INTER_VERSION){
#if defined ( BLE_CENTRAL )
    BLE_Editable_CP.CP = BLEMode_Central;
#endif
#if defined ( BLE_PERIPHERAL )
    BLE_Editable_CP.CP = BLEMode_Peripheral;
#endif
    BLE_Editable_CP.INTER_VERSION = BLE_Parm_INTER_VERSION;
    BLE_Editable_CP.InfoLen = 0;
    osal_memset(BLE_Editable_CP.Info, 0, BLE_Info_Max_Len);
  }

  tasksEvents = (uint16 *)osal_mem_alloc( sizeof( uint16 ) * tasksCnt);
  osal_memset( tasksEvents, 0, (sizeof( uint16 ) * tasksCnt));

  /* LL Task */
  LL_Init( taskID++ );

  /* Hal Task */
  Hal_Init( taskID++ );

  /* HCI Task */
  HCI_Init( taskID++ );

#if defined ( OSAL_CBTIMER_NUM_TASKS )
  /* Callback Timer Tasks */
  osal_CbTimerInit( taskID );
  taskID += OSAL_CBTIMER_NUM_TASKS;
#endif

  /* L2CAP Task */
  L2CAP_Init( taskID++ );

  /* GAP Task */
  GAP_Init( taskID++ );

  /* GATT Task */
  GATT_Init( taskID++ );

  /* SM Task */
  SM_Init( taskID++ );

  /* Profiles */
  if(BLE_Editable_CP.CP == BLEMode_Central){
#if defined ( BLE_CENTRAL )
    GAPCentralRole_Init( taskID++ );
#endif
  } else if(BLE_Editable_CP.CP == BLEMode_Peripheral){
#if defined ( BLE_PERIPHERAL )
    GAPRole_Init( taskID++ );
#endif
  }
  
  GAPBondMgr_Init( taskID++ );
  GATTServApp_Init( taskID++ );

  /* Application */

  APP_Info_Get = BLE_Info_Get;
  APP_Info_Set = BLE_Info_Set;
  APP_Init(taskID++);
  
  if(BLE_Editable_CP.CP == BLEMode_Central){
#if defined ( BLE_CENTRAL )
    BLEC_Init(taskID++, &BLE_CommFunc);
#endif
  } else if(BLE_Editable_CP.CP == BLEMode_Peripheral){
#if defined ( BLE_PERIPHERAL )
    BLEP_Init(taskID++, &BLE_CommFunc);
#endif
  } else {
    taskID++;
  }
  APP_BleCmd_Receive = BLE_CommFunc.CmdReceive;
}

/*********************************************************************
*********************************************************************/
