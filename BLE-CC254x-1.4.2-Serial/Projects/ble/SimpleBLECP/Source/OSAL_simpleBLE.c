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
#include "hal_types.h"
#include "OSAL.h"
#include "OSAL_Tasks.h"

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
#include "central.h"
   
/* Profiles */
#if defined ( PLUS_BROADCASTER )
  #include "peripheralBroadcaster.h"
#else
  #include "peripheral.h"
#endif

#include "OSAL_snv.h"
   
#include "simpleBLE.h"
#include "simpleBLEP.h"
#include "simpleBLEC.h"
#include "OSAL_PwrMgr.h"
#include "npi.h"

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
void BLE_SwitchRole();
void BLE_Power_ForceSleep();
void BLE_Power_ForceWakeup();
void BLE_WaitUs(uint16);

BLE_Type_CommFunc BLE_CommFunc = {
  .SwitchRole = BLE_SwitchRole,
  .WakeupNotify = 0,
  .ForceSleep = BLE_Power_ForceSleep,
  .ForceWakeup = BLE_Power_ForceWakeup,
  .UartReceive = 0,
  .UartSend = BLE_UART_Send
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
  BLE_ProcessEvent
};

const uint8 tasksCnt = sizeof( tasksArr ) / sizeof( tasksArr[0] );
uint16 *tasksEvents;

BLE_Type_Editable_CP BLE_Editable_CP;

/*********************************************************************
 * FUNCTIONS
 *********************************************************************/

/*********************************************************************
 * @fn      BLE_ProcessEvent
 *
 * @brief   
 *
 * @param   void
 *
 * @return  none
 */
uint16 BLE_Role_ProcessEvent( uint8 task_id, uint16 events ){
  if(BLE_Editable_CP.CP == BLEMode_Central){
    return GAPCentralRole_ProcessEvent(task_id, events);
  } else if(BLE_Editable_CP.CP == BLEMode_Peripheral){
    return GAPRole_ProcessEvent(task_id, events);
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
    return BLEC_ProcessEvent(task_id, events);
  } else if(BLE_Editable_CP.CP == BLEMode_Peripheral){
    return BLEP_ProcessEvent(task_id, events);
  }
  return 0;
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
void BLE_SwitchRole(){
  if(BLE_Editable_CP.CP == BLEMode_Central){
    BLE_Editable_CP.CP = BLEMode_Peripheral;
    osal_snv_write(BLE_CP_Flash_Idx, sizeof(BLE_Editable_CP), (uint8*)&BLE_Editable_CP);
    BLE_Editable_CP.CP = BLEMode_Central;
  } else if(BLE_Editable_CP.CP == BLEMode_Peripheral){
    BLE_Editable_CP.CP = BLEMode_Central;
    osal_snv_write(BLE_CP_Flash_Idx, sizeof(BLE_Editable_CP), (uint8*)&BLE_Editable_CP);
    BLE_Editable_CP.CP = BLEMode_Peripheral;
  }
}


/*********************************************************************
 * @fn      Hal_HW_WaitUs
 *
 * @brief   WaitUs
 *
 * @param   void
 *
 * @return  none
 */

void BLE_WaitUs(uint16 microSecs)
{
  while(microSecs--)
  {
    //32 NOPs == 1 usecs
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop");
  }
}


/*********************************************************************
 * @fn      P0_ISR (BLE_Power_Wakeup)
 *
 * @brief   Wait up
 *
 * @param   void
 *
 * @return  none
 */
#pragma vector = P0INT_VECTOR
__interrupt void P0_ISR(void)
{
  HAL_ENTER_ISR();
  if(0x04 & P0IFG)              //判断 是否是 P0.2 RX 引脚 中断
  {
    BLE_Power_ForceWakeup();
  }
  P0IFG = 0;                    //清中断标志
  P0IF = 0;                     //清中断标志，IRCON[5],P0口中断
  HAL_EXIT_ISR();
}

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
  if(pwrmgr_IsSleep == true){
    P0IFG &= ~(0x04);           //清中断标志
    P0IEN &= ~(0x04);           //P0.2 RX 中断使能
    IEN1  &= ~(0x20);           //端口P0中断关闭
    U0CSR  &= ~(0x04);         //开 串口 接收使能
      
    pwrmgr_IsSleep = false;;
    CLEAR_SLEEP_MODE();//退出 休眠 ，进入工作状态
    HAL_BOARD_INIT();//切换到外部32M 晶振 并且 等待稳定
    
    
    HCI_EXT_ClkDivOnHaltCmd( HCI_EXT_DISABLE_CLK_DIVIDE_ON_HALT );
    HCI_EXT_HaltDuringRfCmd( HCI_EXT_HALT_DURING_RF_DISABLE );
      

    NPI_InitTransport(BLE_UART_Receive); 
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
  if(pwrmgr_IsSleep == false){
    P0IFG &= ~(0x04);           //P0.2 清除中断标记
    PICTL &= ~(0x04);             //P0端口下降沿触发
    P0IEN |=  (0x04);             //P0.2 RX 中断使能
    IEN1  |=  (0x20);             //端口P0中断使能
    U0CSR  &= ~(0x04);            //关闭 串口 接收使能
    
    HCI_EXT_ClkDivOnHaltCmd( HCI_EXT_ENABLE_CLK_DIVIDE_ON_HALT );
    HCI_EXT_HaltDuringRfCmd( HCI_EXT_HALT_DURING_RF_ENABLE );
    pwrmgr_IsSleep = true;
  }
}


/*********************************************************************
 * @fn      BLE_UART_Receive
 *
 * @brief   UART Cakkback
 *
 * @param   ......
 *
 * @return  none
 */
void BLE_UART_Receive( uint8 port, uint8 events )
{
  (void)port;
  if (events & (HAL_UART_RX_TIMEOUT | HAL_UART_RX_FULL))
  {
    //Read all data from UART buffer
    uint8 CurrentBufferRLen = NPI_RxBufLen();
    uint8* CurrentBuffer = osal_mem_alloc(CurrentBufferRLen);
    NPI_ReadTransport(CurrentBuffer, CurrentBufferRLen);
    if(BLE_CommFunc.UartReceive != 0){
      BLE_CommFunc.UartReceive( CurrentBuffer, CurrentBufferRLen);
    }
    osal_mem_free(CurrentBuffer);
  }
}

/*********************************************************************
 * @fn      BLE_UART_Send
 *
 * @brief   
 *
 * @param   ......
 *
 * @return  none
 */
void BLE_UART_Send( uint8* Data, uint8 DataLen )
{
  NPI_WriteTransport(Data, DataLen);
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
  if(osal_snv_read(BLE_CP_Flash_Idx, sizeof(BLE_Editable_CP), (uint8*)&BLE_Editable_CP) != SUCCESS || 
  BLE_Editable_CP.INTER_VERSION != BLE_Editable_CP.INTER_VERSION){      //Check whether data is valid
    BLE_Editable_CP.CP = BLEMode_Peripheral;
    BLE_Editable_CP.INTER_VERSION = BLE_Parm_INTER_VERSION;
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
    GAPCentralRole_Init( taskID++ );
  } else if(BLE_Editable_CP.CP == BLEMode_Peripheral){
    GAPRole_Init( taskID++ );
  }
  
  GAPBondMgr_Init( taskID++ );
  GATTServApp_Init( taskID++ );

  /* Application */
  
  NPI_InitTransport(BLE_UART_Receive);
  if(BLE_Editable_CP.CP == BLEMode_Central){
    BLEC_Init(taskID++, &BLE_CommFunc);
  } else if(BLE_Editable_CP.CP == BLEMode_Peripheral){
    BLEP_Init(taskID++, &BLE_CommFunc);
  }
}

/*********************************************************************
*********************************************************************/
