/**************************************************************************************************
  Filename:       app.c
  Revised:        $Date: 2010-08-06 08:56:11 -0700 (Fri, 06 Aug 2010) $
  Revision:       $Revision: 23333 $

  Description:    

**************************************************************************************************/

#if defined ( APP_RSSI_CHECK )
/*********************************************************************
 * INCLUDES
 */
#include "hal_types.h"
#include "OSAL.h"
#include "app_RssiCheck.h"
#include "simpleBLE.h"
#include "simpleBLEP.h"
#include "hal_led.h"
#include "hal_interrupt.h"

/*********************************************************************
 * MACROS
 */
/*********************************************************************
 * CONSTANTS
 */

#define APP_READ_RSSI_EVT                   (0x0001 << 0)

#define APP_READ_RSSI_INTERVAL              200

/*********************************************************************
* TYPEDEFS
*/

/*********************************************************************
 * LOCAL FUNCTIONS
 */
void APP_BleCmd_Send( uint8*, uint8 );
void APP_BleCmd_Receive(void (*)(uint8*, uint8));
void APP_Init( uint8 );
uint16 APP_ProcessEvent( uint8, uint16 );
void APP_Enter_Sleep();
void APP_Enter_Wakeup();

void APP_Sleep();
/*********************************************************************
 * LOCAL Variables
 */

void (*APP_BleCmd_Rec_Callback)(uint8*, uint8) = 0;

uint16 APP_TaskId = 0;
uint8 APP_CurrInfo[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09};
uint8 APP_CurrInfoLen = BLE_Array_Len(APP_CurrInfo);
          
uint8 APP_Name[] = {'A', 'P', 'P', '-', 'R', 'S', 'S', 'I' };
uint8 APP_NameLen = BLE_Array_Len(APP_Name);

uint8 APP_BleConnHandle = 0x00;

uint8* APP_USEBLE_Cmd; 
uint8 APP_USEBLE_CmdLen;
              
/*********************************************************************
 * PUBLIC FUNCTIONS
 */
   

/*********************************************************************
 * @fn      APP_BleCmd_Send
 *
 * @brief   
 *
 * @param   ......
 *
 * @return  none
 */
void APP_BleCmd_Send( uint8* Data, uint8 DataLen )
{
  uint8 Role;
  uint8 Type;
  uint8* Ext;
  uint8 ExtLen;
  if(BLE_CmdParse(Data, DataLen, &Role, &Type, &Ext, &ExtLen)){
    if(Role != BLE_CMD_Msg_RelDev_P){
      if(BLE_CmdStringify_Type(&APP_USEBLE_Cmd, &APP_USEBLE_CmdLen, BLE_MsgType_SwitchRole)){
        APP_BleCmd_Rec_Callback(APP_USEBLE_Cmd, APP_USEBLE_CmdLen);
      }
      return;
    }
    switch(Type){
      case BLE_MsgType_Device_Inited:
        {
          uint8 *PrevInfo;
          uint8 PrevInfoLen;
          if(BLE_CmdExtParse_N(Ext, ExtLen, &PrevInfo, &PrevInfoLen)){
            if(!BLE_Compare_Array(PrevInfo, PrevInfoLen, APP_CurrInfo, APP_CurrInfoLen)){
              if(BLE_CmdStringify_Type_8_32(&APP_USEBLE_Cmd, &APP_USEBLE_CmdLen, BLE_MsgType_Parms_Set, BLEP_SetParm_ENABLE_TRANSMIT_ENCRYPT, false)){
                APP_BleCmd_Rec_Callback(APP_USEBLE_Cmd, APP_USEBLE_CmdLen);
              }
              if(BLE_CmdStringify_Type_8_N(&APP_USEBLE_Cmd, &APP_USEBLE_CmdLen, BLE_MsgType_Parms_Set, BLEP_SetParm_NAME, APP_Name, APP_NameLen)){
                APP_BleCmd_Rec_Callback(APP_USEBLE_Cmd, APP_USEBLE_CmdLen);
              }
              if(BLE_CmdStringify_Type_8_N(&APP_USEBLE_Cmd, &APP_USEBLE_CmdLen, BLE_MsgType_Parms_Set, BLEP_SetParm_INFO, APP_CurrInfo, APP_CurrInfoLen)){
                APP_BleCmd_Rec_Callback(APP_USEBLE_Cmd, APP_USEBLE_CmdLen);
              }
              if(BLE_CmdStringify_Type(&APP_USEBLE_Cmd, &APP_USEBLE_CmdLen, BLE_MsgType_Reboot)){
                APP_BleCmd_Rec_Callback(APP_USEBLE_Cmd, APP_USEBLE_CmdLen);
              }
            }
          }
        }
        break;
      case BLE_MsgType_Hal_Ret:
        {
          uint8 Type0, Type1;
          uint32 Val;
          if(BLE_CmdExtParse_8_8_32(Ext, ExtLen, &Type0, &Type1, &Val)){
            switch(Type0){
              case BLE_HalRet_ReadRssi:
                {
                  //if(Type1 == 0)
                  if(BLE_CmdStringify_Type_8_32(&APP_USEBLE_Cmd, &APP_USEBLE_CmdLen, BLE_MsgType_Send, APP_BleConnHandle, Val)){
                    APP_BleCmd_Rec_Callback(APP_USEBLE_Cmd, APP_USEBLE_CmdLen);
                  }
                }
                break;
            }
          }
        }
        break;
      case BLE_MsgType_ConnStatus_Returned:
        {
          uint8 *DevConnHandles;
          uint8 DevConnNum;
          if(BLE_CmdExtParse_N(Ext, ExtLen, &DevConnHandles, &DevConnNum)){
            HalLedSet(HAL_LED_CHANNEL_P1_0, DevConnHandles[0]);
          }
        }
        break;
    }
  }
}

/*********************************************************************
 * @fn      APP_BleCmd_Receive
 *
 * @brief   
 *
 * @param   ......
 *
 * @return  none
 */
void APP_BleCmd_Receive(void (*Callback)(uint8*, uint8)){
  APP_BleCmd_Rec_Callback = Callback;
}

/*********************************************************************
 * @fn      APP_Enter_Sleep
 *
 * @brief   
 *
 * @param   ......
 *
 * @return  none
 */
void APP_Enter_Sleep(){
  osal_stop_timerEx( APP_TaskId, APP_READ_RSSI_EVT);
  if(BLE_CmdStringify_Type(&APP_USEBLE_Cmd, &APP_USEBLE_CmdLen, BLE_MsgType_Disconnect_DisAdvert)){
    APP_BleCmd_Rec_Callback(APP_USEBLE_Cmd, APP_USEBLE_CmdLen);
  }
  HalLedSet(HAL_LED_CHANNEL_P1_1, false);
}

/*********************************************************************
 * @fn      APP_Enter_Wakeup
 *
 * @brief   
 *
 * @param   ......
 *
 * @return  none
 */
void APP_Enter_Wakeup(){
  if(BLE_CmdStringify_Type(&APP_USEBLE_Cmd, &APP_USEBLE_CmdLen, BLE_MsgType_Connect_EnAdvert)){
    APP_BleCmd_Rec_Callback(APP_USEBLE_Cmd, APP_USEBLE_CmdLen);
  }
  osal_start_reload_timer( APP_TaskId, APP_READ_RSSI_EVT, APP_READ_RSSI_INTERVAL);
  HalLedSet(HAL_LED_CHANNEL_P1_1, true);
}

void APP_Init(uint8 task_id){
  APP_TaskId = task_id;
  HalInterruptSet(HAL_INTERRUPT_CHANNEL_P0_1, APP_Sleep);
  APP_Enter_Wakeup();
}

uint16 APP_ProcessEvent(uint8 task_id, uint16 events){
  if ( events & APP_READ_RSSI_EVT )
  {
    if(BLE_CmdStringify_Type_8_8(&APP_USEBLE_Cmd, &APP_USEBLE_CmdLen, BLE_MsgType_Hal_Set, BLE_HalRet_ReadRssi, APP_BleConnHandle)){
      APP_BleCmd_Rec_Callback(APP_USEBLE_Cmd, APP_USEBLE_CmdLen);
    }
    return ( events ^ APP_READ_RSSI_EVT );
  }
  return 0;
}

void APP_Sleep(){
  if(BLE_CmdStringify_Type(&APP_USEBLE_Cmd, &APP_USEBLE_CmdLen, BLE_MsgType_Sleep)){
    APP_BleCmd_Rec_Callback(APP_USEBLE_Cmd, APP_USEBLE_CmdLen);
  }
}
/*********************************************************************
*********************************************************************/
#endif