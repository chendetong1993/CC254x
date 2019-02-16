/**************************************************************************************************
  Filename:       app.c
  Revised:        $Date: 2010-08-06 08:56:11 -0700 (Fri, 06 Aug 2010) $
  Revision:       $Revision: 23333 $

  Description:    

**************************************************************************************************/

#if defined ( APP_KEY )
/*********************************************************************
 * INCLUDES
 */
#include "hal_types.h"
#include "OSAL.h"
#include "app_BatteryCheck.h"
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

/*********************************************************************
* TYPEDEFS
*/

/*********************************************************************
 * LOCAL FUNCTIONS
 */
void APP_BleCmd_Send( uint8*, uint8 );
void (*APP_BleCmd_Receive)(uint8*, uint8);
void APP_Init( uint8 );
uint16 APP_ProcessEvent( uint8, uint16 );
void APP_Enter_Sleep();
void APP_Enter_Wakeup();

void APP_KEY1_Down();
void APP_KEY2_Down();
/*********************************************************************
 * LOCAL Variables
 */


uint8 APP_CurrInfo[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09};
uint8 APP_CurrInfoLen = BLE_Array_Len(APP_CurrInfo);

uint8 APP_Name[] = {'A', 'P', 'P', '-', 'K', 'E', 'Y'};
uint8 APP_NameLen = BLE_Array_Len(APP_Name);
          
uint8* APP_USEBLE_Cmd; 
uint8 APP_USEBLE_CmdLen;

uint8 APP_BleConnHandle = 0x00;

bool APP_Led1On = true;
              
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
        APP_BleCmd_Receive(APP_USEBLE_Cmd, APP_USEBLE_CmdLen);
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
                APP_BleCmd_Receive(APP_USEBLE_Cmd, APP_USEBLE_CmdLen);
              }
              if(BLE_CmdStringify_Type_8_N(&APP_USEBLE_Cmd, &APP_USEBLE_CmdLen, BLE_MsgType_Parms_Set, BLEP_SetParm_NAME, APP_Name, APP_NameLen)){
                APP_BleCmd_Receive(APP_USEBLE_Cmd, APP_USEBLE_CmdLen);
              }
              if(BLE_CmdStringify_Type_8_32(&APP_USEBLE_Cmd, &APP_USEBLE_CmdLen, BLE_MsgType_Parms_Set, BLEP_SetParm_ENABLE_CMD_CHECK_BIT, false)){
                APP_BleCmd_Receive(APP_USEBLE_Cmd, APP_USEBLE_CmdLen);
              }
              if(BLE_CmdStringify_Type_8_N(&APP_USEBLE_Cmd, &APP_USEBLE_CmdLen, BLE_MsgType_Parms_Set, BLEP_SetParm_INFO, APP_CurrInfo, APP_CurrInfoLen)){
                APP_BleCmd_Receive(APP_USEBLE_Cmd, APP_USEBLE_CmdLen);
              }
              if(BLE_CmdStringify_Type(&APP_USEBLE_Cmd, &APP_USEBLE_CmdLen, BLE_MsgType_Reboot)){
                APP_BleCmd_Receive(APP_USEBLE_Cmd, APP_USEBLE_CmdLen);
              }
            }
          }
        }
        break;
      case BLE_MsgType_ConnStatus_Returned:
        {
          uint8 *DevConnHandles;
          uint8 DevConnNum;
          if(BLE_CmdExtParse_N(Ext, ExtLen, &DevConnHandles, &DevConnNum)){
            if(DevConnHandles[0]){
              HalLedSet(HAL_LED_CHANNEL_P1_0, true);
              HalLedSet(HAL_LED_CHANNEL_P1_1, true);
            } else {
              HalLedSet(HAL_LED_CHANNEL_P1_0, false);
              HalLedSet(HAL_LED_CHANNEL_P1_1, false);
            }
          }
        }
        break;
      case BLE_MsgType_Sended:
        {
          uint8 connHandle;
          if(BLE_CmdExtParse_8(Ext, ExtLen, &connHandle)){
            HalLedSet(HAL_LED_CHANNEL_P1_0, APP_Led1On);
            HalLedSet(HAL_LED_CHANNEL_P1_1, !APP_Led1On);
          }
        }
        break;
      case BLE_MsgType_UnSended:
        {
          uint8 connHandle;
          if(BLE_CmdExtParse_8(Ext, ExtLen, &connHandle)){
            HalLedSet(HAL_LED_CHANNEL_P1_0, false);
            HalLedSet(HAL_LED_CHANNEL_P1_1, false);
          }
        }
        break;
      case BLE_MsgType_Received:
        /*
        {
          uint8 connHandle;
          uint8* Rec;
          uint8 RecLen;
          if(BLE_CmdExtParse_8_N(Ext, ExtLen, &connHandle, &Rec, &RecLen)){}
        }
        */
        break;
    }
  }
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

}

void APP_Init(uint8 task_id){
  HalInterruptSet(HAL_INTERRUPT_CHANNEL_P0_0, APP_KEY1_Down);
  HalInterruptSet(HAL_INTERRUPT_CHANNEL_P0_1, APP_KEY2_Down);
}

uint16 APP_ProcessEvent(uint8 task_id, uint16 events){
  return 0;
}


void APP_KEY1_Down(){
  if(BLE_CmdStringify_Type_8_8(&APP_USEBLE_Cmd, &APP_USEBLE_CmdLen, BLE_MsgType_Send, APP_BleConnHandle, 1)){
    APP_BleCmd_Receive(APP_USEBLE_Cmd, APP_USEBLE_CmdLen);
    APP_Led1On = true;
  }
}

void APP_KEY2_Down(){
  if(BLE_CmdStringify_Type_8_8(&APP_USEBLE_Cmd, &APP_USEBLE_CmdLen, BLE_MsgType_Send, APP_BleConnHandle, 2)){
    APP_BleCmd_Receive(APP_USEBLE_Cmd, APP_USEBLE_CmdLen);
    APP_Led1On = false;
  }
}
/*********************************************************************
*********************************************************************/
#endif