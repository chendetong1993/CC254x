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

#define APP_READ_RSSI_INTERVAL              60

#define APP_BleCmd_Hook_Receive(_D, _DL)                                \
  if(APP_BleCmd_Hook(true, _D, _DL, true)){                             \
    APP_BleCmd_Receive(_D, _DL);                                        \
  }                                                                     \
    
/*********************************************************************
* TYPEDEFS
*/

/*********************************************************************
 * LOCAL FUNCTIONS
 */
void APP_BleCmd_Send( uint8*, uint8 );
void (*APP_BleCmd_Receive)(uint8*, uint8);
bool (*APP_Info_Set)(uint8*, uint8);
void (*APP_Info_Get)(uint8**, uint8*);
void APP_Init( uint8 );
uint16 APP_ProcessEvent( uint8, uint16 );
void APP_Enter_Sleep();
void APP_Enter_Wakeup();

bool APP_BleCmd_Hook(bool, uint8*, uint8, bool);
void APP_Sleep();
/*********************************************************************
 * LOCAL Variables
 */

uint16 APP_TaskId = 0;
uint8 APP_CInfo[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09};
uint8 APP_CInfoLen = BLE_Array_Len(APP_CInfo);
          
uint8 APP_Name[] = {'A', 'P', 'P', '-', 'R', 'S', 'S', 'I' };
uint8 APP_NameLen = BLE_Array_Len(APP_Name);

uint8 APP_BleConnHandle = 0x00;

uint8* APP_SendBuf;
uint8 APP_SendBufLen;
  
bool APP_Inited = false;
/*********************************************************************
 * PUBLIC FUNCTIONS
 */
   

/*********************************************************************
 * @fn      APP_BleCmd_Hook
 *
 * @brief   
 *
 * @param   ......
 *
 * @return  none
 */
bool APP_BleCmd_Hook(bool ToBle, uint8* Data, uint8 DataLen, bool CheckValid){
  uint8 Role, Type, ExtLen;
  uint8* Ext;
  if(BLE_CmdParse(Data, DataLen, &Role, &Type, &Ext, &ExtLen, CheckValid)){
    if(Type == BLE_MsgType_Device_Inited){
      if(!ToBle) {
        uint8* PInfo;
        uint8 PInfoLen;
        APP_Info_Get(&PInfo, &PInfoLen);
        if(!BLE_Array_Compare(PInfo, PInfoLen, APP_CInfo, APP_CInfoLen)){
          if(Role != BLE_CMD_Msg_RelDev_P){
            if(BLE_CmdStringify_Type(&APP_SendBuf, &APP_SendBufLen, BLE_MsgType_SwitchRole)){
              APP_BleCmd_Receive(APP_SendBuf, APP_SendBufLen);
            }
            return false;
          }
          if(BLE_CmdStringify_Type_8_32(&APP_SendBuf, &APP_SendBufLen, BLE_MsgType_Parms_Set, BLEP_SetParm_ENABLE_TRANSMIT_ENCRYPT, false)){
            APP_BleCmd_Receive(APP_SendBuf, APP_SendBufLen);
          }
          if(BLE_CmdStringify_Type_8_N(&APP_SendBuf, &APP_SendBufLen, BLE_MsgType_Parms_Set, BLEP_SetParm_NAME, APP_Name, APP_NameLen)){
            APP_BleCmd_Receive(APP_SendBuf, APP_SendBufLen);
          }
          if(BLE_CmdStringify_Type_8_32(&APP_SendBuf, &APP_SendBufLen, BLE_MsgType_Parms_Set, BLEP_SetParm_ENABLE_CMD_CHECK_BIT, false)){
            APP_BleCmd_Receive(APP_SendBuf, APP_SendBufLen);
          }
          if(BLE_CmdStringify_Type(&APP_SendBuf, &APP_SendBufLen, BLE_MsgType_Reboot)){
            APP_BleCmd_Receive(APP_SendBuf, APP_SendBufLen);
          }
          APP_Info_Set(APP_CInfo, APP_CInfoLen);
          return false;
        } else {
          if(APP_Inited == false){
            APP_Enter_Wakeup();
            APP_Inited = true;
          }
          return true;
        }
      }
    }
    if(APP_Inited == false) return false;
  }
  return true;
}

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
  if(APP_BleCmd_Hook(false, Data, DataLen, false) == false) return;
  uint8 Role, Type, ExtLen;
  uint8* Ext;
  if(BLE_CmdParse(Data, DataLen, &Role, &Type, &Ext, &ExtLen, false)){
    switch(Type){
      case BLE_MsgType_AdditInfo_Ret:
        {
          uint8 Type0, Type1;
          uint32 Val;
          if(BLE_CmdExtParse_8_8_32(Ext, ExtLen, &Type0, &Type1, &Val, false)){
            switch(Type0){
              case BLE_AddRet_ReadRssi:
                {
                  //if(Type1 == 0)
                  if(BLE_CmdStringify_Type_8_32(&APP_SendBuf, &APP_SendBufLen, BLE_MsgType_Send, APP_BleConnHandle, Val)){
                    APP_BleCmd_Hook_Receive(APP_SendBuf, APP_SendBufLen);
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
          if(BLE_CmdExtParse_N(Ext, ExtLen, &DevConnHandles, &DevConnNum, false)){
            HalLedSet(HAL_LED_CHANNEL_P1_0, DevConnHandles[0]);
          }
        }
        break;
      case BLE_MsgType_Received:
        {
          uint8 connHandle;
          uint8* Rec;
          uint8 RecLen;
          if(BLE_CmdExtParse_8_N(Ext, ExtLen, &connHandle, &Rec, &RecLen, false)){
            if(BLE_CmdStringify_Type_8_N(&APP_SendBuf, &APP_SendBufLen, BLE_MsgType_Send, connHandle, Rec, RecLen)){
              APP_BleCmd_Hook_Receive(APP_SendBuf, APP_SendBufLen);
            }
          }
        }
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
  osal_stop_timerEx( APP_TaskId, APP_READ_RSSI_EVT);
  if(BLE_CmdStringify_Type(&APP_SendBuf, &APP_SendBufLen, BLE_MsgType_Disconnect_DisAdvert)){
    APP_BleCmd_Hook_Receive(APP_SendBuf, APP_SendBufLen);
  }
  HalLedSet(HAL_LED_CHANNEL_P1_1, false);
  HalInterruptSet(HAL_INTERRUPT_CHANNEL_P0_1, 0);
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
  if(BLE_CmdStringify_Type(&APP_SendBuf, &APP_SendBufLen, BLE_MsgType_Connect_EnAdvert)){
    APP_BleCmd_Hook_Receive(APP_SendBuf, APP_SendBufLen);
  }
  osal_start_reload_timer( APP_TaskId, APP_READ_RSSI_EVT, APP_READ_RSSI_INTERVAL);
  HalLedSet(HAL_LED_CHANNEL_P1_1, true);
  HalInterruptSet(HAL_INTERRUPT_CHANNEL_P0_1, APP_Sleep);
}

void APP_Init(uint8 task_id){
  APP_TaskId = task_id;
}

uint16 APP_ProcessEvent(uint8 task_id, uint16 events){
  if(events & APP_READ_RSSI_EVT){
    if(BLE_CmdStringify_Type_8_8(&APP_SendBuf, &APP_SendBufLen, BLE_MsgType_AdditOper, BLE_AddRet_ReadRssi, APP_BleConnHandle)){
      APP_BleCmd_Hook_Receive(APP_SendBuf, APP_SendBufLen);
    }
    return ( events ^ APP_READ_RSSI_EVT );
  }
  return 0;
}

void APP_Sleep(){
  if(BLE_CmdStringify_Type(&APP_SendBuf, &APP_SendBufLen, BLE_MsgType_Sleep)){
    APP_BleCmd_Hook_Receive(APP_SendBuf, APP_SendBufLen);
  }
}
/*********************************************************************
*********************************************************************/
#endif