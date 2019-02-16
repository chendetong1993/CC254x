/**************************************************************************************************
  Filename:       app.c
  Revised:        $Date: 2010-08-06 08:56:11 -0700 (Fri, 06 Aug 2010) $
  Revision:       $Revision: 23333 $

  Description:    
**************************************************************************************************/

#if defined ( APP_UART_MODULE )

#if (defined(HAL_UART) && (HAL_UART != TRUE)) || (!defined(HAL_UART))
  HAL_UART Not Defined;
#endif

/*********************************************************************
 * INCLUDES
 */
#include "hal_types.h"
#include "npi.h"
#include "OSAL.h"
#include "app_UartModule.h"
#include "simpleBLE.h"
#include "simpleBLEP.h"
#include "hal_led.h"
/*********************************************************************
 * MACROS
 */
/*********************************************************************
 * CONSTANTS
 */
#define APP_UART_TIMEOUT_INTERVAL              30

#define APP_UART_TIMEOUT_EVT                   (0x0001 << 0)

#define APP_BleCmd_Hook_Receive(_D, _DL)                           \
  if(APP_BleCmd_Hook(true, _D, _DL, true)){                        \
    APP_BleCmd_Receive(_D, _DL);                                   \
  }

/*********************************************************************
* TYPEDEFS
*/

/*********************************************************************
 * LOCAL FUNCTIONS
 */
void APP_BleCmd_Send( uint8*, uint8 );
void APP_UART_Receive( uint8, uint8 );
void (*APP_BleCmd_Receive)(uint8*, uint8);
bool (*APP_Info_Set)(uint8*, uint8);
void (*APP_Info_Get)(uint8**, uint8*);
void APP_Init( uint8 );
uint16 APP_ProcessEvent( uint8, uint16 );
void APP_Enter_Sleep();
void APP_Enter_Wakeup();
bool APP_BleCmd_Hook(bool, uint8*, uint8, bool);

/*********************************************************************
 * LOCAL Variables
 */
uint16 APP_TaskId = 0;
uint8 APP_UARTBuf_RecLen = 0;
uint8 APP_UARTBuf[BLE_CMD_Msg_Max_Len];

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
  return true;
}

/*********************************************************************
 * @fn      APP_UART_Send
 *
 * @brief   
 *
 * @param   ......
 *
 * @return  none
 */
void APP_BleCmd_Send( uint8* Data, uint8 DataLen )
{
  if(APP_BleCmd_Hook(false, Data, DataLen, false)){
    NPI_WriteTransport(Data, DataLen);
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
  HalLedSet(HAL_LED_CHANNEL_P1_0, false);
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
  NPI_InitTransport(APP_UART_Receive);
  HalLedSet(HAL_LED_CHANNEL_P1_0, true);
  HalLedSet(HAL_LED_CHANNEL_P1_1, true);
}

/*********************************************************************
 * @fn      APP_UART_Receive
 *
 * @brief   
 *
 * @param   ......
 *
 * @return  none
 */
void APP_UART_Receive( uint8 port, uint8 events ){
  (void)port;
  if (events & (HAL_UART_RX_TIMEOUT | HAL_UART_RX_FULL)){
    osal_stop_timerEx( APP_TaskId, APP_UART_TIMEOUT_EVT);
    uint8 RxBufLen = NPI_RxBufLen();
    uint8 ReadLen;
    while(RxBufLen > 0){
      if(APP_UARTBuf_RecLen == 0){
        NPI_ReadTransport(&APP_UARTBuf[0], 1);
        RxBufLen -= 1;
        if(APP_UARTBuf[0] > BLE_CMD_Msg_Max_Len || APP_UARTBuf[0] == 0){
          APP_BleCmd_Hook_Receive(0, 0);
          continue;
        }
        APP_UARTBuf_RecLen = 1;
      }
      if((APP_UARTBuf_RecLen + RxBufLen) < APP_UARTBuf[0]){
        NPI_ReadTransport(&APP_UARTBuf[APP_UARTBuf_RecLen], RxBufLen);
        APP_UARTBuf_RecLen += RxBufLen;
        osal_start_timerEx( APP_TaskId, APP_UART_TIMEOUT_EVT, APP_UART_TIMEOUT_INTERVAL);
        break;
      } else {
        ReadLen = APP_UARTBuf[0] - APP_UARTBuf_RecLen;
        NPI_ReadTransport(&APP_UARTBuf[APP_UARTBuf_RecLen], ReadLen);
        APP_UARTBuf_RecLen = 0;
        RxBufLen -= ReadLen;

        APP_BleCmd_Hook_Receive(APP_UARTBuf, APP_UARTBuf[0]);
      }
    }
  }
}

void APP_Init(uint8 task_id){
  APP_TaskId = task_id;
  APP_Enter_Wakeup();
}

uint16 APP_ProcessEvent(uint8 task_id, uint16 events){
  if (events & APP_UART_TIMEOUT_EVT) {
    APP_UARTBuf_RecLen = 0;
    APP_BleCmd_Hook_Receive(0, 0);
    return ( events ^ APP_UART_TIMEOUT_EVT );
  }
  return 0;
}
/*********************************************************************
*********************************************************************/

#endif