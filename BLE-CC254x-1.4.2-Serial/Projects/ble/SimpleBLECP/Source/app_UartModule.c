/**************************************************************************************************
  Filename:       app.c
  Revised:        $Date: 2010-08-06 08:56:11 -0700 (Fri, 06 Aug 2010) $
  Revision:       $Revision: 23333 $

  Description:    
**************************************************************************************************/

#if defined ( APP_UART_MODULE )
/*********************************************************************
 * INCLUDES
 */
#include "hal_types.h"
#include "npi.h"
#include "OSAL.h"
#include "app_UartModule.h"

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
void APP_UART_Receive( uint8, uint8 );
void APP_BleCmd_Receive(void (*)(uint8*, uint8));
void APP_Init( uint8 );
uint16 APP_ProcessEvent( uint8, uint16 );
void APP_Enter_Sleep();
void APP_Enter_Wakeup();
/*********************************************************************
 * LOCAL Variables
 */

uint8 APP_UARTBufferRLen = 0;
uint8* APP_UARTBuffer = 0;
void (*APP_BleCmd_Rec_Callback)(uint8*, uint8);

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

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
  NPI_WriteTransport(Data, DataLen);
}

/*********************************************************************
 * @fn      APP_UART_Rec
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
void APP_UART_Receive( uint8 port, uint8 events )
{
  (void)port;
  if (events & (HAL_UART_RX_TIMEOUT | HAL_UART_RX_FULL))
  {
    //Read all data from UART buffer
    uint8 TLen = NPI_RxBufLen();
    if(TLen > APP_UARTBufferRLen){
      if(APP_UARTBuffer != 0){
        osal_mem_free(APP_UARTBuffer);
      }
      APP_UARTBufferRLen = TLen * 2;
      APP_UARTBuffer = osal_mem_alloc(APP_UARTBufferRLen);
    }
    NPI_ReadTransport(APP_UARTBuffer, TLen);
    APP_BleCmd_Rec_Callback( APP_UARTBuffer, TLen);
  }
}


void APP_Init(uint8 task_id){
  APP_Enter_Wakeup();
}

uint16 APP_ProcessEvent(uint8 task_id, uint16 events){
  return 0;
}
/*********************************************************************
*********************************************************************/

#endif