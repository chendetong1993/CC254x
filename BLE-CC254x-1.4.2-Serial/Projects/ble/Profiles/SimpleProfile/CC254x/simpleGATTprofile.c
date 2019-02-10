/**************************************************************************************************
  Filename:       simpleGATTprofile.c
  Revised:        $Date: 2013-05-06 13:33:47 -0700 (Mon, 06 May 2013) $
  Revision:       $Revision: 34153 $

  Description:    This file contains the Simple GATT profile sample GATT service 
                  profile for use with the BLE sample application.

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
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "gapbondmgr.h"

#include "simpleGATTprofile.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

#define SERVAPP_NUM_ATTR_SUPPORTED        8

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
// Simple GATT Profile Service UUID: 0xFFF0
CONST uint8 simpleProfileServUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(SIMPLEPROFILE_SERV_UUID), HI_UINT16(SIMPLEPROFILE_SERV_UUID)
};

// Characteristic SC UUID: 0xEEF6
CONST uint8 simpleProfileCharSCUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(SIMPLEPROFILE_CHAR_SC_UUID), HI_UINT16(SIMPLEPROFILE_CHAR_SC_UUID)
};

// Characteristic CS UUID: 0xEEF7
CONST uint8 simpleProfileCharCSUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(SIMPLEPROFILE_CHAR_CS_UUID), HI_UINT16(SIMPLEPROFILE_CHAR_CS_UUID)
};

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

static simpleProfileCBs_t *simpleProfile_AppCBs = NULL;

/*********************************************************************
 * Profile Attributes - variables
 */

// Simple Profile Service attribute
static CONST gattAttrType_t simpleProfileService = { ATT_BT_UUID_SIZE, simpleProfileServUUID };

// Simple Profile Characteristic SC Properties
static uint8 simpleProfileCharSCProps = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic SC Value
static uint8 simpleProfileCharSC[SIMPLEPROFILE_CHAR_SC_CS_LEN] = { 0 };
static uint8 simpleProfileCharSCReadLen = 0;
static uint8 simpleProfileCharSCWriteLen = 0;

// Simple Profile Characteristic SC User Description
static uint8 simpleProfileCharSCUserDesp[] = "Server To Client\0";


// Simple Profile Characteristic CS Properties
static uint8 simpleProfileCharCSProps = GATT_PROP_NOTIFY;

// Characteristic CS Value
static uint8 simpleProfileCharCS[SIMPLEPROFILE_CHAR_SC_CS_LEN] = { 0};
static uint8 simpleProfileCharCSReadLen = 0;
static uint8 simpleProfileCharCSWriteLen = 0;

// Simple Profile Characteristic CS Configuration Each client has its own
// instantiation of the Client Characteristic Configuration. Reads of the
// Client Characteristic Configuration only shows the configuration for
// that client and writes only affect the configuration of that client.
static gattCharCfg_t *simpleProfileCharCSConfig;

// Simple Profile Characteristic CS User Description
static uint8 simpleProfileCharCSUserDesp[] = "Client To Server\0";

/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t simpleProfileAttrTbl[SERVAPP_NUM_ATTR_SUPPORTED] = 
{
  // Simple Profile Service
  { 
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8 *)&simpleProfileService            /* pValue */
  },
  // Characteristic SC Declaration
  { 
    { ATT_BT_UUID_SIZE, characterUUID },
    GATT_PERMIT_READ, 
    0,
    &simpleProfileCharSCProps 
  },
  // Characteristic Value SC
  { 
    { ATT_BT_UUID_SIZE, simpleProfileCharSCUUID },
    GATT_PERMIT_READ | GATT_PERMIT_WRITE,
    0, 
    simpleProfileCharSC 
  },
  // Characteristic SC User Description
  { 
    { ATT_BT_UUID_SIZE, charUserDescUUID },
    GATT_PERMIT_READ, 
    0, 
    simpleProfileCharSCUserDesp 
  },
  // Characteristic CS Declaration
  { 
    { ATT_BT_UUID_SIZE, characterUUID },
    GATT_PERMIT_READ, 
    0,
    &simpleProfileCharCSProps 
  },
  // Characteristic Value CS
  { 
    { ATT_BT_UUID_SIZE, simpleProfileCharCSUUID },
    0,
    0, 
    simpleProfileCharCS 
  },
  // Characteristic CS configuration
  { 
    { ATT_BT_UUID_SIZE, clientCharCfgUUID },
    GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
    0, 
    (uint8 *)&simpleProfileCharCSConfig 
  },
  // Characteristic CS User Description
  { 
    { ATT_BT_UUID_SIZE, charUserDescUUID },
    GATT_PERMIT_READ, 
    0, 
    simpleProfileCharCSUserDesp 
  },
};


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static uint8 simpleProfile_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                            uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen, uint8 method );
static bStatus_t simpleProfile_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                 uint8 *pValue, uint8 len, uint16 offset , uint8 method);

static void simpleProfile_HandleConnStatusCB( uint16 connHandle, uint8 changeType );


/*********************************************************************
 * PROFILE CALLBACKS
 */
// Simple Profile Service Callbacks
CONST gattServiceCBs_t simpleProfileCBs =
{
  simpleProfile_ReadAttrCB,  // Read callback function pointer
  simpleProfile_WriteAttrCB, // Write callback function pointer
  NULL                       // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleProfile_AddService
 *
 * @brief   Initializes the Simple Profile service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  Success or Failure
 */
bStatus_t SimpleProfile_AddService( uint32 services )
{
  uint8 status = SUCCESS;

  // Allocate Client Characteristic Configuration table
  simpleProfileCharCSConfig = (gattCharCfg_t *)osal_mem_alloc( sizeof(gattCharCfg_t) * linkDBNumConns );
  if ( simpleProfileCharCSConfig == NULL )
  {     
    return ( bleMemAllocError );
  }
  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, simpleProfileCharCSConfig );
  // Register with Link DB to receive link status change callback
  VOID linkDB_Register( simpleProfile_HandleConnStatusCB );  
  if ( services & SIMPLEPROFILE_SERVICE )
  {
    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService( simpleProfileAttrTbl, 
                                          GATT_NUM_ATTRS( simpleProfileAttrTbl ),
                                          GATT_MAX_ENCRYPT_KEY_SIZE,
                                          &simpleProfileCBs );
  }
  return ( status );
}


/*********************************************************************
 * @fn      SimpleProfile_RegisterAppCBs
 *
 * @brief   Registers the application callback function. Only call 
 *          this function once.
 *
 * @param   callbacks - pointer to application callbacks.
 *
 * @return  SUCCESS or bleAlreadyInRequestedMode
 */
bStatus_t SimpleProfile_RegisterAppCBs( simpleProfileCBs_t *appCallbacks )
{
  if ( appCallbacks )
  {
    simpleProfile_AppCBs = appCallbacks;
    return ( SUCCESS );
  }
  else
  {
    return ( bleAlreadyInRequestedMode );
  }
}
  
/*********************************************************************
 * @fn      SimpleProfile_WriteCharValue
 *
 * @brief   SimpleProfile_WriteCharValue
 *
 * @param   taskId
 * @return  bStatus_t
 */
bStatus_t SimpleProfile_WriteCharValue(uint8 taskId, uint16 connHandle, uint16 handle, void *value, uint8 len )
{
  bStatus_t ret = bleInvalidRange;
  if(len <= SIMPLEPROFILE_CHAR_SC_CS_LEN){
    attWriteReq_t AttReq;
    uint16 LenCheck;
    AttReq.pValue = GATT_bm_alloc( connHandle, ATT_WRITE_REQ, len, &LenCheck );
    if(LenCheck == len){
      AttReq.handle = handle;
      AttReq.len = len;
      AttReq.sig = 0;
      AttReq.cmd = 0;
      osal_memcpy(AttReq.pValue, value, len);
      ret = GATT_WriteCharValue(connHandle, &AttReq, taskId);
    }
    if(ret != SUCCESS){
      GATT_bm_free( (gattMsg_t *)&AttReq, ATT_WRITE_REQ );
      ret = bleInvalidRange;
    }
  } else {
    ret = bleInvalidRange;
  }
  return ret;
}


/*********************************************************************
 * @fn      SimpleProfile_SetParameter
 *
 * @brief   Set a Simple Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to right
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t SimpleProfile_SetParameter( uint8 param, void *value, uint8 len )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case SIMPLEPROFILE_CHAR_SC:
      if (len <= SIMPLEPROFILE_CHAR_SC_CS_LEN ) 
      {
        simpleProfileCharSCReadLen = len;
        VOID osal_memcpy(simpleProfileCharSC, value, len);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;
    case SIMPLEPROFILE_CHAR_CS:
      if ( len <= SIMPLEPROFILE_CHAR_SC_CS_LEN ) 
      {
        simpleProfileCharCSReadLen = len;
        VOID osal_memcpy( simpleProfileCharCS, value, len );
        // See if Notification has been enabled
        if(GATTServApp_ProcessCharCfg( simpleProfileCharCSConfig, simpleProfileCharCS, FALSE,
        simpleProfileAttrTbl, GATT_NUM_ATTRS( simpleProfileAttrTbl ),
        INVALID_TASK_ID, simpleProfile_ReadAttrCB ) != SUCCESS) {
          ret = bleInvalidRange;
        }
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;
    default:
      ret = INVALIDPARAMETER;
      break;
  }
  return ( ret );
}

/*********************************************************************
 * @fn      SimpleProfile_GetParameter
 *
 * @brief   Get a Simple Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to put.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t SimpleProfile_GetParameter( uint8 param, uint8 **value, uint8** len )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case SIMPLEPROFILE_CHAR_SC:
      (*len) = &simpleProfileCharSCWriteLen;
      (*value) = simpleProfileCharSC;
      break;   
    case SIMPLEPROFILE_CHAR_CS:
      (*len) = &simpleProfileCharCSWriteLen;
      (*value) = simpleProfileCharCS;
      break;
    default:
      ret = INVALIDPARAMETER;
      break;
  }
  return ( ret );
}

/*********************************************************************
 * @fn          simpleProfile_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 *
 * @return      Success or Failure
 */
static bStatus_t simpleProfile_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                            uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen, uint8 method )
{
  bStatus_t status = SUCCESS;
  // If attribute permissions require authorization to read, return error
  if ( gattPermitAuthorRead( pAttr->permissions ) )
  {
    // Insufficient authorization
    return ( ATT_ERR_INSUFFICIENT_AUTHOR );
  }
  // Make sure it's not a blob operation (no attributes in the profile are long)
  if ( offset > 0 )
  {
    return ( ATT_ERR_ATTR_NOT_LONG );
  }
  if ( pAttr->type.len == ATT_BT_UUID_SIZE )
  {
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid )
    {
      // No need for "GATT_SERVICE_UUID" or "GATT_CLIENT_CHAR_CFG_UUID" cases;
      // gattserverapp handles those reads
      case SIMPLEPROFILE_CHAR_SC_UUID:
        (*pLen) = simpleProfileCharSCReadLen;
        VOID osal_memcpy( pValue, pAttr->pValue, *pLen );
        break;
      case SIMPLEPROFILE_CHAR_CS_UUID:
        (*pLen) = simpleProfileCharCSReadLen;
        VOID osal_memcpy( pValue, pAttr->pValue, *pLen );
        break;     
      default:
        // Should never get here! (characteristics 3 and 4 do not have read permissions)
        (*pLen) = 0;
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  else
  {
    // 128-bit UUID
    *pLen = 0;
    status = ATT_ERR_INVALID_HANDLE;
  }

  return ( status );
}

/*********************************************************************
 * @fn      simpleProfile_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 *
 * @return  Success or Failure
 */
static bStatus_t simpleProfile_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                 uint8 *pValue, uint8 len, uint16 offset, uint8 method )
{
  bStatus_t status = SUCCESS;
  uint8 notifyApp = 0xFF;
  
  // If attribute permissions require authorization to write, return error
  if ( gattPermitAuthorWrite( pAttr->permissions ) )
  {
    // Insufficient authorization
    return ( ATT_ERR_INSUFFICIENT_AUTHOR );
  }
  
  if ( pAttr->type.len == ATT_BT_UUID_SIZE )
  {
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid )
    {
      case SIMPLEPROFILE_CHAR_SC_UUID:
        //Validate the value
        // Make sure it's not a blob oper
        if ( offset == 0 )
        {
          if ( len > SIMPLEPROFILE_CHAR_SC_CS_LEN )
          {
            status = ATT_ERR_INVALID_VALUE_SIZE;
          }
        }
        else
        {
          status = ATT_ERR_ATTR_NOT_LONG;
        }
        //Write the value
        if ( status == SUCCESS )
        {
	  VOID osal_memcpy( pAttr->pValue, pValue, len );
          simpleProfileCharSCWriteLen = len;
          notifyApp = SIMPLEPROFILE_CHAR_SC;
        }
        break;
      case SIMPLEPROFILE_CHAR_CS_UUID:
        //Validate the value
        // Make sure it's not a blob oper
        if ( offset == 0 )
        {
          if ( len > SIMPLEPROFILE_CHAR_SC_CS_LEN )
          {
            status = ATT_ERR_INVALID_VALUE_SIZE;
          }
        }
        else
        {
          status = ATT_ERR_ATTR_NOT_LONG;
        }
        //Write the value
        if ( status == SUCCESS )
        {
	  VOID osal_memcpy( pAttr->pValue, pValue, len );
          simpleProfileCharCSWriteLen = len;
          notifyApp = SIMPLEPROFILE_CHAR_CS;
        }
        break;
      case GATT_CLIENT_CHAR_CFG_UUID:
        status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                                 offset, GATT_CLIENT_CFG_NOTIFY );
        break;
      default:
        // Should never get here! (characteristics 2 and 4 do not have write permissions)
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  else
  {
    // 128-bit UUID
    status = ATT_ERR_INVALID_HANDLE;
  }
  // If a charactersitic value changed then callback function to notify application of change
  if ( (notifyApp != 0xFF ) && simpleProfile_AppCBs && simpleProfile_AppCBs->pfnSimpleProfileChange )
  {
    simpleProfile_AppCBs->pfnSimpleProfileChange( notifyApp );  
  }
  return ( status );
}

/*********************************************************************
 * @fn          simpleProfile_HandleConnStatusCB
 *
 * @brief       Simple Profile link status change handler function.
 *
 * @param       connHandle - connection handle
 * @param       changeType - type of change
 *
 * @return      none
 */
static void simpleProfile_HandleConnStatusCB( uint16 connHandle, uint8 changeType )
{ 
  // Make sure this is not loopback connection
  if ( connHandle != LOOPBACK_CONNHANDLE )
  {
    // Reset Client Char Config if connection has dropped
    if ( ( changeType == LINKDB_STATUS_UPDATE_REMOVED )      ||
         ( ( changeType == LINKDB_STATUS_UPDATE_STATEFLAGS ) && 
           ( !linkDB_Up( connHandle ) ) ) )
    { 
      GATTServApp_InitCharCfg( connHandle, simpleProfileCharCSConfig );
    }
  }
}
/*********************************************************************
*********************************************************************/
