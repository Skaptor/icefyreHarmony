/*******************************************************************************
  FT5XX6 Driver Functions for Static Single Instance Driver

  Company:
    Microchip Technology Inc.

  File Name:
    drv_ft5xx6_static.c

  Summary:
    FT5XX6 driver impementation for the static single instance driver.

  Description:
    The FT5XX6 device driver provides a simple interface to manage the FT5XX6
    modules. This file contains implemenation for the FT5XX6 driver.
    
  Remarks:
    Static interfaces incorporate the driver instance number within the names
    of the routines, eliminating the need for an object ID or object handle.
    
    Static single-open interfaces also eliminate the need for the open handle.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2015 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*******************************************************************************/
//DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Include Files
// *****************************************************************************
// *****************************************************************************

#include "system_config.h"
#include "system_definitions.h"
#include "driver/touch/ft5xx6/src/drv_ft5xx6_static_local.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data
// *****************************************************************************
// *****************************************************************************

/* This is the driver instance object array. */
DRV_TOUCH_FT5XX6_STATIC_OBJ             gDrvFT5XX6StaticObj;
volatile uint16_t pointX=100;
volatile uint16_t pointY=100;
volatile uint16_t touch_x;
volatile uint16_t touch_y;
// *****************************************************************************
// *****************************************************************************
// Section: FT5XX6 Driver Interface Implementations
// *****************************************************************************
// *****************************************************************************

//******************************************************************************
/* Function:
    SYS_MODULE_OBJ DRV_TOUCH_FT5XX6_Initialize( const SYS_MODULE_INDEX   index, 
                                        const SYS_MODULE_INIT    * const init )

  Summary:
    Initializes hardware and data for the given instance of the FT5XX6 module.

  Description:
    This function initializes hardware for the instance of the FT5XX6 module,
    using the hardware initialization given data.  It also initializes all 
    necessary internal data.

  Parameters:

  Returns:
    If successful, it returns a valid handle to a driver instance object.
    Otherwise, it returns SYS_MODULE_OBJ_INVALID.
*/
 SYS_MODULE_OBJ DRV_TOUCH_FT5XX6_Initialize( const SYS_MODULE_INDEX   index, 
                                        const SYS_MODULE_INIT * const init )
{
    DRV_TOUCH_FT5XX6_STATIC_OBJ *dObj = (DRV_TOUCH_FT5XX6_STATIC_OBJ*)NULL;

    /* Allocate the driver object and set the operation flag to be in use */
    dObj = &gDrvFT5XX6StaticObj;

    /* Update driver object */
    dObj->inUse                 = true;
    dObj->queueSizeCurrent      = 0;
    dObj->touchQueueSizeCurrent = 0;
    dObj->queue                 = NULL;
    dObj->touchQueue            = NULL;
    dObj->deviceAddress         = DRV_TOUCH_FT5XX6_I2C_DEVICE_ADDRESS;
    dObj->queueSize             = 10;
    SYS_INT_SourceStatusClear(INT_SOURCE_EXTERNAL_2);

    /* Update the status */
    dObj->status = SYS_STATUS_READY;

    /* Return the driver handle */
    return( (SYS_MODULE_OBJ)dObj );

}

// *****************************************************************************
/* Function:
    void DRV_TOUCH_FT5XX6_Deinitialize( SYS_MODULE_OBJ object )

  Summary:
    Dynamic implementation of DRV_TOUCH_FT5XX6_Deinitialize system interface function.

  Description:
    This is the dynamic implementation of DRV_TOUCH_FT5XX6_Deinitialize system interface
    function.

  Remarks:
    See drv_ft5xx6.h for usage information.
*/

void  DRV_TOUCH_FT5XX6_Deinitialize( SYS_MODULE_OBJ object )
{
    DRV_TOUCH_FT5XX6_STATIC_OBJ *dObj = (DRV_TOUCH_FT5XX6_STATIC_OBJ*)NULL;

    dObj = (DRV_TOUCH_FT5XX6_STATIC_OBJ*) &gDrvFT5XX6StaticObj;

    if(!dObj->inUse)
    {
        SYS_DEBUG(0, "Invalid system object handle");
        return;
    }

    /* Indicate that this object is not is use */
    dObj->inUse = false;

    /* Deinitialize the FT5XX6 status */
    dObj->status =  SYS_STATUS_UNINITIALIZED ;

    /* Disable the interrupt */
    SYS_INT_SourceDisable(INT_SOURCE_EXTERNAL_2);

}

// *****************************************************************************
/* Function:
    SYS_STATUS DRV_TOUCH_FT5XX6_Status( SYS_MODULE_OBJ object )

  Summary:
    Dynamic implementation of DRV_TOUCH_FT5XX6_Status system interface function.

  Description:
    This is the dynamic implementation of DRV_TOUCH_FT5XX6_Status system interface
    function.

  Remarks:
    See drv_ft5xx6.h for usage information.
*/

SYS_STATUS DRV_TOUCH_FT5XX6_Status( SYS_MODULE_OBJ object )
{
    DRV_TOUCH_FT5XX6_STATIC_OBJ *dObj = (DRV_TOUCH_FT5XX6_STATIC_OBJ*)NULL;
    
    dObj = (DRV_TOUCH_FT5XX6_STATIC_OBJ*) &gDrvFT5XX6StaticObj;
    
    /* Return the system status of the hardware instance object */
    return (dObj->status);
}

// *****************************************************************************
/* Function:
    DRV_HANDLE DRV_TOUCH_FT5XX6_Open( const SYS_MODULE_INDEX drvIndex, 
                                const DRV_IO_INTENT intent )

  Summary:
    Dynamic implementation of DRV_TOUCH_FT5XX6_Open client interface function.

  Description:
    This is the dynamic implementation of DRV_TOUCH_FT5XX6_Open client interface
    function.

  Remarks:
    See drv_ft5xx6.h for usage information.
*/

DRV_HANDLE DRV_TOUCH_FT5XX6_Open( const SYS_MODULE_INDEX drvIndex, 
                              const DRV_IO_INTENT intent )
{
    DRV_TOUCH_FT5XX6_STATIC_OBJ *dObj = (DRV_TOUCH_FT5XX6_STATIC_OBJ*)NULL;
    
    dObj = (DRV_TOUCH_FT5XX6_STATIC_OBJ*) &gDrvFT5XX6StaticObj;

    if((dObj->status != SYS_STATUS_READY) || (dObj->inUse == false))
    {
        /* The FT5XX6 module should be ready */
        SYS_DEBUG(0, "Was the driver initialized?");
        return DRV_HANDLE_INVALID;
    }

    dObj->drvBusHandle = DRV_I2C_Open( DRV_I2C_INDEX_0, 
                                       DRV_IO_INTENT_READWRITE );
    if( DRV_HANDLE_INVALID == dObj->drvBusHandle )
    {
        return (DRV_HANDLE_INVALID);
    }

    DRV_I2C_BufferEventHandlerSet( dObj->drvBusHandle,
        (DRV_I2C_BUFFER_EVENT_HANDLER)_DRV_TOUCH_FT5XX6_I2C_EventHandler,
        (uintptr_t)dObj );

    if(!dObj->clientInUse)
    {
        dObj->clientInUse = true;
        
        dObj->touchEventHandler = NULL;
        dObj->context      = (uintptr_t)NULL;				
        dObj->error        = DRV_TOUCH_FT5XX6_ERROR_NONE;
        
        /* Update the client status */
        dObj->clientStatus = DRV_TOUCH_FT5XX6_CLIENT_STATUS_READY;
        return ((DRV_HANDLE)0 );
        
    }

    return (DRV_HANDLE_INVALID);
}

// *****************************************************************************
/* Function:
    DRV_TOUCH_FT5XX6_CLIENT_STATUS DRV_TOUCH_FT5XX6_Close ( void )

  Summary:
    Dynamic implementation of DRV_TOUCH_FT5XX6_Close client interface function.

  Description:
    This is the dynamic implementation of DRV_TOUCH_FT5XX6_Close client interface
    function.

  Remarks:
    See drv_ft5xx6.h for usage information.
*/

DRV_TOUCH_FT5XX6_CLIENT_STATUS DRV_TOUCH_FT5XX6_Close ( void )
{
    DRV_TOUCH_FT5XX6_STATIC_OBJ *dObj = (DRV_TOUCH_FT5XX6_STATIC_OBJ*)NULL;
    
    dObj = (DRV_TOUCH_FT5XX6_STATIC_OBJ*) &gDrvFT5XX6StaticObj;

    /* Remove all buffers that this client owns from the driver queue. This
       function will map to _DRV_TOUCH_FT5XX6_ClientBufferQueueObjectsRemove() if the
       driver was built for buffer queue support. Else this condition always
       maps to true. */
    if(!_DRV_TOUCH_FT5XX6_ClientBufferQueueObjectsRemove( ))
    {
        /* The function could fail if the mutex time out occurred */
        SYS_DEBUG(0, "Could not remove client buffer objects");
        dObj->clientStatus = DRV_TOUCH_FT5XX6_CLIENT_STATUS_ERROR;
        return (DRV_TOUCH_FT5XX6_CLIENT_STATUS_ERROR);
    }

    /* De-allocate the object */
    dObj->clientStatus = DRV_TOUCH_FT5XX6_CLIENT_STATUS_CLOSED;
    dObj->clientInUse = false;

    return (DRV_TOUCH_FT5XX6_CLIENT_STATUS_CLOSED);
}

// *****************************************************************************
/* Function:
    DRV_TOUCH_FT5XX6_ERROR DRV_TOUCH_FT5XX6_ErrorGet( void )

  Summary:
    Dynamic implementation of DRV_TOUCH_FT5XX6_ErrorGet client interface function.

  Description:
    This is the dynamic implementation of DRV_TOUCH_FT5XX6_ErrorGet client interface
    function.

  Remarks:
    See drv_ft5xx6.h for usage information.
*/

DRV_TOUCH_FT5XX6_ERROR DRV_TOUCH_FT5XX6_ErrorGet( void )
{
    DRV_TOUCH_FT5XX6_STATIC_OBJ *dObj = (DRV_TOUCH_FT5XX6_STATIC_OBJ*)NULL;
    DRV_TOUCH_FT5XX6_ERROR error;

    dObj = (DRV_TOUCH_FT5XX6_STATIC_OBJ*) &gDrvFT5XX6StaticObj;
    
    /* Return the error. Clear the error before
       returning. */
    error = dObj->error;
    dObj->error = DRV_TOUCH_FT5XX6_ERROR_NONE;
    return(error);
}

// *****************************************************************************
/* Function:
    void DRV_TOUCH_FT5XX6_Tasks ( SYS_MODULE_OBJ object )

  Summary:
    Dynamic implementation of DRV_TOUCH_FT5XX6_Tasks system interface function.

  Description:
    This is the dynamic implementation of DRV_TOUCH_FT5XX6_Tasks system interface
    function.

  Remarks:
    See drv_ft5xx6.h for usage information.
*/
void DRV_TOUCH_FT5XX6_Tasks( SYS_MODULE_OBJ object )
{
    /* This is the FT5XX6 Driver Write tasks routine.
       In this function, the driver checks if a transmit
       interrupt is active and if there are any buffers in
       queue. If so the buffer is serviced. A buffer that
       is serviced completely is removed from the queue.
     */

    DRV_TOUCH_FT5XX6_STATIC_OBJ * hDriver = &gDrvFT5XX6StaticObj;

    if((!hDriver->inUse) || (hDriver->status != SYS_STATUS_READY))
    {
        /* This instance of the driver is not initialized. Don't
         * do anything */
        return;
    }

    _DRV_TOUCH_FT5XX6_BufferQueueTasks(hDriver);

}

/*********************************************************************
  Function:
    DRV_TOUCH_POSITION_SINGLE DRV_TOUCH_FT5XX6_TouchStatus( )

  Summary:
    Returns the status of the current touch input.

*/
DRV_TOUCH_POSITION_STATUS DRV_TOUCH_FT5XX6_TouchStatus( const SYS_MODULE_INDEX index )
{
    DRV_TOUCH_FT5XX6_STATIC_OBJ *dObj = (DRV_TOUCH_FT5XX6_STATIC_OBJ*) &gDrvFT5XX6StaticObj;
    return (dObj->touchStatus);
}


/*********************************************************************
  Function:
    void DRV_TOUCH_FT5XX6_TouchDataRead( )

  Summary:
    Notify the driver that the current touch data has been read

*/
void DRV_TOUCH_FT5XX6_TouchDataRead( const SYS_MODULE_INDEX index )
{
    DRV_TOUCH_FT5XX6_STATIC_OBJ *dObj = (DRV_TOUCH_FT5XX6_STATIC_OBJ*) &gDrvFT5XX6StaticObj;
    dObj->touchStatus = DRV_TOUCH_POSITION_NONE;
}


/*********************************************************************
  Function:
    short DRV_TOUCH_FT5XX6_TouchGetX( uint8 touchNumber )

  Summary:
    Returns the x coordinate of touch input.

  Description:
    It returns the x coordinate in form of number of pixes for a touch input
  denoted by touchNumber.

  Parameters
   touchNumber - index to the touch input.

  Returns
    It returns the x coordinate of the touch input in terms of number of pixels.

*/

short DRV_TOUCH_FT5XX6_TouchGetX( uint8_t touchNumber )
{
    DRV_TOUCH_FT5XX6_STATIC_OBJ *dObj = (DRV_TOUCH_FT5XX6_STATIC_OBJ*) &gDrvFT5XX6StaticObj;
    return dObj->mostRecentTouchX;
}

/*********************************************************************
  Function:
    short DRV_TOUCH_FT5XX6_TouchGetY( uint8 touchNumber )

  Summary:
    Returns the y coordinate of touch input.

  Description:
    It returns the y coordinate in form of number of pixes for a touch input
  denoted by touchNumber.

  Parameters
   touchNumber - index to the touch input.

  Returns
    It returns the y coordinate of the touch input in terms of number of pixels.

*/

short DRV_TOUCH_FT5XX6_TouchGetY( uint8_t touchNumber )
{
    DRV_TOUCH_FT5XX6_STATIC_OBJ *dObj = (DRV_TOUCH_FT5XX6_STATIC_OBJ*) &gDrvFT5XX6StaticObj;
    return dObj->mostRecentTouchY;
}

// *****************************************************************************
// *****************************************************************************
// Section: File scope functions
// *****************************************************************************
// *****************************************************************************

bool _DRV_TOUCH_FT5XX6_ClientBufferQueueObjectsRemove( void )
{
    DRV_TOUCH_FT5XX6_STATIC_OBJ * dObj = (DRV_TOUCH_FT5XX6_STATIC_OBJ *) NULL;
    bool interruptWasEnabled = false;
    DRV_TOUCH_FT5XX6_BUFFER_OBJ * iterator = NULL;

    dObj = &gDrvFT5XX6StaticObj;

    /* Disable the transmit interrupt */
    interruptWasEnabled = SYS_INT_SourceDisable(INT_SOURCE_EXTERNAL_2);
    
    iterator = dObj->queue;
    while(iterator != NULL)
    {
        iterator->inUse = false;
        if(iterator->previous != NULL)
        {
            iterator->previous->next = iterator->next;
        }
        if(iterator->next != NULL)
        {
            iterator->next->previous = iterator->previous;
        }
        /* Decrementing Current queue size */
        dObj->queueSizeCurrent --;

        iterator = iterator->next;
    }

    /* If there are no buffers in the write queue.
     * Make the head pointer point to NULL */
    if(dObj->queueSizeCurrent == 0)
    {
        dObj->queue = NULL;
    }
    else
    {
        /* Iterate to update the head pointer to point
         * the first valid buffer object in the queue */
        iterator = dObj->queue;
        while(iterator != NULL)
        {
            if(iterator->inUse == true)
            {
                dObj->queue = iterator;
                break;
            }
            iterator = iterator->next;
        }
    }

    /* Re-enable the interrupt if it was enabled */
    if(interruptWasEnabled)
    {
        SYS_INT_SourceEnable(INT_SOURCE_EXTERNAL_2);
    }

    return true;

}

void _DRV_TOUCH_FT5XX6_BufferQueueTasks(DRV_TOUCH_FT5XX6_STATIC_OBJ * hDriver)
{
    DRV_TOUCH_FT5XX6_BUFFER_OBJ * bufferObj = ( DRV_TOUCH_FT5XX6_BUFFER_OBJ * ) NULL;

    bufferObj = hDriver->queue;

    if( DRV_HANDLE_INVALID != hDriver->drvBusHandle &&
        NULL               != bufferObj )
    {
        if( 0 == bufferObj->size )
        {
            hDriver->queue   = bufferObj->next;
            bufferObj->inUse = false;
            hDriver->queueSizeCurrent--;

            touch_x=(unsigned int)(hDriver->touchData.touch[0].X_Msb)<<8;
            touch_y=(unsigned int)(hDriver->touchData.touch[0].Y_Msb)<<8;

            touch_x=touch_x+(unsigned int)(hDriver->touchData.touch[0].X_Lsb);
            touch_y=touch_y+(unsigned int)(hDriver->touchData.touch[0].Y_Lsb);

            if( hDriver->touchData.status.nTouch  != 0 &&
                /*hDriver->touchData.touch[0].nibble_0.inRange != 0 &&*/
                /*hDriver->touchData.touch[0].x*/ touch_x != 0 &&
                /*hDriver->touchData.touch[0].y*/ touch_y != 0 )
            {
//                //Translate the touch data to X/Y coordinates
                hDriver->mostRecentTouchX =touch_x;// (int16_t)DRV_TOUCH_FT5XX6_TouchInputMap(/*hDriver->touchData.touch[0].x*/ touch_x<<4 , 240);
                hDriver->mostRecentTouchY =touch_y;// (int16_t)DRV_TOUCH_FT5XX6_TouchInputMap(/*hDriver->touchData.touch[0].y*/touch_y<<4, 320);
                //Translate the touch data to X/Y coordinates
//                hDriver->mostRecentTouchX =touch_y;// (int16_t)DRV_TOUCH_FT5XX6_TouchInputMap(/*hDriver->touchData.touch[0].x*/ touch_x<<4 , 240);
//                hDriver->mostRecentTouchY =240-touch_x;// (int16_t)DRV_TOUCH_FT5XX6_TouchInputMap(/*hDriver->touchData.touch[0].y*/touch_y<<4, 320);

            }
            else
            {
                hDriver->mostRecentTouchX = -1;
                hDriver->mostRecentTouchY = -1;
            }

            hDriver->touchStatus = DRV_TOUCH_POSITION_SINGLE;

            if( NULL != hDriver->queue )
            {
                bufferObj = hDriver->queue;

                switch( bufferObj->flags )
                {
                    case DRV_TOUCH_FT5XX6_BUFFER_OBJ_FLAG_REG_READ:
                    {
                        bufferObj->hBusBuffer =
                                DRV_I2C_BufferAddWriteRead( hDriver->drvBusHandle,
                                                            (uint8_t *)&hDriver->deviceAddress,
                                                            ( void *)&bufferObj->regAddress,
                                                            1,
                                                            ( void * )bufferObj->readBuffer,
                                                            bufferObj->size,
                                                            (void *)NULL );

                        break;
                    }

                    case DRV_TOUCH_FT5XX6_BUFFER_OBJ_FLAG_REG_WRITE:
                    {
                        bufferObj->hBusBuffer =
                                    DRV_I2C_BufferAddWrite( hDriver->drvBusHandle,
                                                            (uint8_t *)&hDriver->deviceAddress,
                                                            (uint8_t *)bufferObj->writeBuffer,
                                                            bufferObj->size,
                                                            (void *) NULL);
                        break;
                    }

                }
            }
        }
    }

    return;
}

/*******************************************************************************
 End of File
*/
