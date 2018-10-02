

// **************************************************************************
// the includes

#include "Const.h"
#include <stdio.h>
#include <uartRs.h>
#include "main.h"

// **************************************************************************
// the defines


// **************************************************************************
// the globals


// **************************************************************************
// the functions

SERIAL_Handle SERIAL_init(void *pMemory,const size_t numBytes)
{
  SERIAL_Handle handle;

  if(numBytes < sizeof(SERIAL_Obj))
    return((SERIAL_Handle)NULL);

  // assign the handle
  handle = (SERIAL_Handle)pMemory;

  return(handle);
} // end of SERIAL_init() function

void SERIAL_UartOnMcbsp(SERIAL_Handle handle, char *str, char *mcbsp)
{
    SERIAL_Obj *serialRx = (SERIAL_Obj *)handle;

    // 5 bytes on transmission line are needed to transmit 4 characters
    // 8 bits of data -> 10 transmitted bits (1 start bit + 8 data bits + 1 stop bit)
    // 10/8 = 5/4


    uint16_t uartLength = (serialRx->lengthOfBuffer/5) * 4;

    char uartArray[64];
    char *uart =  &uartArray[0];

    strncpy(uart,str,uartLength);

    volatile uint16_t n;
    uint16_t u, m;
    for(n=0; n<(serialRx->lengthOfBuffer/5); n++)
    {
        u = n*4;
        m = n*5;

        mcbsp[0+m] = (uart[0+u] << 1) & 0xFE;
        mcbsp[1+m] = ((uart[1+u] << 3) & 0xF8) + 0x02 + ((uart[0+u] >> 7) & 0x01);
        mcbsp[2+m] = ((uart[2+u] << 5) & 0xE0) + 0x08 + ((uart[1+u] >> 5) & 0x07);
        mcbsp[3+m] = ((uart[3+u] << 7) & 0x80) + 0x20 + ((uart[2+u] >> 3) & 0x1F);
        mcbsp[4+m] = 0x80 + ((uart[3+u] >> 1) & 0x7F);
    }
}


SERIAL_ErrorCode_e SERIAL_fifoWrite(SERIAL_Handle handle,char *str)
{
    SERIAL_Obj *serialRx = (SERIAL_Obj *)handle;
    SERIAL_ErrorCode_e error;
    volatile uint_least8_t tempHead;

    // Copy string to circular buffer
    memcpy((void*)&(serialRx->buffer[serialRx->head][0]),(void*)str,serialRx->lengthOfBuffer);

    error = SERIAL_ErrorCode_Error1;

    tempHead = serialRx->head;
    tempHead++;
    if(tempHead >= serialRx->noOfBuffers)
    {
        tempHead = 0;
    }
    if(tempHead != serialRx->tail)
    {
        serialRx->head = tempHead;
        error = SERIAL_ErrorCode_NoError;
    }

    return error;
}


SERIAL_ErrorCode_e SERIAL_fifoWriteMcbsp(SERIAL_Handle handle,char *str)
{
    SERIAL_Obj *serialRx = (SERIAL_Obj *)handle;
    SERIAL_ErrorCode_e error;

    volatile uint_least8_t tempHead;
    char uartArray[64];
    char *uart =  &uartArray[0];

    // 5 bytes on transmission line are needed to transmit 4 characters
    // 8 bits of data -> 10 transmitted bits (1 start bit + 8 data bits + 1 stop bit)
    // 10/8 = 5/4
    uint16_t uartLength = (serialRx->lengthOfBuffer/5) * 4;

    strncpy(uart,str,uartLength);

    volatile uint16_t n;
    uint16_t u, m;
    for(n=0; n<(serialRx->lengthOfBuffer/5); n++)
    {
        u = n*4;
        m = n*5;

        serialRx->buffer[serialRx->head][0+m] = (uart[0+u] << 1) & 0xFE;
        serialRx->buffer[serialRx->head][1+m] = ((uart[1+u] << 3) & 0xF8) + 0x02 + ((uart[0+u] >> 7) & 0x01);
        serialRx->buffer[serialRx->head][2+m] = ((uart[2+u] << 5) & 0xE0) + 0x08 + ((uart[1+u] >> 5) & 0x07);
        serialRx->buffer[serialRx->head][3+m] = ((uart[3+u] << 7) & 0x80) + 0x20 + ((uart[2+u] >> 3) & 0x1F);
        serialRx->buffer[serialRx->head][4+m] = 0x80 + ((uart[3+u] >> 1) & 0x7F);
    }

    error = SERIAL_ErrorCode_Error1;

    tempHead = serialRx->head;
    tempHead++;
    if(tempHead >= serialRx->noOfBuffers)
    {
        tempHead = 0;
    }
    if(tempHead != serialRx->tail)
    {
        serialRx->head = tempHead;
        error = SERIAL_ErrorCode_NoError;
    }

    return error;
}



SERIAL_ErrorCode_e SERIAL_parseCommand(SERIAL_Handle handle)
{
    SERIAL_Obj *serialRx = (SERIAL_Obj *)handle;
    if (strncmp(&serialRx->buffer[serialRx->tail][0], serialId, 1) == 0)

    {
        f2[0] = (&serialRx->buffer[serialRx->tail][0]+1);
        f2[1] = (&serialRx->buffer[serialRx->tail][0]+2);
        serialData[0] = *f2[0];
        serialData[1] = *f2[1];

    switch(serialData[0])
    {
        case STOP_DRIVE:
            stopMotor1();
            sendToSlave(STOP_DRIVE);
            handle->positionIn = 0;
            break;
        case 'L':
            switchLed1(STATE_TOGGLE);
            handle->positionIn = 0;
            break;
        case SOFT_STOP:
            initSoftStop();
            sendToSlave(SOFT_STOP);
            handle->positionIn = 0;
            break;
        case 'N':
            driveMotor1(100);
            driveMotor2(100);
            handle->positionIn = 0;
            break;
        case DRIVE_FORWARD:
            driveForward();
            sendToSlave(DRIVE_FORWARD);
            handle->positionIn = 0;
            break;
        case DRIVE_BACKWARD:
            driveBackward();
            sendToSlave(DRIVE_BACKWARD);
            handle->positionIn = 0;
            break;
        case 'Z':
            sendStart();
            handle->positionIn = 0;
            break;
        case DRIVE_FORWARD_DISTANCE:
            driveForward();
            distanceMode = STATE_ON;
            sendToSlave(DRIVE_FORWARD_DISTANCE);
            handle->positionIn = 0;
            break;
        case DRIVE_BACKWARD_DISTANCE:
            driveBackward();
            distanceMode = STATE_ON;
            sendToSlave(DRIVE_BACKWARD_DISTANCE);
            handle->positionIn = 0;
            break;
        case 'Q':
            driveMotor3(500);
            handle->positionIn = 0;
            break;
        case 'W':
            driveMotor3Up();
            handle->positionIn = 0;
            break;
        case INCREASE_SPEED:
            increaseSpeed1(50);
            sendToSlave(INCREASE_SPEED);
            handle->positionIn = 0;
            break;
        case DECREASE_SPEED:
            decreaseSpeed1(50);
            sendToSlave(DECREASE_SPEED);
            handle->positionIn = 0;
            break;
        case 'T':
            ScannerTrig();
            handle->positionIn = 0;
            break;
        case '9':
            SetProximityIntClear();
            handle->positionIn = 0;
            break;
        case 'P':
            setProximity();
            handle->positionIn = 0;
            break;
        default:
            handle->positionIn = 0;
            break;
    }

    // analyze received string...

    // when received string is analyzed move to the next string in cyclic buffer
    serialRx->tail++;

    // if head gets out of range, return to the beginning of cyclic buffer
    if(serialRx->tail >= serialRx->noOfBuffers)
    {
        serialRx->tail = 0;
    }
 //return error;
}
    else
    {
        // when received string is analyzed move to the next string in cyclic buffer
           serialRx->tail++;

           // if head gets out of range, return to the beginning of cyclic buffer
           if(serialRx->tail >= serialRx->noOfBuffers)
           {
               serialRx->tail = 0;
           }
    }

}

void SERIAL_setDefaults(SERIAL_Handle handle)
{
    handle->positionIn = 0;
    handle->positionOut = 0;
    handle->head = 0;
    handle->tail = 0;
    handle->status = 0;
    handle->noOfBuffers = 4;
    handle->lengthOfBuffer = sizeof(handle->buffer) / handle->noOfBuffers;

}



// end of file
