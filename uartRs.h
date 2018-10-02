#ifndef _FIBER_H_
#define _FIBER_H_

//! \file   modules/fiber/src/32b/fiber.h
//! \brief  Contains the public interface to the 
//!         Proportional-Integral-Derivative (PID) controller module routines
//!
//! (C) Copyright 2011, Texas Instruments, Inc.


// **************************************************************************
// the includes

//modules
//#include "sw/modules/types/src/types.h"

//#include "sw/modules/iqmath/src/32b/IQmathLib.h"
//#include <include/IQmathLib.h>
#include <string.h>
//#include <typedefs.h>
#include "types.h"
//#include "main.h"

//!
//!
//! \defgroup FIBER FIBER
//!
//@{

// Include the algorithm overview defined in modules/<module>/docs/doxygen/doxygen.h
//! \defgroup FIBER_OVERVIEW 

#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines



// **************************************************************************
// the typedefs

//! \brief Enumeration to define the communication error code
//!
typedef enum
{
  SERIAL_ErrorCode_NoError = 0,    //!< Denotes Error code "NoError"
  SERIAL_ErrorCode_Error1 = 1      //!< Denotes Error code "NoError"
} SERIAL_ErrorCode_e;



//!
typedef struct _SERIAL_Obj_
{
  volatile uint16_t buffer[4][80];  //!< the buffer of buffers
  volatile uint16_t head;           //!< defines which buffer is currently written
  volatile uint16_t tail;           //!< defines the first (oldest) unprocessed buffer
  volatile uint16_t positionIn;     //!< defines a write position in buffer
  volatile uint16_t positionOut;    //!< defines a read position in buffer
  volatile uint16_t status;         //!< defines the status of buffer
  volatile uint16_t noOfBuffers;    //!< defines the number of buffers in FIFO bufer
  volatile uint16_t lengthOfBuffer; //!< defines length of buffer
} SERIAL_Obj;


//! \brief Defines the SERIAL handle
//!
typedef struct _SERIAL_Obj_ *SERIAL_Handle;


// **************************************************************************
// the function prototypes

//! \brief      Writes string to FIFO buffer
//! \param[in]  pMemory   A pointer to the memory for the fiber optic module
//! \param[in]  numBytes  The number of bytes allocated for the fiber optic module
//! \return     The FIBER object handle
extern SERIAL_Handle SERIAL_init(void *pMemory,const size_t numBytes);

//! \brief      Formats string for transmission via McBSP
//! \param[in]  handle  The SERIAL object handle
//! \param[in]  str     A pointer to the string to be written
//! \param[in]  mcbsp   A pointer to the output string
extern void SERIAL_UartOnMcbsp(SERIAL_Handle handle, char *str, char *mcbsp);

//! \brief      Writes string to FIFO buffer
//! \param[in]  handle  The SERIAL object handle
//! \param[in]  str     A pointer to the string to be written
//! \return     SERIAL Error code
extern SERIAL_ErrorCode_e SERIAL_fifoWrite(SERIAL_Handle handle,char *str);

//! \brief      Formats string for transmission via McBSP and writes to FIFo buffer
//! \param[in]  handle  The SERIAL object handle
//! \param[in]  str     A pointer to the string to be written
//! \return     SERIAL Error code
extern SERIAL_ErrorCode_e SERIAL_fifoWriteMcbsp(SERIAL_Handle handle,char *str);


//! \brief      Parses received command
//! \param[in]  handle  The SERIAL object handle
//! \return     SERIAL Error code
extern SERIAL_ErrorCode_e SERIAL_parseCommand(SERIAL_Handle handle);

extern SERIAL_ErrorCode_e SERIAL_parseCommand2(SERIAL_Handle handle);

void SERIAL_setDefaults(SERIAL_Handle handle);


#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif

