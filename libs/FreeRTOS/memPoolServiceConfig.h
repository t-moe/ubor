#ifndef MEMPOOLSERVICECONFIG_H_
#define MEMPOOLSERVICECONFIG_H_
/******************************************************************************/
/** \file       memPoolServiceConfig.h
 *******************************************************************************
 *
 *  \brief      This module allows you to obtain/return fixed-size memory
 *              blocks from  a previously created pool of a contiguous memory
 *              area. All memory blocks have the same size and a pool contains
 *              a defined number of memory blocks. More than one memory pool
 *              can exist, so your application can obtain memory blocks of
 *              different size from different pools. However, a specific
 *              memory block must be returned to the memory pool from which
 *              it came from.
 *              This module contains three files:
 *              memPoolService.c --> Implementation file
 *              memPoolService.h --> Declaration file
 *              memPoolServiceConfig.h --> Module configuration file
 *
 *  \author     wht4
 *
 ******************************************************************************/
/*
 *  function    .
 *
 ******************************************************************************/

//----- Header-Files -----------------------------------------------------------

//----- Macros -----------------------------------------------------------------
/* Set to '1' and function input arguments will be checked. If set to '0' all */
/* function arguments will be trusted                                         */
#define MEM_ARGUMENT_CHECK            ( 1 )

/* Set to '1' and each pool can be created with a descriptive name            */
#define MEM_POOL_NAME                 ( 1 )
/* The maximum permissible length of the descriptive name given to a memory   */
/* pool when the memory pool is created. The length is specified in the number*/
/* of characters including the NULL termination byte.                         */
#define MEM_POOL_NAME_LENGTH          ( 16 )

/* If a RTOS is underlying your application, then this can be beneficial for  */
/* this module. With a counting semaphore, we will implement the              */
/* eMemTakeBlockWithTimeout function. This function will wait for a specified */
/* time that a memory block becomes available.                                */
#define MEM_USE_COUNTING_SEMAPHORE    ( 1 )

//----- Data types -------------------------------------------------------------

//----- Function prototypes ----------------------------------------------------

//----- Data -------------------------------------------------------------------

#endif /* MEMPOOLSERVICECONFIG_H_ */
