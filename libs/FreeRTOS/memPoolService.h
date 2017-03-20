#ifndef MEMPOOLSERVICE_H_
#define MEMPOOLSERVICE_H_
/******************************************************************************/
/** \file       memPoolService.h
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
 *  function    eMemCreateMemoryPool
 *              eMemTakeBlock
 *              eMemTakeBlockWithTimeout
 *              eMemTakeBlockFromISR
 *              eMemGiveBlock
 *              eMemGiveBlockFromISR
 *
 ******************************************************************************/

//----- Header-Files -----------------------------------------------------------
#include "FreeRTOS.h"                            /* RTOS include files        */
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "portmacro.h"

#include "memPoolServiceConfig.h"

//----- Macros -----------------------------------------------------------------
/* With a counting semaphore, we will implement the eMemTakeBlockWithTimeout  */
/* function. This function will wait for a specified time that a memory block */
/* becomes available. Just wrap the required RTOS functions. At the moment we */
/* will use for this purpose the freeRTOS functions but with this wrappers    */
/* the underlying RTOS can easily be replaced by an other one.                */
#if(MEM_USE_COUNTING_SEMAPHORE == 1)
/* Counting semaphore data type */
#define MEM_COUNTING_SEMAPHORE      xSemaphoreHandle

/* Wrapper for creating a counting semaphore */
#define MEM_SEMAPHORE_CREATE(uxMaxCount, uxInitialCount)                       \
    xSemaphoreCreateCounting((unsigned portBASE_TYPE) uxMaxCount,              \
                             (unsigned portBASE_TYPE) uxInitialCount)

/* Wrapper for taking a counting semaphore */
#define MEM_SEMAPHORE_TAKE(xSemaphore, xBlockTime)                             \
	xSemaphoreTake(xSemaphore, xBlockTime)

/* Wrapper for giving a counting semaphore */
#define MEM_SEMAPHORE_GIVE(xSemaphore)                                         \
	xSemaphoreGive(xSemaphore)

/* Wrapper for taking a counting semaphore out of an ISR */
#define MEM_SEMAPHORE_TAKE_ISR(xSemaphore, pxTaskWoken)                        \
    xQueueReceiveFromISR((xQueueHandle) (xSemaphore), NULL, pxTaskWoken)

/* Wrapper for giving a counting semaphore out of an ISR */
#define MEM_SEMAPHORE_GIVE_ISR(xSemaphore, pxTaskWoken)                        \
    xSemaphoreGiveFromISR(xSemaphore, pxTaskWoken)

#endif // (MEM_USE_COUNTING_SEMAPHORE == 1)

//----- Data types -------------------------------------------------------------
/* Return values. This enum includes all error options. */
typedef enum     {
    MEM_NO_ERROR                 = 0,  /* No error detected                   */

    MEM_INVALID_ADDRESS          = 1,  /* Input argument points to (void *) 0 */
    MEM_INVALID_ALIGNMENT        = 2,  /* Blocks must be pointer size aligned */
    MEM_INVALID_NUMBER_OF_BLOCKS = 3,  /* Number of blocks must be > 2        */
    MEM_INVALID_BLOCK_SIZE       = 4,  /* Invalid size of memory block        */
    MEM_COULDNT_CREATE_SEMAPHORE = 5,  /* Couldn't create counting semaphore  */
    MEM_NO_FREE_BLOCKS           = 6,  /* All blocks are occupied             */
    MEM_SEM_UNKNOWN_ERROR        = 7,  /* Unknown semaphore error             */
    MEM_TIMEOUT_ELAPSED          = 8,  /* Timeout of semaphore elapsed        */
    MEM_POOL_FULL                = 9   /* Memory pool is already full         */
} enumMemError;

/* Every memory poll needs a MemPoolManager structure. This structure holds   */
/* all important informations about the memory pool                           */
typedef struct   {
    void              *pvMemAddress;             /* Pointer to the start of the 
                                                    pool                      */
    void              *pvMemFreeList;            /* List to the free memory
                                                    blocks                    */
    unsigned portLONG  u32MemBlockSize;          /* Size of one memory block
                                                    [bytes]                   */
    unsigned portLONG  u32MemNumberOfBlocks;     /* Number of memory blocks   */
    unsigned portLONG  u32MemNumberOfFreeBlocks; /* Number of free memory
                                                   blocks                     */

#if(MEM_USE_COUNTING_SEMAPHORE == 1)
    MEM_COUNTING_SEMAPHORE semaphoreMemoryPool; /* Counting semaphore handle  */
#endif // (MEM_USE_COUNTING_SEMAPHORE == 1)

#if(MEM_POOL_NAME == 1)
    portCHAR  *pcMemName[MEM_POOL_NAME_LENGTH]; /* Name of the memory pool    */
#endif //(MEM_POOL_NAME == 1)
} MemPoolManager;

//----- Function prototypes ----------------------------------------------------
extern enumMemError  eMemCreateMemoryPool(MemPoolManager    *psMemPoolManager,
                                          void              *pvMemAddress,
                                          unsigned portLONG  u32MemBlockSize,
                                          unsigned portLONG  u32MemNumberOfBlocks,
                                          const portCHAR    *pcMemName);

extern enumMemError  eMemTakeBlock(MemPoolManager  *psMemPoolManager,
                                   void           **ppvMemBlock);

#if (MEM_USE_COUNTING_SEMAPHORE == 1)
extern enumMemError  eMemTakeBlockWithTimeout(MemPoolManager     *psMemPoolManager,
                                              void              **ppvMemBlock,
                                              unsigned portLONG   u32Timeout);
#endif /* (MEM_USE_COUNTING_SEMAPHORE == 1) */


extern enumMemError  eMemTakeBlockFromISR(MemPoolManager  *psMemPoolManager,
                                          void           **ppvMemBlock,
                                          portBASE_TYPE   *ps32TaskWoken);

extern enumMemError  eMemGiveBlock(MemPoolManager *psMemPoolManager,
                                   void           *pvMemBlock);

extern enumMemError  eMemGiveBlockFromISR(MemPoolManager *psMemPoolManager,
                                          void           *pvMemBlock,
                                          portBASE_TYPE  *ps32TaskWoken);

//----- Data -------------------------------------------------------------------

#endif /* MEMPOOLSERVICE_H_ */
