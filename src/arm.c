/******************************************************************************/
/** \file       arm.c
 *******************************************************************************
 *
 *  \brief      Functions to move roboter
 *
 *  \author     ingmacmech
 *
 *  \date       03.04.2017
 *
 *  \remark     Last Modification
 *               \li ingmacmech, 03.04.2017, Created
 *
 *
 ******************************************************************************/
/*
 *  functions  global:
 *
 *  functions  local:
 *              .
 *
 ******************************************************************************/

//----- Header-Files -----------------------------------------------------------
#include <stdio.h>                      /* Standard Input/Output              */

#include <FreeRTOS.h>                   /* All freeRTOS headers               */
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include <timers.h>
#include <memPoolService.h>

#include "arm.h"
#include "ucan.h"

//----- Macros -----------------------------------------------------------------


//----- Data types -------------------------------------------------------------

//----- Function prototypes ----------------------------------------------------

//----- Data -------------------------------------------------------------------
static QueueHandle_t queueRobot;
static SemaphoreHandle_t mutexMiddlePosition;

//----- Implementation ---------------------------------------------------------


/*******************************************************************************
 *  function :    vMoveRoboter
 ******************************************************************************/
/** \brief
 *
 *  \type         global
 *
 *  \return       void
 *
 ******************************************************************************/
void vMoveRoboter(void *pvData) {

	int leftRightSel = (int)pvData;
	uint8_t posArm[10][8];


	/* Init */
	if(leftRightSel == 0)
	{
		mutexMiddlePosition = xSemaphoreCreateMutex();
		queueRobot = xQueueCreate(MSG_QUEUE_SIZE, 8);

		 uint8_t posArm[10][8] = {
			/*                      Arm	   B     S     E     H     G     x     x */
			/*0 Nullposition     */ {0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
			/*1 StartPosition    */ {0x02, 0x00, 0xF0, 0x3A, 0xFD, 0x00, 0x00, 0x00},
			/*2 Zp1 FB Links     */ {0x02, 0x00, 0xF0, 0x35, 0xFD, 0x00, 0x00, 0x00},
			/*3 Zp2 FB Links     */ {0x02, 0x00, 0x00, 0x2E, 0xE8, 0x00, 0x00, 0x00},
			/*4 G offen FB Links */ {0x02, 0x00, 0x19, 0x37, 0xE8, 0x01, 0x00, 0x00},
			/*5 G zu FB Links    */ {0x02, 0x00, 0x19, 0x37, 0xE8, 0x00, 0x00, 0x00},
			/*6 Zp1 FB Mitte     */ {0x02, 0xD3, 0xF0, 0x35, 0xFD, 0x00, 0x00, 0x00},
			/*7 Zp2 FB Mitte     */ {0x02, 0xD3, 0x00, 0x2E, 0xE8, 0x00, 0x00, 0x00},
			/*8 G offen FB Mitte */ {0x02, 0xD3, 0x19, 0x37, 0xE8, 0x01, 0x00, 0x00},
			/*9 G zu FB Mitte    */ {0x02, 0xD3, 0x19, 0x37, 0xE8, 0x00, 0x00, 0x00},

			};
	}

	if(leftRightSel == 1)
	{
	 	 uint8_t posArm[10][8] = {
				/*                      Arm	   B     S     E     H     G     x     x */
				/*0 Nullposition     */ {0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
				/*1 StartPosition    */ {0x02, 0x00, 0xF0, 0x3A, 0xFD, 0x00, 0x00, 0x00},
				/*2 Zp1 FB Rechts    */ {0x02, 0x00, 0xF0, 0x35, 0xFD, 0x00, 0x00, 0x00},
				/*3 Zp2 FB Rechts    */ {0x02, 0x00, 0x00, 0x2E, 0xE8, 0x00, 0x00, 0x00},
				/*4 G offen FB Rechts*/ {0x02, 0x00, 0x19, 0x37, 0xE8, 0x01, 0x00, 0x00},
				/*5 G zu FB Rechts   */ {0x02, 0x00, 0x19, 0x37, 0xE8, 0x00, 0x00, 0x00},
				/*6 Zp1 FB Mitte     */ {0x02, 0x2D, 0xF0, 0x35, 0xFD, 0x00, 0x00, 0x00},
				/*7 Zp2 FB Mitte     */ {0x02, 0x2D, 0x00, 0x2E, 0xE8, 0x00, 0x00, 0x00},
				/*8 G offen FB Mitte */ {0x02, 0x2D, 0x19, 0x37, 0xE8, 0x01, 0x00, 0x00},
				/*9 G zu FB Mitte    */ {0x02, 0x2D, 0x19, 0x37, 0xE8, 0x00, 0x00, 0x00},
				};
	}



	while(1)
	{
		for(int n = 0; n < 10; n++)
		{

			if(n == 1)
			{
				xSemaphoreGive(mutexMiddlePosition);
			}

			if (n == 6)
			{
				if(xSemaphoreTake(mutexMiddlePosition, BLOCK_TIME_MIDDLE_POS) == pdTRUE);
			}
			ucan_send_data(COMAND_DLC, ROBOT_L_COMAND_REQUEST_ID, posArm[n] );
			//waitUntilPos(posArm[n]);
			vTaskDelay(TASK_DELAY);
		}


	}

}
/*
void waitUntilPos(uint8_t *pos)
{

	while(pcMsgBuffer != pos)
	{
		ucan_send_data(ROBOT_L_STATUS_REQUEST_ID, STATUS_REQEST_DLC, STAUS_ARM );

		if (xQueueReceive(queueRobot, (void *)&pcMsgBuffer, portMAX_DELAY) == pdTRUE);

	}

}*/

void initArm()
{
	xTaskCreate(vMoveRoboter,
	                "Arm Left",
	                ARM_TASK_STACKSIZE,
	                (void*)0,
	                ARM_TASK_PRIORITY,
	                NULL);
/*
	xTaskCreate(vMoveRoboter,
	                "Arm Right",
	                ARM_TASK_STACKSIZE,
	                (void*)1,
	                ARM_TASK_PRIORITY,
	                NULL);*/
}

