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
#include <stdbool.h>

#include "arm.h"
#include "ucan.h"

//----- Macros -----------------------------------------------------------------


//----- Data types -------------------------------------------------------------

//----- Function prototypes ----------------------------------------------------

//----- Data -------------------------------------------------------------------
static QueueHandle_t queueRobotLeft;
static QueueHandle_t queueRobotRight;
static SemaphoreHandle_t mutexMiddlePosition;
CARME_CAN_MESSAGE pcMsgBufferRight;
CARME_CAN_MESSAGE pcMsgBufferLeft;

uint8_t status_request[2] = {0x02,0x00};
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
	uint8_t *posArm;
	int id_arm_comand_request;

	/* Init */
	if(leftRightSel == 0)
	{
		static uint8_t posArmRight[10][8] = {
			/*                      Arm	   B     S     E     H     G     x     x */
			/*0 Nullposition     */ {0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
			/*1 StartPosition    */ {0x02, 0x00, 0xF0, 0x3A, 0xFD, 0x00, 0x00, 0x00},
			/*2 Zp1 FB Links     */ {0x02, 0x00, 0xF0, 0x35, 0xFD, 0x00, 0x00, 0x00},
			/*3 Zp2 FB Links     */ {0x02, 0x00, 0x00, 0x2E, 0xE8, 0x00, 0x00, 0x00},
			/*4 G offen FB Links */ {0x02, 0x00, 0x19, 0x37, 0xE8, 0x01, 0x00, 0x00},
			/*5 G zu FB Links    */ {0x02, 0x00, 0x19, 0x37, 0xE8, 0x00, 0x00, 0x00},
			/*6 Zp1 FB Mitte     */ {0x02, 0xD3, 0xF0, 0x35, 0xFD, 0x00, 0x00, 0x00},
			/*7 Zp2 FB Mitte     */ {0x02, 0xD3, 0x00, 0x2E, 0xE8, 0x00, 0x00, 0x00},
			/*8 G zu FB Mitte    */ {0x02, 0xD3, 0x19, 0x37, 0xE8, 0x00, 0x00, 0x00},
			/*9 G offen FB Mitte */ {0x02, 0xD3, 0x19, 0x37, 0xE8, 0x01, 0x00, 0x00},
			};


		mutexMiddlePosition = xSemaphoreCreateMutex();
		queueRobotRight = xQueueCreate(MSG_QUEUE_SIZE, sizeof(CARME_CAN_MESSAGE));
		ucan_link_message_to_queue(ROBOT_R_STATUS_RETURN_ID, queueRobotRight);
		posArm = (uint8_t *)posArmRight;
		id_arm_comand_request = ROBOT_R_COMAND_REQUEST_ID;
	}

	if(leftRightSel == 1)
	{
	 	static uint8_t posArmLeft[10][8] = {
				/*                      Arm	   B     S     E     H     G     x     x */
				/*0 Nullposition     */ {0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
				/*1 StartPosition    */ {0x02, 0x00, 0xF0, 0x3A, 0xFD, 0x00, 0x00, 0x00},
				/*2 Zp1 FB Rechts    */ {0x02, 0x00, 0xF0, 0x35, 0xFD, 0x00, 0x00, 0x00},
				/*3 Zp2 FB Rechts    */ {0x02, 0x00, 0x00, 0x2E, 0xE8, 0x00, 0x00, 0x00},
				/*4 G offen FB Rechts*/ {0x02, 0x00, 0x19, 0x37, 0xE8, 0x01, 0x00, 0x00},
				/*5 G zu FB Rechts   */ {0x02, 0x00, 0x19, 0x37, 0xE8, 0x00, 0x00, 0x00},
				/*6 Zp1 FB Mitte     */ {0x02, 0x2D, 0xF0, 0x35, 0xFD, 0x00, 0x00, 0x00},
				/*7 Zp2 FB Mitte     */ {0x02, 0x2D, 0x00, 0x2E, 0xE8, 0x00, 0x00, 0x00},
				/*8 G zu FB Mitte    */ {0x02, 0x2D, 0x19, 0x37, 0xE8, 0x00, 0x00, 0x00},
				/*9 G offen FB Mitte */ {0x02, 0x2D, 0x19, 0x37, 0xE8, 0x01, 0x00, 0x00},
				};
	 	posArm = (uint8_t *)posArmLeft;
	 	id_arm_comand_request = ROBOT_L_COMAND_REQUEST_ID;
	 	queueRobotLeft = xQueueCreate(MSG_QUEUE_SIZE, sizeof(CARME_CAN_MESSAGE));
	 	ucan_link_message_to_queue(ROBOT_L_STATUS_RETURN_ID, queueRobotLeft);
	}

	while(1)
	{

		for(int n = 0; n < 10; n++)
		{
			if(leftRightSel == 0)
			{
				//display_log(DISPLAY_NEWLINE, "Right arm position %i", n);
			}
			else
			{
				//display_log(DISPLAY_NEWLINE, "Left arm position %i", n);
			}


			if(n == 1)
			{
				xSemaphoreGive(mutexMiddlePosition);
				if(leftRightSel == 0)
				{
					//display_log(DISPLAY_NEWLINE, "Right arm give semaphore");
				}
				else
				{
					//display_log(DISPLAY_NEWLINE, "Left arm give semaphore");
				}
			}



			if (n == 3)
			{
				if(xSemaphoreTake(mutexMiddlePosition, portMAX_DELAY) == pdTRUE);
				if(leftRightSel == 0)
				{
					//display_log(DISPLAY_NEWLINE, "Right arm take semaphore");
				}
				else
				{
					//display_log(DISPLAY_NEWLINE, "Left arm take semaphore");
				}
			}

			ucan_send_data(COMAND_DLC, id_arm_comand_request, &posArm[n*8] );
			waitUntilPos(&posArm[n*8], leftRightSel);
			vTaskDelay(TASK_DELAY);
		}
	}
}

void waitUntilPos(uint8_t *pos, int side)
{
	uint8_t *temp = (uint8_t *)pos;
	bool closeEnough = false;

	if(side == 0){
		//display_log(DISPLAY_NEWLINE, "Right arm wait until position reached");

		while(closeEnough != true)
		{
			/*display_log(DISPLAY_NEWLINE,"right pos %x %x %x %x %x %x",pcMsgBufferRight.data[0],
					pcMsgBufferRight.data[1],
					pcMsgBufferRight.data[2],
					pcMsgBufferRight.data[3],
					pcMsgBufferRight.data[4],
					pcMsgBufferRight.data[5]
			);
			display_log(DISPLAY_NEWLINE, "right pos soll %x %x %x %x %x %x",temp[0],temp[1],
								temp[2],
								temp[3],
								temp[4],
								temp[5]);*/
			ucan_send_data(STATUS_REQEST_DLC, ROBOT_R_STATUS_REQUEST_ID, status_request );
			vTaskDelay(500);
			xQueueReceive(queueRobotRight, (void *)&pcMsgBufferRight, portMAX_DELAY);
			closeEnough = true;
			for(int i=1; i<6; i++){
						if(abs(temp[i]-pcMsgBufferRight.data[i])>0x02){
						closeEnough = false;
						break;
						}

					}

		}
		//display_log(DISPLAY_NEWLINE, "Right arm reached position");
	}
	else{
		//display_log(DISPLAY_NEWLINE, "Left arm wait until position reached");
		while(closeEnough != true)
		{
			/*display_log(DISPLAY_NEWLINE,"left pos %x %x %x %x %x %x",pcMsgBufferLeft.data[0],
								pcMsgBufferLeft.data[1],
								pcMsgBufferLeft.data[2],
								pcMsgBufferLeft.data[3],
								pcMsgBufferLeft.data[4],
								pcMsgBufferLeft.data[5]
						);
			display_log(DISPLAY_NEWLINE, "left pos soll %x %x %x %x %x %x",temp[0],temp[1],
					temp[2],
					temp[3],
					temp[4],
					temp[5]);*/
			ucan_send_data(STATUS_REQEST_DLC, ROBOT_L_STATUS_REQUEST_ID, status_request );
			vTaskDelay(500);
			xQueueReceive(queueRobotLeft, (void *)&pcMsgBufferLeft, portMAX_DELAY);
			closeEnough = true;
			for(int i=1; i<6; i++){
									if(abs(temp[i]-pcMsgBufferLeft.data[i])>0x02){
									closeEnough = false;
									break;
									}

								}
		}

		//display_log(DISPLAY_NEWLINE, "Left arm reached position");
	}


}

void init_arm()
{

	ucan_send_data(0, ROBOT_L_RESET_ID, 0 );
	ucan_send_data(0, ROBOT_R_RESET_ID, 0);

	xTaskCreate(vMoveRoboter,
	                "Arm Left",
	                ARM_TASK_STACKSIZE,
	                (void*)1,
	                ARM_TASK_PRIORITY,
	                NULL);

	xTaskCreate(vMoveRoboter,
	                "Arm Right",
	                ARM_TASK_STACKSIZE,
	                (void*)0,
	                ARM_TASK_PRIORITY,
	                NULL);
}

