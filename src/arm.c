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

#include <carme_io1.h>

#include "arm.h"
#include "ucan.h"
#include "bcs.h"

//----- Macros -----------------------------------------------------------------
#define BUTTON_T0 0x01
#define BUTTON_T1 0x02
#define BUTTON_T2 0x03
#define BUTTON_T3 0x04

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
            display_log(DISPLAY_NEWLINE, "Going to position %u",n);

			if(n == 1)
			{
				xSemaphoreGive(mutexMiddlePosition);
                //display_log(DISPLAY_NEWLINE, "give semaphore");
			}


			if (n == 3)
			{

                if(leftRightSel == 1)
                {
                	xSemaphoreTake(bcs_left_end_semaphore, portMAX_DELAY);
                }
                else
                {
                	xSemaphoreTake(bcs_right_end_semaphore, portMAX_DELAY);
                }
                xSemaphoreTake(mutexMiddlePosition, portMAX_DELAY);
                //display_log(DISPLAY_NEWLINE, "take semaphore");
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


void vManualArmMovment(void *pvData)
{
	uint8_t buttonData;
	uint8_t switchData;

	uint8_t posManuel[8] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	bool increment = false;
	bool leftSelect = false;

	CARME_CAN_MESSAGE pcMsgBufferRightM;
	CARME_CAN_MESSAGE pcMsgBufferLeftM;

	while(1)
	{
		CARME_IO1_BUTTON_Get(&buttonData);
		CARME_IO1_SWITCH_Get(&switchData);



		switch(buttonData){

		case BUTTON_T0:
			if(increment == true){
				posManuel[1]++;
			}
			else{
				posManuel[1]--;
			}
			break;

		case BUTTON_T1:
			if(increment == true){
				posManuel[2]++;;
			}
			else{
				posManuel[2]--;
			}
			break;

		case BUTTON_T2:
			if(increment == true){
				posManuel[3]++;
			}
			else{
				posManuel[3]--;
			}
			break;

		case BUTTON_T3:
			if(increment == true){
				posManuel[4]++;
			}
			else
			{
				posManuel[4]--;
			}
			break;
		default:
			break;
		}

		// not sure if it's working because of uint8 and negative values
		if(posManuel[1] == BASIS_MIN)     posManuel[1] = BASIS_MIN;
		if(posManuel[1] == BASIS_MAX)     posManuel[1] = BASIS_MAX;

		if(posManuel[2] == SHOULDER_MIN)  posManuel[2] = SHOULDER_MIN;
	    if(posManuel[2] == SHOULDER_MAX)  posManuel[2] = SHOULDER_MAX;

	    if(posManuel[3] == ELBOW_MIN)     posManuel[3] = ELBOW_MIN;
	    if(posManuel[3] == ELBOW_MAX)     posManuel[3] = ELBOW_MAX;

	    if(posManuel[4] == HAND_MIN)      posManuel[4] = HAND_MIN;
	    if(posManuel[4] == HAND_MAX)      posManuel[4] = HAND_MAX;

	    if(leftSelect == true){
	    	ucan_send_data(COMAND_DLC, ROBOT_L_COMAND_REQUEST_ID, posManuel);
	    	vTaskDelay(5);
	    	ucan_send_data(STATUS_REQEST_DLC, ROBOT_L_STATUS_REQUEST_ID, status_request );
	    	xQueueReceive(queueRobotLeft, (void *)&pcMsgBufferLeftM, portMAX_DELAY);
	    	vTaskDelay(5);
	    }
	    else{
	    	ucan_send_data(COMAND_DLC, ROBOT_R_COMAND_REQUEST_ID, posManuel);
	        vTaskDelay(5);
	    	ucan_send_data(STATUS_REQEST_DLC, ROBOT_R_STATUS_REQUEST_ID, status_request );

	    	vTaskDelay(5);
	    }

	}

}
