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
#define BUTTON_T2 0x04
#define BUTTON_T3 0x08

//----- Data types -------------------------------------------------------------

//----- Function prototypes ----------------------------------------------------

//----- Data -------------------------------------------------------------------
static QueueHandle_t queueRobotLeft;
static QueueHandle_t queueRobotRight;
static QueueHandle_t queueRobotManual;
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
			/*0 Nullposition     */ {0x02, 0x00, 0x19, 0x1A, 0x21, 0x01, 0x00, 0x00},
			/*1 StartPosition    */ {0x02, 0x00, 0x1F, 0x1A, 0x21, 0x01, 0x00, 0x00},
			/*2 Zp1 FB Links     */ {0x02, 0x00, 0x1F, 0x1A, 0x21, 0x00, 0x00, 0x00},
			/*3 Zp2 FB Links     */ {0x02, 0x00, 0x19, 0x1A, 0x21, 0x00, 0x00, 0x00},
			/*4 G offen FB Links */ {0x02, 0xee, 0x00, 0x40, 0x21, 0x00, 0x00, 0x00},
			/*5 G zu FB Links    */ {0x02, 0xD2, 0x00, 0x2b, 0x21, 0x00, 0x00, 0x00},
			/*6 Zp1 FB Mitte     */ {0x02, 0xD2, 0x1c, 0x1d, 0x21, 0x00, 0x00, 0x00},
			/*7 Zp2 FB Mitte     */ {0x02, 0xD2, 0x1c, 0x1d, 0x21, 0x01, 0x00, 0x00},
			/*8 G zu FB Mitte    */ {0x02, 0xD2, 0x13, 0x1d, 0x21, 0x01, 0x00, 0x00},
			/*9 G offen FB Mitte */ {0x02, 0xD2, 0x00, 0x36, 0x21, 0x01, 0x00, 0x00},
			};


		mutexMiddlePosition = xSemaphoreCreateBinary();
		xSemaphoreGive(mutexMiddlePosition);

		queueRobotRight = xQueueCreate(MSG_QUEUE_SIZE, sizeof(CARME_CAN_MESSAGE));
		ucan_link_message_to_queue(ROBOT_R_STATUS_RETURN_ID, queueRobotRight);

		posArm = (uint8_t *)posArmRight;
		id_arm_comand_request = ROBOT_R_COMAND_REQUEST_ID;

		ucan_send_data(0, ROBOT_R_RESET_ID, 0);
	}

	if(leftRightSel == 1)
	{
	 	static uint8_t posArmLeft[10][8] = {
				/*                      Arm	   B     S     E     H     G     x     x */
	 									{0x02, 0x00, 0x19, 0x1A, 0x21, 0x01, 0x00, 0x00}, //Warte Position ECTS Abholen
	 									{0x02, 0x00, 0x1F, 0x1A, 0x21, 0x01, 0x00, 0x00}, //ECTS Holen Mitte
	 									{0x02, 0x00, 0x1F, 0x1A, 0x21, 0x00, 0x00, 0x00}, //ECTS Greifen
	 									{0x02, 0x00, 0x19, 0x1A, 0x21, 0x00, 0x00, 0x00}, //ECTS Anheben
	 									{0x02, 0x12, 0x00, 0x40, 0x21, 0x00, 0x00, 0x00}, //Drehen bis pos 1
	 									{0x02, 0x2D, 0x00, 0x2b, 0x21, 0x00, 0x00, 0x00}, //Drehen bis pos 2 bereit zum ablegen
	 									{0x02, 0x2D, 0x1c, 0x1d, 0x21, 0x00, 0x00, 0x00}, //ECTS Ablegen
	 									{0x02, 0x2D, 0x1c, 0x1d, 0x21, 0x01, 0x00, 0x00}, //ECTS Ablegen
	 									{0x02, 0x2D, 0x13, 0x1d, 0x21, 0x01, 0x00, 0x00}, //Arm auserhalb ects
	 									{0x02, 0x2D, 0x00, 0x36, 0x21, 0x01, 0x00, 0x00}, //Arm ausserhalb mitte
				};
	 	posArm = (uint8_t *)posArmLeft;
	 	id_arm_comand_request = ROBOT_L_COMAND_REQUEST_ID;

	 	queueRobotLeft = xQueueCreate(MSG_QUEUE_SIZE, sizeof(CARME_CAN_MESSAGE));
	 	ucan_link_message_to_queue(ROBOT_L_STATUS_RETURN_ID, queueRobotLeft);

		ucan_send_data(0, ROBOT_L_RESET_ID, 0 );
	}

	vTaskDelay(500); //needed for reset to be applied

	while(1)
	{

		for(int n = 0; n < 10; n++)
		{
            display_log(DISPLAY_NEWLINE, "Going to position %u",n);

			if(n == 6){ //before we want to access the mid position
				display_log(DISPLAY_NEWLINE, "take semaphore");
				xSemaphoreTake(mutexMiddlePosition, portMAX_DELAY);
			}





			ucan_send_data(COMAND_DLC, id_arm_comand_request, &posArm[n*8] );
			waitUntilPos(&posArm[n*8], leftRightSel);

			if(n == 0) { //after we moved above the position were we want to grab Mitte
				int8_t pos;
				if(leftRightSel == 1) {
					xQueueReceive(bcs_left_end_queue,&pos,portMAX_DELAY);
				} else {
					xQueueReceive(bcs_right_end_queue,&pos,portMAX_DELAY);
				}
				display_log(DISPLAY_NEWLINE,"block is at pos %d",pos);
			}

			if(n == 9){ //after we moved out of the mid position
				display_log(DISPLAY_NEWLINE, "give semaphore");
				xSemaphoreGive(mutexMiddlePosition);
				xSemaphoreGive(bcs_mid_start_semaphore);
			}


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
			vTaskDelay(200);
			ucan_send_data(STATUS_REQEST_DLC, ROBOT_R_STATUS_REQUEST_ID, status_request );


			xQueueReceive(queueRobotRight, (void *)&pcMsgBufferRight, portMAX_DELAY);
			closeEnough = true;
            for(int i=1; i<6; i++){
                if(abs(temp[i]-pcMsgBufferRight.data[i])>0x01){
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

			vTaskDelay(200);

			ucan_send_data(STATUS_REQEST_DLC, ROBOT_L_STATUS_REQUEST_ID, status_request );
		
			xQueueReceive(queueRobotLeft, (void *)&pcMsgBufferLeft, portMAX_DELAY);
			closeEnough = true;
			for(int i=1; i<6; i++){
									if(abs(temp[i]-pcMsgBufferLeft.data[i])>0x01){
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
/*
	xTaskCreate(vManualArmMovment,
				"Manual Arm Movement",
				ARM_TASK_STACKSIZE,
				NULL,
				ARM_TASK_PRIORITY,
				NULL);*/
}


/*
 * Switch 0: select left or right arm
 * Switch 1: increment or decrement position value
 * Switch 2: gripper open or close
 *
 * Button 0: change value of basis
 * Button 1: change value of shoulder
 * Button 2: change value of elbow
 * Button 3: change value of hand
*/


void vManualArmMovment(void *pvData)
{
	uint8_t buttonData;
	uint8_t switchData;

	uint8_t posManuel[8] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	bool increment = false;
	bool leftSelect = false;

	CARME_CAN_MESSAGE pcMsgBuffer;
	queueRobotManual = xQueueCreate(MSG_QUEUE_SIZE, sizeof(CARME_CAN_MESSAGE));
	ucan_link_message_to_queue(ROBOT_R_STATUS_RETURN_ID, queueRobotManual);
	ucan_link_message_to_queue(ROBOT_L_STATUS_RETURN_ID, queueRobotManual);

	while(1)
	{
		CARME_IO1_BUTTON_Get(&buttonData);
		CARME_IO1_SWITCH_Get(&switchData);

		if( (switchData & MASK_SWITCH_0) != 0){
			leftSelect = true;
		}
		else{
			leftSelect = false;
		}

		if( (switchData & MASK_SWITCH_1 ) != 0){
			increment = true;
		}
		else{
			increment = false;
		}

		if( (switchData & MASK_SWITCH_2) != 0){
			posManuel[5] = GRIPPER_MAX;
		}
		else{
			posManuel[5] = GRIPPER_MIN;
		}


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


	    if(leftSelect == true){
	    	ucan_send_data(COMAND_DLC, ROBOT_L_COMAND_REQUEST_ID, posManuel);
	    	vTaskDelay(20);
	    	ucan_send_data(STATUS_REQEST_DLC, ROBOT_L_STATUS_REQUEST_ID, status_request );
	    	vTaskDelay(20);
	    	xQueueReceive(queueRobotManual, (void *)&pcMsgBuffer, portMAX_DELAY);
	    	vTaskDelay(20);
	    }
	    else{
	    	ucan_send_data(COMAND_DLC, ROBOT_R_COMAND_REQUEST_ID, posManuel);
	        vTaskDelay(20);
	    	ucan_send_data(STATUS_REQEST_DLC, ROBOT_R_STATUS_REQUEST_ID, status_request );
	    	xQueueReceive(queueRobotManual, (void *)&pcMsgBuffer, portMAX_DELAY);
	    	vTaskDelay(20);
	    }

	    display_log(DISPLAY_NEWLINE,"Position: %x %x %x %x %x %x",pcMsgBufferRight.data[0],
	    					pcMsgBuffer.data[1],
	    					pcMsgBuffer.data[2],
	    					pcMsgBuffer.data[3],
	    					pcMsgBuffer.data[4],
	    					pcMsgBuffer.data[5]);

	}

}

