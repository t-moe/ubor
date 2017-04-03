#ifndef UCAN_H
#define UCAN_H

/*----- Header-Files ---------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include <carme.h>
#include <can.h>
#include <stm32f4xx.h>
#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

#include "display.h"

/*----- Defines --------------------------------------------------------------*/

/* CAN status request message id's for the conveyor belt system*/
#define CAN_ID_CONVL_STAT_REQ 0x110 // request status left conveyor belt 
#define CAN_ID_CONVM_STAT_REQ 0x120 // request status middle conveyor belt
#define CAN_ID_CONVR_STAT_REQ 0x130 // request status right conveyor belt

/* CAN status answer message id's for the conveyor belt system*/
#define CAN_ID_CONVL_STAT_ANSW 0x111 // status answer left conveyor belt
#define CAN_ID_CONVM_STAT_ANSW 0x121 // status answer middle conveyor belt
#define CAN_ID_CONVR_STAT_ANSW 0x131 // status answer right conveyor belt

/* CAN command message id's for the conveyor belt system*/
#define CAN_ID_CONVL_CMD 0x112 // send command to left conveyor belt
#define CAN_ID_CONVM_CMD 0x122 // send command to middle conveyor belt
#define CAN_ID_CONVR_CMD 0x132 // send command to right conveyor belt

/* CAN command message answer id's for the conveyor belt system*/
#define CAN_ID_CONVL_CMD_ANSW 0x113 // command answer left conveyor belt
#define CAN_ID_CONVM_CMD_ANSW 0x123 // command answer middle conveyor belt
#define CAN_ID_CONVR_CMD_ANSW 0x133 // command answer right conveyor belt

/* CAN reset message id's for the conveyor belt system*/
#define CAN_ID_CONVL_RES 0x11F // reset left conveyor belt
#define CAN_ID_CONVM_RES 0x12F // reset middle conveyor belt
#define CAN_ID_CONVR_RES 0x13F // reset right conveyor belt

/* CAN message id's for the dispatcher */
#define CAN_ID_DISP_STAT_REQ 0x140 // request status dispatcher
#define CAN_ID_DISP_STAT_ANSW 0x141 // status answer dispatcher
#define CAN_ID_DISP_CMD 0x142 // send command to dispatcher
#define CAN_ID_DISP_CMD_ANSW 0x143 // command answer dispatcher
#define CAN_ID_DISP_RES 0x14F // reset dispatcher

/* CAN status request message id's for the arms */
#define CAN_ID_ARML_STAT_REQ 0x150 // request status left arm
#define CAN_ID_ARMR_STAT_REQ 0x160 // request status right arm

/* Can status answer message id's for the arms */
#define CAN_ID_ARML_STAT_ANSW 0x151 // answer status left arm
#define CAN_ID_ARMR_STAT_ANSW 0x161 // answer status right arm

/* CAN command message id's for the arms */
#define CAN_ID_ARML_CMD 0x152 // send command to left arm
#define CAN_ID_ARMR_CMD 0x162 // send command to right arm

/* CAN command message answer id's for the arms */
#define CAN_ID_ARML_CMD_ANSW 0x153 // command answer left arm
#define CAN_ID_ARMR_CMD_ANSW 0x163 // command answer right arm

/* CAN resed command id's for the arms */
#define CAN_ID_ARML_RES 0x15F // reset left arm
#define CAN_ID_ARMR_RES 0x16F // reset right arm


/*----- Data types -----------------------------------------------------------*/

/*----- Function prototypes --------------------------------------------------*/
bool ucan_init(void);
bool ucan_send_data(uint8_t n_data_bytes, uint16_t msg_id, const uint8_t *data);
bool ucan_link_message_to_queue(uint16_t message_id, QueueHandle_t queue);

QueueHandle_t get_queue_by_id(uint16_t message_id);

#endif // UCAN_H
