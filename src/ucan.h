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
#define QUEUE_SIZE 10

/*----- Data types -----------------------------------------------------------*/

/*----- Function prototypes --------------------------------------------------*/
bool ucan_init(void);
bool ucan_send_data(uint8_t n_data_bytes, uint16_t msg_id, const uint8_t *data);
bool ucan_receive_data(void);
bool ucan_link_message_to_queue(uint16_t message_id, QueueHandle_t queue);

#endif // UCAN_H
