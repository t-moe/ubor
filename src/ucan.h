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
#include <semphr.h>

#include "display.h"

/*----- Defines --------------------------------------------------------------*/

#define UCAN_LOG_SENT 0 //!< Enable or disable logging by setting this to either true or false
#define UCAN_LOG_SENDING 0 //!< Set loglevel to sent messages
#define UCAN_LOG_RECEIVE 0 //!< Set loglevel to recieved messages
#define UCAN_LOG_DISPATCH 0 //!< Set loglevel to dispatched messages
#define UCAN_LOG_DROP 0 //!< Set loglevel to dropped messages (unable to dispatch)

#define LOG_IF(cond,...) do{ if(cond) display_log(__VA_ARGS__); } while(false) // Write to log using loglevels

/*----- Data types -----------------------------------------------------------*/

/*----- Function prototypes --------------------------------------------------*/
bool ucan_init(void);
bool ucan_send_data(uint8_t n_data_bytes, uint16_t msg_id, const uint8_t *data);
bool ucan_link_message_to_queue(uint16_t message_id, QueueHandle_t queue);
bool ucan_link_message_to_queue_mask(uint16_t mask, uint16_t message_id, QueueHandle_t queue);

#endif // UCAN_H
