/*****************************************************************************
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA  02110-1301, USA.
 *
 *****************************************************************************/
#include "ucan.h"

/* ----- Definitions --------------------------------------------------------*/
#define SIZE_MAP        100 // The size of the message link map
#define QUEUE_SIZE      10  // Length of the data queues
#define STACKSIZE_TASK  256 // Stacksize for new tasks
#define PRIORITY_TASK   2   // Taskpriority

/* ----- Datatypes -----------------------------------------------------------*/
typedef struct msg_link_s {
    QueueHandle_t queue;
    uint16_t message_id;
    uint16_t mask;
} msg_link_t;

/* ----- Globals ------------------------------------------------------------*/
static CARME_CAN_MESSAGE rx_msg;
static CARME_CAN_MESSAGE tx_msg;

static QueueHandle_t can_tx_queue;
static QueueHandle_t can_rx_queue;

static msg_link_t message_map[SIZE_MAP];
static uint16_t n_message_map;

static SemaphoreHandle_t can_semaphore;


/* ----- Functions -----------------------------------------------------------*/

/**
 * @brief writes data to the ucan
 * @param pv_data
 */
static void ucan_write_data(void *pv_data)
{
    while(true) {
        xQueueReceive(can_tx_queue, &tx_msg, portMAX_DELAY); // get latest message from queue

        /* check if semaphore is already taken */
        if(xSemaphoreTake(can_semaphore, portMAX_DELAY) == pdTRUE) {
            CARME_CAN_Write(&tx_msg); // Send message to CAN BUS
            xSemaphoreGive(can_semaphore); //return semaphore
            LOG_IF(UCAN_LOG_SENT,DISPLAY_NEWLINE, "Sent msg_id 0x%03x to can", tx_msg.id); // Log message to display
        }
    }
}

/*******************************************************************************
 *  function :    ucan_read_data
 ******************************************************************************/
/** @brief      Task which reads can messages and sends them to the can message
 *              queue
 *
 *  @type       static
 *
 *  @param[in]  void      *pv_data
 *
 *  @return     none
 ******************************************************************************/
static void ucan_read_data(void *pv_data)
{
    while(true) {
        /* check if semaphore is already taken */
        if(xSemaphoreTake(can_semaphore, portMAX_DELAY) == pdTRUE) {
            if (CARME_CAN_Read(&rx_msg) == CARME_NO_ERROR) {
                xSemaphoreGive(can_semaphore); //return semaphore
                LOG_IF(UCAN_LOG_RECEIVE,DISPLAY_NEWLINE, "Got msg_id 0x%03x", rx_msg.id); // Log message to display
                xQueueSend(can_rx_queue, &rx_msg, portMAX_DELAY);
            } else {
                xSemaphoreGive(can_semaphore); //return semaphore
            }
            vTaskDelay(100);
        }
    }
}

/*******************************************************************************
 *  function :    ucan_dispatch_data
 ******************************************************************************/
/** @brief
 *
 *  @type         global
 *
 *  @param[in]    type    Type definition, specifies the output tag
 *  @param[in]    msg     Text string
 *
 *  @return       none
 *
 *  @author       Schmocker Aaron
 *
 ******************************************************************************/
/* Read incomming can messages from the rx_queue and forwards them according to the queue map */
static void ucan_dispatch_data(void *pv_data)
{
    CARME_CAN_MESSAGE tmp_msg;
    QueueHandle_t queue;
    bool match = false;

    while(true) {
        xQueueReceive(can_rx_queue, &tmp_msg, portMAX_DELAY); // get a message from the rx queue
        match = false;
        /* search for the corresponding queue handles */
        for(int i = 0; i < n_message_map; i++) {
            /* Apply the message mask to the tmp id and search for matches in the message_map*/
            if((tmp_msg.id & message_map[i].mask) == message_map[i].message_id) {
                queue = message_map[i].queue;
                match = true;
                LOG_IF(UCAN_LOG_DISPATCH, DISPLAY_NEWLINE, "Dispatched msg_id 0x%03x", tmp_msg.id);
                xQueueSend(queue, &tmp_msg, portMAX_DELAY); // forward it to the queue
            }
        }

        /* if there were no matches, drop messages */
        if(!match) {
            LOG_IF(UCAN_LOG_DROP,DISPLAY_NEWLINE, "Dropped msg_id 0x%03x", tmp_msg.id);
        }
    }
}

/* setup acceptance filter */
static void ucan_setup_acceptance_filter(void)
{
    CARME_CAN_ACCEPTANCE_FILTER af;

    /* set the SJA1000 chip in running mode */
    CARME_CAN_SetMode(CARME_CAN_DF_RESET);

    /* single filter */
    af.afm = MODE_SINGLE;

    /* Unmask the important bits by setting them to Zero */
    af.amr[0] = 0x00; // unmask bit 0 - 7
    af.amr[1] = 0x1f; // unmask bit 8 - 6
    af.amr[2] = 0xff; // don't care in Mode with normal id length
    af.amr[3] = 0xff; // don't care in Mode with normal id length

    /* Set the bits which have to be high */
    af.acr[0] = 0xAA; // 11001100
    af.acr[1] = 0xA0; // 11000000
    af.acr[2] = 0x00; // don't care in Mode with normal id length
    af.acr[3] = 0x00; // don't care in Mode with normal id length

    /* Set the AF */
    CARME_CAN_SetAcceptaceFilter(&af);

    /* Set the SJA1000 chip in running mode */
    CARME_CAN_SetMode(CARME_CAN_DF_NORMAL);
}

/* Set a message mask to map multiple message to a queue*/
bool ucan_link_message_to_queue_mask(uint16_t mask, uint16_t message_id, QueueHandle_t queue)
{
    /* Check if there is enough space left */
    if(n_message_map >= SIZE_MAP) {
        return false;
    }

    /* increment message counter, save message_id, the mask and the corresponding queue */
    message_map[n_message_map].message_id = message_id;
    message_map[n_message_map].queue = queue;
    message_map[n_message_map].mask = mask;
    n_message_map++;

    return true;

}

/* Link a single message type to a queue */
bool ucan_link_message_to_queue(uint16_t message_id, QueueHandle_t queue)
{
    ucan_link_message_to_queue_mask(0x0FFF, message_id, queue);
}

/* Initialize can hardware */
bool ucan_init(void)
{
    GPIO_InitTypeDef g;

    /* Init gpio for can */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    g.GPIO_Pin = GPIO_Pin_0;
    g.GPIO_Mode = GPIO_Mode_OUT;
    g.GPIO_OType = GPIO_OType_PP;
    g.GPIO_Speed = GPIO_Speed_2MHz;
    g.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &g);

    /* Init can chip */
    CARME_CAN_Init(CARME_CAN_BAUD_250K, CARME_CAN_DF_RESET);
    CARME_CAN_SetMode(CARME_CAN_DF_NORMAL);

    /* Setup acceptance filter */
    //ucan_setup_acceptance_filter();

    /* Clear the rx and tx CAN message */
    for(int i = 0; i < 7; i++) {
        rx_msg.data[i] = 0;
        tx_msg.data[i] = 0;
    }

    /* Create message queues for can communication */
    can_tx_queue = xQueueCreate(QUEUE_SIZE, sizeof(CARME_CAN_MESSAGE));
    can_rx_queue = xQueueCreate(QUEUE_SIZE, sizeof(CARME_CAN_MESSAGE));

    /* create binary semaphore */
    can_semaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(can_semaphore);

    n_message_map = 0;

    /* Spawn tasks */
    xTaskCreate(ucan_write_data, "CAN_Write_Task", STACKSIZE_TASK, NULL, PRIORITY_TASK, NULL);
    xTaskCreate(ucan_read_data, "CAN_Read_Task", STACKSIZE_TASK, NULL, PRIORITY_TASK, NULL);
    xTaskCreate(ucan_dispatch_data, "CAN_Dispatch_Task", STACKSIZE_TASK, NULL, PRIORITY_TASK, NULL);

    return true;
}

/* Send data to the output message queue */
bool ucan_send_data(uint8_t n_data_bytes, uint16_t msg_id, const uint8_t *data)
{
    CARME_CAN_MESSAGE tmp_msg;

    /* Setup basic CAN message header for temporary message */
    tmp_msg.id = msg_id; // Message ID
    tmp_msg.rtr = 0; // Something weird
    tmp_msg.ext = 0; // Something weird
    tmp_msg.dlc = n_data_bytes; // Number of bytes

    memcpy(tmp_msg.data, data, min(n_data_bytes, 8)); // copy databytes to output buffer but only 8bytes
    LOG_IF(UCAN_LOG_SENDING,DISPLAY_NEWLINE, "Insert msg_id 0x%03x to queue", msg_id); // Log message to display
    xQueueSend(can_tx_queue, &tmp_msg, portMAX_DELAY); // Send message to the message queue

    return true;
}
