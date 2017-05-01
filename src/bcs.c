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
#include "display.h"
#include <FreeRTOS.h>
#include <stdio.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include <stdbool.h>
#include <string.h>
#include "ucan.h"
#include "bcs.h"

// -------------------- Configuration  ------------
#define STACKSIZE_TASK  256 //!< Stack size of all bcs tasks
#define PRIORITY_TASK   2 //!< Priority of all bcs tasks

#define MAX_BLOCK_COUNT 4 //!< Number of blocks to work with. Must be between 2 and 4

// ------------------ Implementation --------------

#define SWITCH  ((volatile unsigned char*)(0x6C000400))

/**
  @brief BCS can message structure
  */
typedef struct {
    uint16_t subid; //!< Id offset of the base-id
    uint8_t length; //!< Data-Length of the CAN message
    uint8_t data[]; //!< CAN data
} message_t;


/**
  @brief Status message received by belt conveyer system
  */
typedef struct __attribute__((__packed__))
{
    uint8_t error; //!< error or not
    uint8_t engine; //!< engines running or not
    uint8_t lightbarrier; //!< light  barrier status
    uint8_t detection; //!< detection status
    uint16_t position; //!< block position (along moving axis)
    int8_t location; //!< block location
}
status_t;

//Messages for conveyer system
static const message_t msg_status_request = {0,1,{1}};
static const uint16_t msg_status_response_id = 1;
static const message_t msg_cmd_start = {2,3,{1,0,0}};
static const message_t msg_cmd_stop = {2,3,{2,0,0}};
static const message_t msg_cmd_stoppos = {2,3,{3,0x00,0xB6}};
static const message_t msg_cmd_done= {2,3,{4,0,0}};
static const message_t msg_cmd_reset= {0xF,0};

//Messages for dispatcher
static const message_t msg_cmd_disp_initial_pos = {0x142, 3, {1, 0, 100}};
static const message_t msg_cmd_disp_start_right = {0x142, 3, {1, 0xE4, 100}};
static const message_t msg_cmd_disp_move_right = {0x142, 3, {1, 0x32, 50}};
static const message_t msg_cmd_disp_start_left = {0x142, 3, {1, 0x1C, 100}};
static const message_t msg_cmd_disp_move_left = {0x142, 3, {1, 0xCE, 50}};

//Messages Queues to receive data from can
static QueueHandle_t ucan_queue_mid;
static QueueHandle_t ucan_queue_left;
static QueueHandle_t ucan_queue_right;

//Semaphores
static SemaphoreHandle_t bcs_left_start_semaphore; //Given by mid task, Taken by left
static SemaphoreHandle_t bcs_mid_start_semaphore; //Given by arm Tasks, taken by Mid task
static SemaphoreHandle_t bcs_right_start_semaphore; //Given by mid task, Taken by right
static SemaphoreHandle_t bcs_left_free_semaphore; //Given by left task, Taken by arm task
static SemaphoreHandle_t bcs_mid_free_semaphore; //Given by mid task, Taken by left/right task
static SemaphoreHandle_t bcs_right_free_semaphore; //Given by right task, Taken by arm task

static QueueHandle_t bcs_left_end_queue; //Given by left task, Taken by Arm Left
static QueueHandle_t bcs_right_end_queue; //Given by right task, taken by Arm Right



/**
 * @brief       Send a CAN message to the belt conveyer system
 * @type        static
 * @param[in]   msg         The message to send
 * @param[in]   baseaddress The base adress for the id
 * @return      none
 **/
static void bcs_send_msg(const message_t* msg, uint16_t baseaddr)
{
    ucan_send_data(msg->length,baseaddr + msg->subid, msg->data);
    vTaskDelay(5);
}


/**
 * @brief       Waits until a block is detected on the specified belt
 * @type        static
 * @param[in]   belt            The belt to wait for a block
 * @param[in]   queue           The queue to receive the CAN data from
 * @param[out]  tmp_message     The buffer where to store the temporary CAN messages
 * @return      A pointer to the status message that was received (valid as long as tmp_message is valid)
 **/
static status_t* bcs_await_block(enum belt_select belt, QueueHandle_t ucan_queue, CARME_CAN_MESSAGE* tmp_message)
{
    uint8_t statR = display_log(DISPLAY_NEWLINE,"Waiting on block...");
    uint16_t wait_count = 0;
    status_t* status;

    /* Wait until the block is fully detected */
    while(true) {

        /* Request status */
        bcs_send_msg(&msg_status_request,belt);

        /* Wait on status response */
        do {
            if(xQueueReceive(ucan_queue,tmp_message,4000)==pdFALSE) {
                wait_count++;
                display_log(statR,"Waiting on block (%u): timeout", wait_count);
            }
        } while(tmp_message->id != belt+msg_status_response_id); //repeat until correct response message arrives

        status = (status_t*)&(tmp_message->data);
        wait_count++;

        /* Block detected */
        if(status->detection == 3) {
            display_log(statR,"Waiting on block. Found! position %04x location %d", status->position, status->location );
            return status;
        }

        display_log(statR,"Waiting on block (%u): detection: %u pos: %04x",wait_count,status->detection, status->position);
        vTaskDelay(100);

        /* Timeout */
        if(wait_count >= 100) {
            bcs_send_msg(&msg_cmd_done,belt);
            display_log(statR,"Waiting on block (%u): Aborted",wait_count);
            return NULL;
        }
    }
}



/**
 * @brief       Prepares a block drop operation to a specific belt
 * @type        global
 * @param[in]   belt    The belt a block will be dropped to
 * @return      None
 **/
void bcs_prepare_drop(enum belt_select belt)
{

    switch(belt) {
    case belt_left:
        xSemaphoreTake(bcs_left_free_semaphore,portMAX_DELAY);
        break;
    case belt_right:
        xSemaphoreTake(bcs_right_free_semaphore,portMAX_DELAY);
        break;
    case belt_mid:
        xSemaphoreTake(bcs_mid_free_semaphore,portMAX_DELAY);
        break;

    }
}

/**
 * @brief       Signal that a block has been dropped on a belt
 * @type        global
 * @param[in]   belt    The belt the block has been dropped onto
 * @return      None
 **/
void bcs_signal_dropped(enum belt_select belt)
{
    switch(belt) {
    case belt_left:
        xSemaphoreGive(bcs_left_start_semaphore);
        break;
    case belt_right:
        xSemaphoreGive(bcs_right_start_semaphore);
        break;
    case belt_mid:
        xSemaphoreGive(bcs_mid_start_semaphore);
        break;

    }
}

/**
 * @brief       Signal that a block has been removed from a belt and the belt is free again
 * @type        global
 * @param[in]   belt    The belt that is now free
 * @return      None
 **/
void bcs_signal_band_free(enum belt_select belt)
{
    switch(belt) {
    case belt_left:
        xSemaphoreGive(bcs_left_free_semaphore);
        break;
    case belt_right:
        xSemaphoreGive(bcs_right_free_semaphore);
        break;
    case belt_mid:
        xSemaphoreGive(bcs_mid_free_semaphore);
        break;

    }
}

/**
 * @brief       Awaits until a block has been dropped on the specific belt
 * @type        static
 * @param[in]   belt    The belt we want to wait for a block
 * @param[in]   allow_skip Whether or not we want to abort if now block is found after a timeout.
 * @return      None
 **/
static void bcs_await_drop(enum belt_select belt, bool allow_skip)
{

    switch(belt) {
    case belt_left:
        xSemaphoreTake(bcs_left_start_semaphore,portMAX_DELAY);
        break;
    case belt_right:
        xSemaphoreTake(bcs_right_start_semaphore,portMAX_DELAY);
        break;
    case belt_mid:
        if(allow_skip) {
            xSemaphoreTake(bcs_mid_start_semaphore,2000); //try to aquire mutex anyway, in case it was already there
            //if mutex could not be taken => go on (skipping is allowed)
        } else {
            xSemaphoreTake(bcs_mid_start_semaphore,portMAX_DELAY);
        }
        break;

    }

}


/**
 * @brief       Instructs the system that we want to grab a block from the bcs
 * @type        global
 * @param[in]   belt    The belt we want to grab a block from
 * @return      Position of the block (relative to the center of the band)
 **/
int8_t bcs_grab(enum belt_select belt)
{
    int8_t pos;
    if(belt == belt_left) {
        xQueueReceive(bcs_left_end_queue,&pos,portMAX_DELAY);
    } else if(belt == belt_right) {
        xQueueReceive(bcs_right_end_queue,&pos,portMAX_DELAY);
    }
    return pos;
}

/**
 * @brief       bcs main task
 * @type        static
 * @param[in]   pv_data     The belt we want to run the task for. Pass a value of belt_select here
 * @return      None
 **/
static void bcs_task(void *pv_data)
{
    enum belt_select belt = (enum belt_select)pv_data;

    QueueHandle_t ucan_queue;

    switch(belt) {
    case belt_left:
        ucan_queue = ucan_queue_left;
        break;
    case belt_right:
        ucan_queue = ucan_queue_right;
        break;
    case belt_mid:
        ucan_queue = ucan_queue_mid;
        break;

    }


    //only for mid task
    bool move_left = true; //whether the dispatcher should move left or right
    uint8_t mid_start_without_mutex_count = 0; //The number of times we started the mid band without awaiting the mutex


    while(true) {
        //----- Step 0: Reset band
        bcs_send_msg(&msg_cmd_reset,belt);
        display_log(DISPLAY_NEWLINE,"reset band");

        //----- Step 1: Wait on a block (take semaphore), before we start the band ------------------
        bool allow_skip = false;
        if(belt == belt_mid) {
            if(mid_start_without_mutex_count < MAX_BLOCK_COUNT) { //we are in the init phase
                mid_start_without_mutex_count++;
                allow_skip = true;
            }

            display_log(DISPLAY_NEWLINE,"Reset dispatcher");
            bcs_send_msg(&msg_cmd_disp_initial_pos,0);
        }

        bcs_await_drop(belt,allow_skip);


        //----- Step 2: Start the band, and wait till we detect the block
        bcs_send_msg(&msg_cmd_start,belt);
        bcs_send_msg(&msg_cmd_stoppos,belt);
        display_log(DISPLAY_NEWLINE,"start band");

        CARME_CAN_MESSAGE tmp_message;
        status_t* status = bcs_await_block(belt,ucan_queue,&tmp_message);
        if(status==NULL) { //timeout
            while(true);
            //TODO: Listen on button press
        }

        //----- Step 3 (only mid band): Move the dispatcher so we don't interfere with the coming block
        if(belt== belt_mid) {
            if(*SWITCH&0x01) { //Manual Direction selection
                move_left = *SWITCH&0x02; //read direction from switch
            }
            display_log(DISPLAY_NEWLINE,"Making dispatcher ready for moving %s",move_left ? "left" : "right");
            bcs_send_msg(move_left ? &msg_cmd_disp_start_left : &msg_cmd_disp_start_right,0);
        }
        vTaskDelay(2000); //let block move to the end of the band


        //----- Step 4: Mark the block ready for further processing (give semaphore) and optionally move the dispatcher (only mid band).
        switch(belt) {
        case belt_left:
            //bcs_prepare_pickup(belt_left)
            xQueueSend(bcs_left_end_queue,&(status->location),portMAX_DELAY);
            break;
        case belt_right:
            //bcs_prepare_pickup(belt_right)
            xQueueSend(bcs_right_end_queue,&(status->location),portMAX_DELAY);
            break;
        case belt_mid:
            bcs_prepare_drop(move_left ? belt_left : belt_right);

            display_log(DISPLAY_NEWLINE,"Dispatcher moves %s",move_left ? "left" : "right");
            bcs_send_msg(move_left ? &msg_cmd_disp_move_left : &msg_cmd_disp_move_right,0);
            vTaskDelay(1000); //let dispatcher move block away

            bcs_signal_dropped(move_left ? belt_left : belt_right);
            move_left = ! move_left;
            break;


        }

        //---- Step 5: Tell the band that we're finished
        bcs_send_msg(&msg_cmd_done,belt);
        if(belt == belt_mid) {
            bcs_signal_band_free(belt_mid);
        }
    }
}

/**
 * @brief       Initializes the belt conveyer system and starts the belt tasks
 * @type        global
 * @return      None
 **/
void bcs_init()
{
    bcs_left_start_semaphore = xSemaphoreCreateBinary();
    bcs_right_start_semaphore = xSemaphoreCreateBinary();
    bcs_mid_start_semaphore = xSemaphoreCreateBinary();

    bcs_left_free_semaphore = xSemaphoreCreateBinary();
    bcs_mid_free_semaphore = xSemaphoreCreateBinary();
    bcs_right_free_semaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(bcs_left_free_semaphore);
    //xSemaphoreGive(bcs_mid_free_semaphore);
    xSemaphoreGive(bcs_right_free_semaphore);

    bcs_left_end_queue = xQueueCreate(1,sizeof(int8_t));
    bcs_right_end_queue = xQueueCreate(1,sizeof(int8_t));

    xTaskCreate(bcs_task,"mid",STACKSIZE_TASK,(void*)belt_mid,PRIORITY_TASK,NULL);
    xTaskCreate(bcs_task,"left",STACKSIZE_TASK,(void*)belt_left,PRIORITY_TASK,NULL);
    xTaskCreate(bcs_task,"right",STACKSIZE_TASK,(void*)belt_right,PRIORITY_TASK,NULL);

    ucan_queue_right = xQueueCreate(1,sizeof(CARME_CAN_MESSAGE));
    ucan_queue_left = xQueueCreate(1,sizeof(CARME_CAN_MESSAGE));
    ucan_queue_mid = xQueueCreate(1,sizeof(CARME_CAN_MESSAGE));

    ucan_link_message_to_queue_mask(0xFF0,belt_mid,ucan_queue_mid);
    ucan_link_message_to_queue_mask(0xFF0,belt_left,ucan_queue_left);
    ucan_link_message_to_queue_mask(0xFF0,belt_right,ucan_queue_right);
}
