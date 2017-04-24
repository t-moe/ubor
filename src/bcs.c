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
#define STACKSIZE_TASK  256
#define PRIORITY_TASK   2

#define MAX_BLOCK_COUNT 4

// ------------------ Implementation --------------

typedef struct {
    uint16_t subid;
    uint8_t length;
    uint8_t data[];
} message_t;

typedef struct __attribute__((__packed__))
{
    uint8_t error;
    uint8_t engine;
    uint8_t lightbarrier;
    uint8_t detection;
    uint16_t position;
    int8_t location;
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




static void bcs_send_msg(const message_t* msg, uint16_t baseaddr)
{
    ucan_send_data(msg->length,baseaddr + msg->subid, msg->data);
    vTaskDelay(5);
}


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


void bcs_prepare_drop(enum belt_select belt) {

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

void bcs_signal_dropped(enum belt_select belt) {
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

void bcs_signal_band_free(enum belt_select belt) {
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

int8_t bcs_grab(enum belt_select belt) {
    int8_t pos;
    if(belt == belt_left) {
        xQueueReceive(bcs_left_end_queue,&pos,portMAX_DELAY);
    } else if(belt == belt_right){
        xQueueReceive(bcs_right_end_queue,&pos,portMAX_DELAY);
    }
    return pos;
}

void bcs_task(void *pv_data)
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
    bool moveLeft = true; //whether the dispatcher should move left or right
    uint8_t midStartWithoutMutexCount = 0; //The number of times we started the mid band without awaiting the mutex


    while(true) {
        //----- Step 0: Reset band
        bcs_send_msg(&msg_cmd_reset,belt);
        display_log(DISPLAY_NEWLINE,"reset band");

        //----- Step 1: Wait on a block (take semaphore), before we start the band ------------------
        switch(belt) {
        case belt_left:
            xSemaphoreTake(bcs_left_start_semaphore,portMAX_DELAY);
            break;
        case belt_right:
            xSemaphoreTake(bcs_right_start_semaphore,portMAX_DELAY);
            break;
        case belt_mid:
            display_log(DISPLAY_NEWLINE,"Reset dispatcher");
            bcs_send_msg(&msg_cmd_disp_initial_pos,0);

            if(midStartWithoutMutexCount < MAX_BLOCK_COUNT) { //we are in the init phase
                midStartWithoutMutexCount++;
                xSemaphoreTake(bcs_mid_start_semaphore,2000); //try to aquire mutex anyway, in case it was already there
                //if mutex could not be taken => go on (since we're in the init phase)
            } else {
                xSemaphoreTake(bcs_mid_start_semaphore,portMAX_DELAY);
            }
            
            break;

        }

        //----- Step 2: Start the band, and wait till we detect the block
        bcs_send_msg(&msg_cmd_start,belt);
        bcs_send_msg(&msg_cmd_stoppos,belt);
        display_log(DISPLAY_NEWLINE,"start band");

        CARME_CAN_MESSAGE tmp_message;
        status_t* status = bcs_await_block(belt,ucan_queue,&tmp_message);
        if(status==NULL){ //timeout
            while(true);
            //TODO: Listen on button press
        }

        //----- Step 3 (only mid band): Move the dispatcher so we don't interfere with the coming block
        if(belt== belt_mid) {
            display_log(DISPLAY_NEWLINE,"Making dispatcher ready for moving %s",moveLeft ? "left" : "right");
            bcs_send_msg(moveLeft ? &msg_cmd_disp_start_left : &msg_cmd_disp_start_right,0);
        }
        vTaskDelay(2000); //let block move to the end of the band


        //----- Step 4: Mark the block ready for further processing (give semaphore) and optionally move the dispatcher (only mid band).
        switch(belt) {
        case belt_left:
            xQueueSend(bcs_left_end_queue,&(status->location),portMAX_DELAY);
            break;
        case belt_right:
            xQueueSend(bcs_right_end_queue,&(status->location),portMAX_DELAY);
            break;
        case belt_mid:
            bcs_prepare_drop(moveLeft ? belt_left : belt_right);

            display_log(DISPLAY_NEWLINE,"Dispatcher moves %s",moveLeft ? "left" : "right");
            bcs_send_msg(moveLeft ? &msg_cmd_disp_move_left : &msg_cmd_disp_move_right,0);
            vTaskDelay(1000); //let dispatcher move block away

            bcs_signal_dropped(moveLeft ? belt_left : belt_right);
            moveLeft = ! moveLeft;
            break;


        }

        //---- Step 5: Tell the band that we're finished
        bcs_send_msg(&msg_cmd_done,belt);
        if(belt == belt_mid) {
            bcs_signal_band_free(belt_mid);
        }
    }
}

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
