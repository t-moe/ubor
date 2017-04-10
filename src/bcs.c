#include "display.h"
#include <FreeRTOS.h>
#include <stdio.h>
#include <task.h>
#include <queue.h>
#include <stdbool.h>
#include <string.h>
#include "ucan.h"


// -------------------- Configuration  ------------
#define STACKSIZE_TASK        ( 256 )
#define PRIORITY_TASK         ( 2 ) //  low priority number denotes low priority task


// ------------------ Implementation ------------------------


typedef struct {
    uint16_t subid;
    uint8_t length;
    uint8_t data[];
} message_t;


typedef struct __attribute__((__packed__)) {
    uint8_t error;
    uint8_t engine;
    uint8_t lightbarrier;
    uint8_t detection;
    uint16_t position;
    int8_t location;
} status_t;

static message_t msg_status_request = {0,1,{1}};
static uint16_t msg_status_response_id = 1;
static message_t msg_cmd_start = {2,3,{1,0,0}};
static message_t msg_cmd_stop = {2,3,{2,0,0}};
static message_t msg_cmd_stoppos = {2,3,{3,0x00,0xB6}};
static message_t msg_cmd_done= {2,3,{4,0,0}};
static message_t msg_cmd_reset= {0xF,0};

static const uint16_t bcs_left= 0x110;
static const uint16_t bcs_mid = 0x120;
static const uint16_t bcs_right = 0x130;



static message_t msg_cmd_disp_initial_pos = {0x142, 3, {1, 0, 100}};
static message_t msg_cmd_disp_start_right = {0x142, 3, {1, 0xE4, 100}};
static message_t msg_cmd_disp_move_right = {0x142, 3, {1, 0x32, 50}};
static message_t msg_cmd_disp_start_left = {0x142, 3, {1, 0x1C, 100}};
static message_t msg_cmd_disp_move_left = {0x142, 3, {1, 0xCE, 50}};


static QueueHandle_t uart_queue;


void bcs_send_msg(const message_t* msg, uint16_t baseaddr)
{
    ucan_send_data(msg->length,baseaddr + msg->subid, msg->data);
    vTaskDelay(5);
}



void bcs_mid_task()
{
    bool moveLeft = true;
    while(true) {
        bcs_send_msg(&msg_cmd_reset,bcs_mid);
        bcs_send_msg(&msg_cmd_disp_initial_pos,0);
        display_log(DISPLAY_NEWLINE,"reset mid band and dispatcher");

        bcs_send_msg(&msg_cmd_start,bcs_mid);
        bcs_send_msg(&msg_cmd_stoppos,bcs_mid);
        display_log(DISPLAY_NEWLINE,"start mid band");


        uint8_t statR = display_log(DISPLAY_NEWLINE,"Waiting on block...");
        uint16_t wait_count = 0;
        status_t* status;
        CARME_CAN_MESSAGE tmp_message;

        while(true) { //Wait until the block is fully detected

            //Request status
            bcs_send_msg(&msg_status_request,bcs_mid);

            //Wait on status response
            do {
                if(xQueueReceive(uart_queue,&tmp_message,4000)==pdFALSE) {
                    wait_count++;
                    display_log(statR,"Waiting on block (%u): timeout", wait_count);
                }
            } while(tmp_message.id != bcs_mid+msg_status_response_id); //repeat until correct response message arrives

            status = (status_t*)&(tmp_message.data);
            wait_count++;

            if(status->detection == 3) { //Block detected
               display_log(statR,"Waiting on block. Found! position %04x location %d", status->position, status->location );
                break;
            }

            display_log(statR,"Waiting on block (%u): detection: %u pos: %04x",wait_count,status->detection, status->position);

            if(wait_count >= 100) { //Timout
                bcs_send_msg(&msg_cmd_done,bcs_mid);
                display_log(statR,"Waiting on block (%u): Aborted",wait_count);
                while(true);
                //TODO: Listen on button press
            }
        }


        display_log(DISPLAY_NEWLINE,"Making dispatcher ready for moving %s",moveLeft ? "left" : "right");
        bcs_send_msg(moveLeft ? &msg_cmd_disp_start_left : &msg_cmd_disp_start_right ,0);
        vTaskDelay(2000); //let block move into position where dispatcher can move it


        display_log(DISPLAY_NEWLINE,"Dispatcher moves %s",moveLeft ? "left" : "right");
        bcs_send_msg(&msg_cmd_done,bcs_mid);
        bcs_send_msg(moveLeft ? &msg_cmd_disp_move_left : &msg_cmd_disp_move_right,0);
        moveLeft = ! moveLeft;
        vTaskDelay(2000); //let dispatcher move block away

    }
}


void bcs_init()
{
    xTaskCreate(bcs_mid_task,
                "Mid",
                STACKSIZE_TASK,
                NULL,
                PRIORITY_TASK,
                NULL);

     uart_queue =  xQueueCreate(1,sizeof(CARME_CAN_MESSAGE));
     ucan_link_message_to_queue_mask(0xFF0,bcs_mid,uart_queue);

}



