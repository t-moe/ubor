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
static message_t msg_cmd_stoppos = {2,3,{3,0x00,0x96}};
static message_t msg_cmd_done= {2,3,{4,0,0}};
static message_t msg_cmd_reset= {0xF,0};

static const uint16_t bcs_left= 0x110;
static const uint16_t bcs_mid = 0x120;
static const uint16_t bcs_right = 0x130;

static QueueHandle_t uart_queue;


void bcs_send_msg(const message_t* msg, uint16_t baseaddr)
{
    ucan_send_data(msg->length,baseaddr + msg->subid, msg->data);
    vTaskDelay(5);
}



void bcs_task()
{
    while(true) {
        bcs_send_msg(&msg_cmd_reset,bcs_mid);
        display_log(DISPLAY_NEWLINE,"reset");
        //vTaskDelay(2000);

        bcs_send_msg(&msg_cmd_start,bcs_mid);
        bcs_send_msg(&msg_cmd_stoppos,bcs_mid);

        display_log(DISPLAY_NEWLINE,"start");

       // vTaskDelay(2000);

        uint8_t statR = display_log(DISPLAY_NEWLINE,"Waiting on block...");
        uint16_t wait_count = 0;
        status_t* status;
        CARME_CAN_MESSAGE tmp_message;
        while(true) {


            bcs_send_msg(&msg_status_request,bcs_mid);

            do {
                if(xQueueReceive(uart_queue,&tmp_message,4000)==pdFALSE) {
                    wait_count++;
                    display_log(statR,"Waiting on block (%u): timeout", wait_count);
                }
            } while(tmp_message.id != bcs_mid+msg_status_response_id);

            status = (status_t*)&(tmp_message.data);
            wait_count++;
            display_log(statR,"Waiting on block (%u): detection: %u pos: %04x",wait_count,status->detection, status->position);
            if(status->detection == 3) {
                break;
            }
            if(wait_count >= 100) {
                bcs_send_msg(&msg_cmd_done,bcs_mid);
                display_log(statR,"Waiting on block (%u): Aborted",wait_count);
                while(true);
            }

        }
        display_log(DISPLAY_NEWLINE,"position %04x location %d", status->position, status->location );



       // bcs_send_msg(&msg_cmd_stop,bcs_mid);
        //display_log(DISPLAY_NEWLINE,"stop");
        vTaskDelay(4000);

         bcs_send_msg(&msg_cmd_done,bcs_mid);

    }

}


void bcs_init()
{
    xTaskCreate(bcs_task,
                "Mid",
                STACKSIZE_TASK,
                NULL,
                PRIORITY_TASK,
                NULL);

     uart_queue =  xQueueCreate(1,sizeof(CARME_CAN_MESSAGE));
     ucan_link_message_to_queue_mask(0xFF0,bcs_mid,uart_queue);

}



