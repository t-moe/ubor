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


static message_t msg_status_request = {0,1,{1}};
static uint16_t msg_status_response_id = 1;
static message_t msg_cmd_start = {2,3,{1,0,0}};
static message_t msg_cmd_stop = {2,3,{2,0,0}};
static message_t msg_cmd_stoppos = {2,3,{3,0x12,0x34}};
static message_t msg_cmd_reset= {2,0xF,0};

static const uint16_t bcs_left= 0x110;
static const uint16_t bcs_mid = 0x120;
static const uint16_t bcs_right = 0x130;

static QueueHandle_t uart_queue;


void bcs_send_msg(const message_t* msg, uint16_t baseaddr)
{
    ucan_send_data(msg->length,baseaddr + msg->subid, msg->data);
}



void bcs_task()
{
    while(true) {
        bcs_send_msg(&msg_cmd_reset,bcs_mid);
        display_log(DISPLAY_NEWLINE,"reset");
        vTaskDelay(2000);

        bcs_send_msg(&msg_cmd_start,bcs_mid);
        display_log(DISPLAY_NEWLINE,"start");
        vTaskDelay(2000);


        uint8_t statR = display_log(DISPLAY_NEWLINE,"requesting state..");
        bcs_send_msg(&msg_status_request,bcs_mid);
        static CARME_CAN_MESSAGE tmp_message;
        if(xQueueReceive(uart_queue,&tmp_message,4000)==pdFALSE) {
            display_log(statR,"requesting state... fail");
        } else {
            display_log(statR,"requesting state... ok");
        }

        bcs_send_msg(&msg_cmd_stop,bcs_mid);
        display_log(DISPLAY_NEWLINE,"stop");
        vTaskDelay(2000);

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
     ucan_link_message_to_queue(bcs_mid+msg_status_response_id,uart_queue);

}



