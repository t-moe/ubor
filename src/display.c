#include "display.h"
#include <FreeRTOS.h>
#include <stdio.h>
#include <task.h>
#include <queue.h>
#include <lcd.h>
#include <stdbool.h>


// -------------------- Configuration  ------------
#define STACKSIZE_TASK        ( 256 )
#define PRIORITY_TASK         ( 2 ) //  low priority number denotes low priority task

#define QUEUE_SIZE 10

#define DISPLAY_LINES   30


// ------------------ Implementation ------------------------

typedef struct {
    const char* taskname;
    uint8_t id;
    char message[30];
} message_t;

static uint8_t last_given_id = 0;
static QueueHandle_t display_queue;


uint8_t display_log(uint8_t id, const char *fmtstr, ...)
{
    message_t msg; //Buffer for message. must be on stack (multi-task env)

    //Generate string from formatstring + arguments
    va_list args;
    va_start (args, fmtstr);
    vsprintf(msg.message,fmtstr, args);
    va_end(args);

    msg.taskname = pcTaskGetName(xTaskGetCurrentTaskHandle());


    //Assign a new id to the message
    uint8_t newId;

    //Todo: mutex protect
    if(id == DISPLAY_NEWLINE) {
        newId = ++last_given_id;
        if(last_given_id == 0xFF) {
            last_given_id = 0;
        }
    } else {
        newId = id;
    }
    //Todo: mutex release


    msg.id = newId;

    //Send message to display task
    xQueueSend(display_queue, &msg, portMAX_DELAY );

}


void display_print_message(uint8_t line, message_t* msg)
{
    uint8_t x=0;

    //Print task name in gray
    LCD_SetTextColor(GUI_COLOR_LIGHT_GRAY);
    const char* charPtr = msg->taskname;
    while(*charPtr!=0) {
        LCD_DisplayCharLine(line,x++,*charPtr++);
    }
    LCD_DisplayCharLine(line,x++,':');
    LCD_DisplayCharLine(line,x++,' ');

    //Print message
    LCD_SetTextColor(GUI_COLOR_WHITE);
    charPtr = msg->message;
    while(*charPtr!=0) {
        LCD_DisplayCharLine(line,x++,*charPtr++);
    }

    for(uint8_t i =0; i<10; i++) {
        LCD_DisplayCharLine(line,x++,' ');
    }
}


uint8_t visible_messages=0;
uint8_t buffer_offset = 0;
message_t message_buffer[DISPLAY_LINES];



void display_task()
{

    while(true) {
        if(visible_messages == DISPLAY_LINES) {
            // static message_t tmp_message;
            // xQueueReceive(display_queue,&tmp_message,portMAX_DELAY);
            // uint8_t top_id;

            xQueueReceive(display_queue,&message_buffer[buffer_offset],portMAX_DELAY);
            buffer_offset = (buffer_offset +1) % DISPLAY_LINES;
            for(uint8_t i =0; i< DISPLAY_LINES; i++) {
                uint8_t buffer_index = (buffer_offset + i) % DISPLAY_LINES;
                display_print_message(i,&message_buffer[buffer_index]);
            }
        } else {
            uint8_t buffer_index = (buffer_offset + visible_messages) % DISPLAY_LINES;
            xQueueReceive(display_queue,&message_buffer[buffer_index],portMAX_DELAY);
            display_print_message(visible_messages,&message_buffer[buffer_index]);
            visible_messages++;
        }
    }
}




void display_init()
{
    LCD_Init();
    LCD_Clear(GUI_COLOR_BLACK);
    xTaskCreate(display_task,
                "Display Task",
                STACKSIZE_TASK,
                NULL,
                PRIORITY_TASK,
                NULL);

    display_queue =  xQueueCreate(QUEUE_SIZE,sizeof(message_t));

}

