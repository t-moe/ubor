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
#include <lcd.h>
#include <stdbool.h>
#include <string.h>
#include <semphr.h>


// -------------------- Configuration  ------------
#define STACKSIZE_TASK        ( 256 ) //!< Stack size of the display task
#define PRIORITY_TASK         ( 3 ) //!< Priority of the Display task  (low priority number denotes low priority task)

#define QUEUE_SIZE 10 //!< Size of the message queue

#define DISPLAY_LINES   30 //!< Number of lines that fit on the display (vertically)
#define DISPLAY_CHARS   64 //!< Number of horizontal characters that can be displayed


// ------------------ Implementation ------------------------

/**
  @brief Structure to describe a log message
  */
typedef struct {
    const char* taskname; //!< Name of the task that wrote the message
    uint8_t id; //!< Id that was assigned to the task
    char message[DISPLAY_CHARS+1]; //!< Message (formatted)
} log_message_t;

static uint8_t last_given_id = 0; //!< last given message id
static QueueHandle_t display_queue; //!< Queue to send messages to the display task
static SemaphoreHandle_t display_id_mutex; //!< Mutex so ensure atomic operations on last_given_id

/**
 * @brief       Logs a message to the display
 * @type        global
 * @param[in]   uint8_t  id  Message-ID to overwrite. Pass \ref DISPLAY_NEWLINE to create a new message
 * @param[in]   const char*  fmtstr  Printf formatstring
 * @param[in]   ...    Arguments for format specification
 * @return      uint8_t Id of the message that was printed
 **/
uint8_t display_log(uint8_t id, const char *fmtstr, ...)
{
    log_message_t msg; //Buffer for message. must be on stack (multi-task env)

    //Generate string from formatstring + arguments
    va_list args;
    va_start (args, fmtstr);
    vsprintf(msg.message,fmtstr, args);
    va_end(args);

    msg.taskname = pcTaskGetName(xTaskGetCurrentTaskHandle());


    //Assign a new id to the message
    uint8_t newId;

    xSemaphoreTake(display_id_mutex,portMAX_DELAY);

    if(id == DISPLAY_NEWLINE) {
        newId = ++last_given_id;
        if(last_given_id == 0xFF) {
            last_given_id = 0;
        }
    } else {
        newId = id;
    }
    xSemaphoreGive(display_id_mutex);


    msg.id = newId;

    //Send message to display task
    xQueueSend(display_queue, &msg, portMAX_DELAY );

    return newId;

}


/**
 * @brief       Prints a single message on the display
 * @type        static
 * @param[in]   uint8_t  line  line to print the message on
 * @param[in]   message_t*  msg  message to print
 * @return      bool
 **/
static void display_print_message(uint8_t line, log_message_t* msg)
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

    while(x<DISPLAY_CHARS) {
        LCD_DisplayCharLine(line,x++,' ');
    }
}


uint8_t visible_messages=0; //!< Number of currently visible messages
uint8_t buffer_offset = 0;  //!< Offset in the message_buffer to get to the top message
log_message_t message_buffer[DISPLAY_LINES]; //!< buffer of all visible messages (ring buffer!)


/**
 * @brief       DisplayTask that takes messages out of the queue and writes them to the display
 * @type        static
 * @return      none
 **/
static void display_task()
{

    while(true) {
        uint8_t top_id = message_buffer[buffer_offset].id;
        uint8_t bottom_id  = message_buffer[(buffer_offset + visible_messages -1 )%DISPLAY_LINES].id;
        static log_message_t tmp_message;
        xQueueReceive(display_queue,&tmp_message,portMAX_DELAY);
        uint8_t new_id = tmp_message.id;

        //Check if message has the same id as a message that is currently beeing displayed
        if((top_id <= bottom_id && new_id >= top_id && new_id <= bottom_id) ||
                (top_id > bottom_id && new_id >= top_id)) {
            //Replace message
            uint8_t replace_buffer_index =(buffer_offset + (new_id-top_id)) % DISPLAY_LINES;
            memcpy(&message_buffer[replace_buffer_index],&tmp_message,sizeof(log_message_t));
            display_print_message(new_id-top_id, &message_buffer[replace_buffer_index]);
        } else if(top_id > bottom_id && new_id <= bottom_id ) {
            //Replace message (and handle index wraping)
            uint8_t replace_buffer_index =(buffer_offset + visible_messages -1 - (bottom_id-new_id)) % DISPLAY_LINES;
            memcpy(&message_buffer[replace_buffer_index],&tmp_message,sizeof(log_message_t));
            display_print_message(visible_messages -1 - (bottom_id-new_id), &message_buffer[replace_buffer_index]);
        } else { //message is new
            if(visible_messages == DISPLAY_LINES) {
                memcpy(&message_buffer[buffer_offset],&tmp_message,sizeof(log_message_t));
                buffer_offset = (buffer_offset +1) % DISPLAY_LINES;
                for(uint8_t i =0; i< DISPLAY_LINES; i++) {
                    uint8_t buffer_index = (buffer_offset + i) % DISPLAY_LINES;
                    display_print_message(i,&message_buffer[buffer_index]);
                }
            } else { // Display not full yet
                uint8_t buffer_index = (buffer_offset + visible_messages) % DISPLAY_LINES;
                memcpy(&message_buffer[buffer_index],&tmp_message,sizeof(log_message_t));
                display_print_message(visible_messages,&message_buffer[buffer_index]);
                visible_messages++;
            }
        }
    }
}



/**
 * @brief      Initializes the display and starts the display task
 * @type       global
 * @return     none
 **/
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

    display_queue =  xQueueCreate(QUEUE_SIZE,sizeof(log_message_t));
    display_id_mutex = xSemaphoreCreateMutex();

}

