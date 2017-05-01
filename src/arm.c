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

/**
 * @defgroup arm Arm
 * @brief Module controlling the robot arms
 */
/*@{*/

//----- Header-Files -----------------------------------------------------------
#include <stdio.h>

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include <timers.h>
#include <memPoolService.h>
#include <stdbool.h>

#include <carme_io1.h>

#include "arm.h"
#include "ucan.h"
#include "bcs.h"

//----- Macros -----------------------------------------------------------------
#define BUTTON_T0 0x01
#define BUTTON_T1 0x02
#define BUTTON_T2 0x04
#define BUTTON_T3 0x08

// Left
#define ROBOT_L_STATUS_REQUEST_ID   0x150
#define ROBOT_L_STATUS_RETURN_ID    0x151
#define ROBOT_L_COMAND_REQUEST_ID   0x152
#define ROBOT_L_COMAND_RETURN_ID    0x153
#define ROBOT_L_RESET_ID            0x15F
#define COMAND_DLC                  0x006
#define STATUS_REQEST_DLC           0x002


// Right
#define ROBOT_R_STATUS_REQUEST_ID   0x160
#define ROBOT_R_STATUS_RETURN_ID    0x161
#define ROBOT_R_COMAND_REQUEST_ID   0x162
#define ROBOT_R_COMAND_RETURN_ID    0x163
#define ROBOT_R_RESET_ID            0x16F

#define MASK_SWITCH_0               0x01
#define MASK_SWITCH_1               0x02
#define MASK_SWITCH_2               0x04
#define MASK_SWITCH_3               0x08
#define MASK_SWITCH_4               0x10


#define BLOCK_TIME_MIDDLE_POS 200000 // block time for mutex midle position
#define MSG_QUEUE_SIZE 1
#define ARM_TASK_PRIORITY 2
#define ARM_TASK_STACKSIZE 256
#define TASK_DELAY 100
#define GRIPPER_MAX 1
#define GRIPPER_MIN 0

/**
 * @brief The arm_select enum differenciates between the different arms (left/right)
 */
enum arm_select {arm_left=belt_left, //!< the left arm
                 arm_right=belt_right//!< the right arm
                };

//----- Data types -------------------------------------------------------------

//----- Function prototypes ----------------------------------------------------
static  void  wait_until_pos(uint8_t *pos, enum arm_select side);

//----- Data -------------------------------------------------------------------
static QueueHandle_t robot_left_queue;
static QueueHandle_t robot_right_queue;
static QueueHandle_t robot_manual_queue;

static SemaphoreHandle_t arm_mid_air_mutex;

CARME_CAN_MESSAGE robot_msg_buffer_right;
CARME_CAN_MESSAGE robot_msg_buffer_left;

uint8_t status_request[2] = {0x02,0x00};

//----- Implementation ---------------------------------------------------------

/**
 * @brief       Controls the airspace in the middle position. This position is
 *              protected with a mutex. Before enter the critical area this
 *              function takes the mutex.
 *
 *  @type       static
 *
 *  @param[in]  none
 *
 *  @return     none
 **/
static void arm_enter_critical_air_space()
{
    display_log(DISPLAY_NEWLINE, "take semaphore mid");
    xSemaphoreTake(arm_mid_air_mutex, portMAX_DELAY); //to protect the airspace around mid
}

/**
 * @brief       Controls the airspace in the middle position. This position is
 *              protected with a mutex. After exit the critical area this
 *              function give the mutex.
 *
 *  @type       static
 *
 *  @param[in]  none
 *
 *  @return     none
 **/
static void arm_leave_critical_air_space()
{
    display_log(DISPLAY_NEWLINE, "give semaphore mid");
    xSemaphoreGive(arm_mid_air_mutex);
}


/**
 * @brief       Task for left and right arm.
 *
 *  @type       public
 *
 *  @param[in]  *pvData chose left(0) or right(1) arm
 *
 *  @return     none
**/
void move_roboter(void *pv_data)
{

    enum arm_select left_right_sel = (enum arm_select)pv_data;
    uint8_t *pos_arm;
    int id_arm_comand_request;

    /* Init */
    if(left_right_sel == arm_right) {
        static uint8_t pos_arm_right[11][6] = {
            /*                      Arm    B     S     E     H     G  */
            /*0 Nullposition     */ {0x02, 0x00, 0x19, 0x1A, 0x21, 0x01},
            /*1 StartPosition    */ {0x02, 0x00, 0x1F, 0x1A, 0x21, 0x01},
            /*2 Zp1 FB Links     */ {0x02, 0x00, 0x1F, 0x1A, 0x21, 0x00},
            /*3 Zp2 FB Links     */ {0x02, 0x00, 0x19, 0x1A, 0x21, 0x00},
            /*4 G offen FB Links */ {0x02, 0xee, 0x00, 0x40, 0x21, 0x00},
            /*5 G zu FB Links    */ {0x02, 0xD2, 0x00, 0x2b, 0x21, 0x00},
            /*6 Zp1 FB Mitte     */ {0x02, 0xD2, 0x1c, 0x1d, 0x21, 0x00},
            /*7 Zp2 FB Mitte     */ {0x02, 0xD2, 0x1c, 0x1d, 0x21, 0x01},
            /*8 G zu FB Mitte    */ {0x02, 0xD2, 0x13, 0x1d, 0x21, 0x01},
            /*9 G offen FB Mitte */ {0x02, 0xD2, 0x00, 0x36, 0x21, 0x01},
            {0x02, 0xEE, 0x00, 0x36, 0x21, 0x01},
        };

        arm_mid_air_mutex = xSemaphoreCreateBinary();
        xSemaphoreGive(arm_mid_air_mutex);

        robot_right_queue = xQueueCreate(MSG_QUEUE_SIZE, sizeof(CARME_CAN_MESSAGE));
        ucan_link_message_to_queue(ROBOT_R_STATUS_RETURN_ID, robot_right_queue);

        pos_arm = (uint8_t *)pos_arm_right;
        id_arm_comand_request = ROBOT_R_COMAND_REQUEST_ID;

        ucan_send_data(0, ROBOT_R_RESET_ID, 0);
    }

    if(left_right_sel == arm_left) {
        static uint8_t pos_arm_left[11][6] = {
            /*                      Arm    B     S     E     H     G     x     x */
            {0x02, 0x00, 0x19, 0x1A, 0x21, 0x01}, //Warte Position ECTS Abholen
            {0x02, 0x00, 0x1F, 0x1A, 0x21, 0x01}, //ECTS Holen Mitte
            {0x02, 0x00, 0x1F, 0x1A, 0x21, 0x00}, //ECTS Greifen
            {0x02, 0x00, 0x19, 0x1A, 0x21, 0x00}, //ECTS Anheben
            {0x02, 0x12, 0x00, 0x40, 0x21, 0x00}, //Drehen bis pos 1
            {0x02, 0x2D, 0x00, 0x2b, 0x21, 0x00}, //Drehen bis pos 2 bereit zum ablegen
            {0x02, 0x2D, 0x1c, 0x1d, 0x21, 0x00}, //ECTS Ablegen
            {0x02, 0x2D, 0x1c, 0x1d, 0x21, 0x01}, //ECTS Ablegen
            {0x02, 0x2D, 0x13, 0x1d, 0x21, 0x01}, //Arm auserhalb ects
            {0x02, 0x2D, 0x00, 0x36, 0x21, 0x01}, //Arm ausserhalb mitte
            {0x02, 0x12, 0x00, 0x36, 0x21, 0x01}, //Arm ausserhalb mitte
        };

        pos_arm = (uint8_t *)pos_arm_left;
        id_arm_comand_request = ROBOT_L_COMAND_REQUEST_ID;

        robot_left_queue = xQueueCreate(MSG_QUEUE_SIZE, sizeof(CARME_CAN_MESSAGE));
        ucan_link_message_to_queue(ROBOT_L_STATUS_RETURN_ID, robot_left_queue);

        ucan_send_data(0, ROBOT_L_RESET_ID, 0 );
    }

    vTaskDelay(500); //needed for reset to be applied

    while(1) {

        for(int n = 0; n < 11; n++) {
            display_log(DISPLAY_NEWLINE, "Going to position %u",n);

            //before we want to grab the block
            if(n==1) {
                int8_t pos;
                pos = bcs_grab(left_right_sel);
                display_log(DISPLAY_NEWLINE,"block is at pos %d",pos);
            }

            if(n == 6) { //before we want to access the mid position
                arm_enter_critical_air_space();
            }

            if(n==7) { //before we open the grip (to drop the block)
                bcs_prepare_drop(belt_mid);
            }



            ucan_send_data(COMAND_DLC, id_arm_comand_request, &pos_arm[n*6] );
            wait_until_pos(&pos_arm[n*6], left_right_sel);

            if(n==3) { //after we picked up a block
                bcs_signal_band_free(left_right_sel);
            }

            if(n== 8) { //after we dropped a block
                bcs_signal_dropped(belt_mid);
            }

            if(n == 9) { //after we moved out of the mid position
                arm_leave_critical_air_space();
            }


            vTaskDelay(TASK_DELAY);
        }
    }
}

/**
 * @brief       Checks the arm position and waits until the desired position
 *              is reached.
 *
 *  @type       public
 *
 *  @param[in]  *pos: the position to reach / side: left or right arm
 *
 *  @return     none
 **/
static void wait_until_pos(uint8_t *pos, enum arm_select side)
{
    uint8_t *temp = (uint8_t *)pos;
    bool close_enough = false;

    if(side == arm_right) {
        //display_log(DISPLAY_NEWLINE, "Right arm wait until position reached");

        while(close_enough != true) {
            vTaskDelay(200);
            ucan_send_data(STATUS_REQEST_DLC, ROBOT_R_STATUS_REQUEST_ID, status_request );


            xQueueReceive(robot_right_queue, (void *)&robot_msg_buffer_right, portMAX_DELAY);
            close_enough = true;
            for(int i=1; i<6; i++) {
                if(abs(temp[i]-robot_msg_buffer_right.data[i])>0x01) {
                    close_enough = false;
                    break;
                }
            }

        }
        //display_log(DISPLAY_NEWLINE, "Right arm reached position");
    } else {
        //display_log(DISPLAY_NEWLINE, "Left arm wait until position reached");
        while(close_enough != true) {

            vTaskDelay(200);

            ucan_send_data(STATUS_REQEST_DLC, ROBOT_L_STATUS_REQUEST_ID, status_request );

            xQueueReceive(robot_left_queue, (void *)&robot_msg_buffer_left, portMAX_DELAY);
            close_enough = true;
            for(int i=1; i<6; i++) {
                if(abs(temp[i]-robot_msg_buffer_left.data[i])>0x01) {
                    close_enough = false;
                    break;
                }
            }
        }

        //display_log(DISPLAY_NEWLINE, "Left arm reached position");
    }
    vTaskDelay(1000);
}

/**
 * @brief       Creates the arm tasks.
 *
 *  @type       public
 *
 *  @param[in]  none
 *
 *  @return     none
 **/
void init_arm()
{

    xTaskCreate(move_roboter,
                "Arm Left",
                ARM_TASK_STACKSIZE,
                (void*)arm_left,
                ARM_TASK_PRIORITY,
                NULL);

    xTaskCreate(move_roboter,
                "Arm Right",
                ARM_TASK_STACKSIZE,
                (void*)arm_right,
                ARM_TASK_PRIORITY,
                NULL);
    /*
    xTaskCreate(manual_arm_movement,
                "Manual Arm Movement",
                ARM_TASK_STACKSIZE,
                NULL,
                ARM_TASK_PRIORITY,
                NULL);*/
}

/**
 * @brief       Task to move the roboter with the Buttons. To create this
 *              task unkomment it in the function init_arm()
 *
 *  @type       public
 *
 *  @param[in]  *pvData not used
 *
 *  @return     none
 **/
void manual_arm_movement(void *pvData)
{
    /*
     * Switch 0: select left or right arm
     * Switch 1: increment or decrement position value
     * Switch 2: gripper open or close
     *
     * Button 0: change value of basis
     * Button 1: change value of shoulder
     * Button 2: change value of elbow
     * Button 3: change value of hand
     */

    uint8_t button_data;
    uint8_t switch_data;

    uint8_t pos_manuel[6] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x00};
    bool increment = false;
    bool left_select = false;

    CARME_CAN_MESSAGE robot_msg_buffer_manual;
    robot_manual_queue = xQueueCreate(MSG_QUEUE_SIZE, sizeof(CARME_CAN_MESSAGE));
    ucan_link_message_to_queue(ROBOT_R_STATUS_RETURN_ID, robot_manual_queue);
    ucan_link_message_to_queue(ROBOT_L_STATUS_RETURN_ID, robot_manual_queue);

    while(1) {
        CARME_IO1_BUTTON_Get(&button_data);
        CARME_IO1_SWITCH_Get(&switch_data);

        if( (switch_data & MASK_SWITCH_0) != 0) {
            left_select = true;
        } else {
            left_select = false;
        }

        if( (switch_data & MASK_SWITCH_1 ) != 0) {
            increment = true;
        } else {
            increment = false;
        }

        if( (switch_data & MASK_SWITCH_2) != 0) {
            pos_manuel[5] = GRIPPER_MAX;
        } else {
            pos_manuel[5] = GRIPPER_MIN;
        }

        switch(button_data) {

        case BUTTON_T0:
            if(increment == true) {
                pos_manuel[1]++;
            } else {
                pos_manuel[1]--;
            }
            break;

        case BUTTON_T1:
            if(increment == true) {
                pos_manuel[2]++;;
            } else {
                pos_manuel[2]--;
            }
            break;

        case BUTTON_T2:
            if(increment == true) {
                pos_manuel[3]++;
            } else {
                pos_manuel[3]--;
            }
            break;

        case BUTTON_T3:
            if(increment == true) {
                pos_manuel[4]++;
            } else {
                pos_manuel[4]--;
            }
            break;
        default:
            break;
        }

        if(left_select == true) {
            ucan_send_data(COMAND_DLC, ROBOT_L_COMAND_REQUEST_ID, pos_manuel);
            vTaskDelay(20);
            ucan_send_data(STATUS_REQEST_DLC, ROBOT_L_STATUS_REQUEST_ID, status_request );
            vTaskDelay(20);
            xQueueReceive(robot_manual_queue, (void *)&robot_msg_buffer_manual, portMAX_DELAY);
            vTaskDelay(20);
        } else {
            ucan_send_data(COMAND_DLC, ROBOT_R_COMAND_REQUEST_ID, pos_manuel);
            vTaskDelay(20);
            ucan_send_data(STATUS_REQEST_DLC, ROBOT_R_STATUS_REQUEST_ID, status_request );
            xQueueReceive(robot_manual_queue, (void *)&robot_msg_buffer_manual, portMAX_DELAY);
            vTaskDelay(20);
        }

        display_log(DISPLAY_NEWLINE,"Position: %x %x %x %x %x %x",robot_msg_buffer_right.data[0],
                    robot_msg_buffer_manual.data[1],
                    robot_msg_buffer_manual.data[2],
                    robot_msg_buffer_manual.data[3],
                    robot_msg_buffer_manual.data[4],
                    robot_msg_buffer_manual.data[5]);
    }
}

/*@}*/

