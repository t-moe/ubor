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

/* CAN status request message id's for the conveyor belt system*/
#define CAN_ID_CONVL_STAT_REQ 0x110 // request status left conveyor belt 
#define CAN_ID_CONVM_STAT_REQ 0x120 // request status middle conveyor belt
#define CAN_ID_CONVR_STAT_REQ 0x130 // request status right conveyor belt

/* CAN status answer message id's for the conveyor belt system*/
#define CAN_ID_CONVL_STAT_ANSW 0x111 // status answer left conveyor belt
#define CAN_ID_CONVM_STAT_ANSW 0x121 // status answer middle conveyor belt
#define CAN_ID_CONVR_STAT_ANSW 0x131 // status answer right conveyor belt

/* CAN command message id's for the conveyor belt system*/
#define CAN_ID_CONVL_CMD 0x112 // send command to left conveyor belt
#define CAN_ID_CONVM_CMD 0x122 // send command to middle conveyor belt
#define CAN_ID_CONVR_CMD 0x132 // send command to right conveyor belt

/* CAN command message answer id's for the conveyor belt system*/
#define CAN_ID_CONVL_CMD_ANSW 0x113 // command answer left conveyor belt
#define CAN_ID_CONVM_CMD_ANSW 0x123 // command answer middle conveyor belt
#define CAN_ID_CONVR_CMD_ANSW 0x133 // command answer right conveyor belt

/* CAN reset message id's for the conveyor belt system*/
#define CAN_ID_CONVL_RES 0x11F // reset left conveyor belt
#define CAN_ID_CONVM_RES 0x12F // reset middle conveyor belt
#define CAN_ID_CONVR_RES 0x13F // reset right conveyor belt

/* CAN message id's for the dispatcher */
#define CAN_ID_DISP_STAT_REQ 0x140 // request status dispatcher
#define CAN_ID_DISP_STAT_ANSW 0x141 // status answer dispatcher
#define CAN_ID_DISP_CMD 0x142 // send command to dispatcher
#define CAN_ID_DISP_CMD_ANSW 0x143 // command answer dispatcher
#define CAN_ID_DISP_RES 0x14F // reset dispatcher

/* CAN status request message id's for the arms */
#define CAN_ID_ARML_STAT_REQ 0x150 // request status left arm
#define CAN_ID_ARMR_STAT_REQ 0x160 // request status right arm

/* Can status answer message id's for the arms */
#define CAN_ID_ARML_STAT_ANSW 0x151 // answer status left arm
#define CAN_ID_ARMR_STAT_ANSW 0x161 // answer status right arm

/* CAN command message id's for the arms */
#define CAN_ID_ARML_CMD 0x152 // send command to left arm
#define CAN_ID_ARMR_CMD 0x162 // send command to right arm

/* CAN command message answer id's for the arms */
#define CAN_ID_ARML_CMD_ANSW 0x153 // command answer left arm
#define CAN_ID_ARMR_CMD_ANSW 0x163 // command answer right arm

/* CAN resed command id's for the arms */
#define CAN_ID_ARML_RES 0x15F // reset left arm
#define CAN_ID_ARMR_RES 0x16F // reset right arm

/* ----- Globals ------------------------------------------------------------*/
CARME_CAN_MESSAGE rx_msg;
CARME_CAN_MESSAGE tx_msg;

/* ----- Functins -----------------------------------------------------------*/

/* setup acceptance filter */
static void ucan_setup_acceptance_filter(void)
{
    CARME_CAN_ACCEPTANCE_FILTER af;

    /* set the SJA1000 chip in running mode */
    CARME_CAN_SetMode(CARME_CAN_DF_RESET);

    af.afm = MODE_SINGLE;   //Single filter */

    /* Unmask the important bits by setting them to Zero */
    af.amr[0] = 0x00;       // unmask bit 0 - 7
    af.amr[1] = 0x1f;       // unmask bit 8 - 6
    af.amr[2] = 0xff;       // don't care in Mode with normal id length
    af.amr[3] = 0xff;       // don't care in Mode with normal id length

    /* Set the bits which have to be high */
    af.acr[0] = 0xAA;       // 11001100
    af.acr[1] = 0xA0;       // 11000000
    af.acr[2] = 0x00;       // don't care in Mode with normal id length
    af.acr[3] = 0x00;       // don't care in Mode with normal id length

    /* Set the AF */
    CARME_CAN_SetAcceptaceFilter(&af);

    /* Set the SJA1000 chip in running mode */
    CARME_CAN_SetMode(CARME_CAN_DF_NORMAL);
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
    ucan_setup_acceptance_filter();

    /* Clear the rx CAN message */
    for(int i = 0; i < 7; i++) {
        rx_msg.data[i] = 0;
    }

    /* Clear the tx CAN message */
    for(int i = 0; i < 7; i++) {
        tx_msg.data[i] = 0;
    }

    return true;
}

/* Send data via can */
bool ucan_send_data(uint8_t n_data_bytes, uint8_t msg_id, const uint8_t *data)
{
    /* Setup basic CAN message header */
    tx_msg.id = msg_id; // Message ID
    tx_msg.rtr = 0; // Something weird
    tx_msg.ext = 0; // Something weird
    tx_msg.dlc = n_data_bytes; // Send n bytes

    memcpy(tx_msg.data, data, min(n_data_bytes, 8)); // copy databytes to output buffer but only 8bytes

    display_log(DISPLAY_NEWLINE, "Test: "); // Log message to display
    CARME_CAN_Write(&tx_msg); // Send message to CAN BUS

}

bool ucan_receive_data(void)
{
    if (CARME_CAN_Read(&rx_msg) == CARME_NO_ERROR) {
        // Todo: Choose message queue
    }
}
