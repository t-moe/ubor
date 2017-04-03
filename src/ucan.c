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
#define CAN_ID_STAT_REQL 0x110 // request status left conveyor belt 
#define CAN_ID_STAT_REQM 0x120 // request status middle conveyor belt
#define CAN_ID_STAT_REQR 0x130 // request status right conveyor belt

#define CAN_ID_STAT_ANSWL 0x111 // status answer left conveyor belt
#define CAN_ID_STAT_ANSWM 0x121 // status answer middle conveyor belt
#define CAN_ID_STAT_ANSWR 0x131 // status answer right conveyor belt

#define CAN_ID_CMDL 0x112 // send command to left conveyor belt
#define CAN_ID_CMDM 0x122 // send command to middle conveyor belt
#define CAN_ID_CMDR 0x132 // send command to right conveyor belt

#define CAN_ID_CMD_ANSWL 0x113 // command answer left conveyor belt
#define CAN_ID_CMD_ANSWM 0x123 // command answer middle conveyor belt
#define CAN_ID_CMD_ANSWR 0x133 // command answer right conveyor belt

#define CAN_ID_RESL 0x11F // reset left conveyor belt
#define CAN_ID_RESM 0x12F // reset middle conveyor belt
#define CAN_ID_RESR 0x13F // reset right conveyor belt

/* ----- Globals ------------------------------------------------------------*/
CARME_CAN_MESSAGE rx_msg;
CARME_CAN_MESSAGE tx_msg;

/* ----- Prototypes ---------------------------------------------------------*/
static void setup_acceptance_filter();
static void recieve_data();
bool init_can(void);


/* setup acceptance filter */
static void setup_acceptance_filter()
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
bool init_can(void)
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
    setup_acceptance_filter();

    /* Setup acceptance filter */
    setup_acceptance_filter();

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
bool send_data(uint8_t rx_id, uint8_t data)
{
    /* Setup basic CAN message header */
    tx_msg.id = TX_ID; // Your ID
    tx_msg.rtr = 0; // Something weird
    tx_msg.ext = 0; // Something weird
    tx_msg.dlc = 1; // Send 1byte
}

bool recieve_data()
{
}
