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
//----- Header-Files ---------------------------------------------------------
#include <stdio.h>
#include <FreeRTOS.h>
#include <carme.h>
#include <carme_io1.h>
#include <carme_io2.h>
#include <task.h>

#include "ucan.h"
#include "display.h"
#include "bcs.h"
#include "arm.h"

/** 
 * @brief      Main function which calls the scheduler
 * @return     none
 **/
int  main(void)
{
    /* initialize the IO libs */
    CARME_IO1_Init();
    CARME_IO2_Init();

    /* Call init functions */
    ucan_init();
    display_init();
    bcs_init();
    init_arm();

    /* Start scheduler */
    vTaskStartScheduler();

    for (;;) {
        // All work and no play makes jack a dull boy
    }

    return 0;
}
