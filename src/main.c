//----- Header-Files -----------------------------------------------------------
#include <stdio.h>                      /* Standard Input/Output              */
#include <FreeRTOS.h>                   /* All freeRTOS headers               */
#include <carme.h>
#include <carme_io1.h>                  /* CARMEIO1 Board Support Package     */
#include <carme_io2.h>
#include <task.h>


#include "ucan.h"
#include "display.h"
#include "bcs.h"
#include "arm.h"

int  main(void)
{
    CARME_IO1_Init();               // Initialize the CARMEIO1
    CARME_IO2_Init();

    ucan_init();
    display_init();
    bcs_init();
    init_arm();

    vTaskStartScheduler();

    for (;;) {
        // All work and no play makes jack a dull boy
    }

    return 0;
}
