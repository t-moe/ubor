//----- Header-Files -----------------------------------------------------------
#include <stdio.h>                      /* Standard Input/Output              */
#include <FreeRTOS.h>                   /* All freeRTOS headers               */
#include "ucan.h"
#include <carme.h>
#include <carme_io1.h>                  /* CARMEIO1 Board Support Package     */
#include <carme_io2.h>


#include "display.h"
#include <task.h>




#define STACKSIZE_TASK        ( 256 )

#define PRIORITY_TASK         ( 2 ) //  low priority number denotes low priority task


static void vAppTask(void *pvData)
{
    uint8_t i = 0;
    while(1) {
        uint8_t id =  display_log(DISPLAY_NEWLINE,"test %u",i++);
        vTaskDelay(100);
        uint8_t id2 = display_log(DISPLAY_NEWLINE,"test %u",i++);
        vTaskDelay(200);

        display_log(id,"test updated %u",i);
        vTaskDelay(100);
        display_log(id2,"test updated %u",i);
        vTaskDelay(100);
    }
}





int  main(void)
{

    CARME_IO1_Init();               // Initialize the CARMEIO1
    CARME_IO2_Init();


    display_init();

    xTaskCreate(vAppTask,
                "Taskx",
                STACKSIZE_TASK,
                NULL,
                PRIORITY_TASK,
                NULL);




    vTaskStartScheduler();

    for (;;) {
        // All work and no play makes jack a dull boy
    }

    return 0;
}
