#ifndef UCAN_H
#define UCAN_H

/*----- Header-Files ---------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include <carme.h>
#include <can.h>
#include <stm32f4xx.h>


/*----- Macros ---------------------------------------------------------------*/

/*----- Data types -----------------------------------------------------------*/

/*----- Function prototypes --------------------------------------------------*/
bool init_can(void);
bool send_data(uint8_t rx_id, uint8_t data);
bool recieve_data(void);

#endif // UCAN_H
