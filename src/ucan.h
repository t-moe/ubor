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
bool ucan_init(void);
bool ucan_send_data(uint8_t rx_id, uint8_t data);
bool ucan_recieve_data(void);

#endif // UCAN_H
