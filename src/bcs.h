#ifndef BCS_H
#define BCS_H
#include <stdint.h>

//Enum to destinguish the different tasks. The members point to the base address of the can endpoints
enum belt_select {belt_left=0x110,
                  belt_mid=0x120,
                  belt_right=0x130};

int8_t bcs_grab(enum belt_select belt);
void bcs_prepare_drop(enum belt_select belt);
void bcs_signal_dropped(enum belt_select belt);
void bcs_signal_band_free(enum belt_select belt);
void bcs_init();

#endif /* BCS_H */
