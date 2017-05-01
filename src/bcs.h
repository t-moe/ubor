#ifndef BCS_H
#define BCS_H
#include <stdint.h>

/**
 * @brief The belt_select enum differenciates between the different belt tasks. The members point to the base address of the can endpoints
 */
enum belt_select {belt_left=0x110, //!< the left belt
                  belt_mid=0x120, //!< the middle belt
                  belt_right=0x130//!< the right belt
                 };


//doc see bcs.c
int8_t bcs_grab(enum belt_select belt);
void bcs_prepare_drop(enum belt_select belt);
void bcs_signal_dropped(enum belt_select belt);
void bcs_signal_band_free(enum belt_select belt);
void bcs_init();

#endif /* BCS_H */
