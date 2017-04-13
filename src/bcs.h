
#include <semphr.h>

extern SemaphoreHandle_t bcs_left_end_semaphore;
extern SemaphoreHandle_t bcs_right_end_semaphore;
extern SemaphoreHandle_t bcs_mid_start_semaphore;

void bcs_init();
