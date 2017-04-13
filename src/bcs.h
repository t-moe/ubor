
#include <semphr.h>

extern SemaphoreHandle_t bcs_mid_start_semaphore;
extern QueueHandle_t bcs_left_end_queue;
extern QueueHandle_t bcs_right_end_queue;

void bcs_init();
