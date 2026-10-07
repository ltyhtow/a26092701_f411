#include "FreeRTOS.h"
#include "task.h"
#include "board_status.h"
void app_fatal_error(void) { board_fault_halt(); }
void vApplicationMallocFailedHook(void) { board_fault_halt(); }
void vApplicationStackOverflowHook(TaskHandle_t task, char *name) {
    (void)task; (void)name; board_fault_halt();
}
