#include "board_status.h"
#include "main.h"
#include "motor_driver.h"
void board_status_set(bool on) {
    HAL_GPIO_WritePin(SYS_LED_GPIO_Port, SYS_LED_Pin, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}
void board_status_toggle(void) { HAL_GPIO_TogglePin(SYS_LED_GPIO_Port, SYS_LED_Pin); }
_Noreturn void board_fault_halt(void) {
    motor_driver_emergency_stop();
    __disable_irq();
    for (;;) {
        board_status_toggle();
        for (volatile uint32_t i = 0; i < 100000; ++i) {}
    }
}
