#ifndef BOARD_STATUS_H
#define BOARD_STATUS_H
#include <stdbool.h>
void board_status_set(bool on);
void board_status_toggle(void);
_Noreturn void board_fault_halt(void);
#endif
