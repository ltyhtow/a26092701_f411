#ifndef TEST_FAULT_STM32F4XX_H
#define TEST_FAULT_STM32F4XX_H
#include <stdint.h>
typedef struct {
    uint32_t CFSR, HFSR, MMFAR, BFAR, CPACR, VTOR, ICSR, SHCSR;
} SCB_Type;
extern SCB_Type test_fault_scb;
#define SCB (&test_fault_scb)
void test_fault_barrier(void);
void test_fault_halt(void);
#define __DSB() test_fault_barrier()
#define __NOP() test_fault_halt()
#endif
