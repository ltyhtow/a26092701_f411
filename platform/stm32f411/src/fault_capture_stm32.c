#include "fault_capture.h"
#include "motor_driver.h"
#include "stm32f4xx.h"
#include <stddef.h>

/* Keep this translation unit integer-only (-mgeneral-regs-only on ARM). It must
 * not trigger lazy FP stacking while it is inspecting the original exception. */
volatile fault_snapshot_t g_fault_snapshot;
volatile uint32_t g_fault_capture_guard;
__attribute__((aligned(8), used)) uint32_t g_fault_capture_stack[FAULT_CAPTURE_STACK_BYTES / sizeof(uint32_t)];
__attribute__((aligned(8), used)) uint32_t g_fault_nested_stack[FAULT_CAPTURE_STACK_BYTES / sizeof(uint32_t)];

static bool valid_exc_return(uint32_t value) {
    /* ARMv7-M FPU: handler/MSP, thread/MSP, thread/PSP, with basic or extended frame. */
    return value == 0xFFFFFFF1UL || value == 0xFFFFFFF9UL || value == 0xFFFFFFFDUL ||
           value == 0xFFFFFFE1UL || value == 0xFFFFFFE9UL || value == 0xFFFFFFEDUL;
}

bool fault_capture_decode_frame(volatile fault_snapshot_t *snapshot,
                                uintptr_t frame_address, uint32_t exc_return,
                                uint32_t cfsr, uintptr_t sram_begin, uintptr_t sram_end) {
    if (snapshot == NULL) return false;
    snapshot->frame_address = (uint32_t)frame_address;
    snapshot->frame_valid = 0U;
    snapshot->frame_extended = (exc_return & (1UL << 4U)) == 0U ? 1U : 0U;
    snapshot->stacked_r0 = snapshot->stacked_r1 = snapshot->stacked_r2 = snapshot->stacked_r3 = 0U;
    snapshot->stacked_r12 = snapshot->stacked_lr = snapshot->stacked_pc = snapshot->stacked_xpsr = 0U;

    /* ARM DUI0553A, Figure 2-3 (section 2.3.7, page 2-27): R0 is at the
     * hardware SP in BOTH layouts. The 18 FP words FOLLOW xPSR at higher
     * addresses; advancing the SP by 18 would incorrectly report FP data as PC.
     * https://documentation-service.arm.com/static/5f2ac4ab60a93e65927bbdbf */
    const uintptr_t fp_bytes = snapshot->frame_extended ? 18U * sizeof(uint32_t) : 0U;
    const uintptr_t required_bytes = fp_bytes + 8U * sizeof(uint32_t);
    /* Subtraction only after ordered bounds checks prevents pointer arithmetic
     * wrap. Never inspect xPSR/PC or any stacked word before these checks. */
    if (!valid_exc_return(exc_return) || (cfsr & FAULT_CAPTURE_STACK_ERROR_MASK) != 0U ||
        (frame_address & 3U) != 0U || sram_begin > sram_end ||
        frame_address < sram_begin || frame_address > sram_end ||
        sram_end - frame_address < required_bytes) return false;

    const volatile uint32_t *core = (const volatile uint32_t *)frame_address;
    snapshot->stacked_r0 = core[0];
    snapshot->stacked_r1 = core[1];
    snapshot->stacked_r2 = core[2];
    snapshot->stacked_r3 = core[3];
    snapshot->stacked_r12 = core[4];
    snapshot->stacked_lr = core[5];
    snapshot->stacked_pc = core[6];
    snapshot->stacked_xpsr = core[7];
    snapshot->frame_valid = 1U;
    return true;
}

void fault_capture_entry(const uint32_t *frame, uint32_t exc_return,
                         uint32_t original_msp, uint32_t original_psp) {
    /* The naked wrapper already masks normal IRQs and selects a private MSP.
     * Stop the bridge before collecting any optional diagnostic information. */
    motor_driver_emergency_stop();
    g_fault_snapshot.magic = 0U;
    g_fault_snapshot.version = FAULT_CAPTURE_VERSION;
    g_fault_snapshot.exception_number = g_fault_capture_guard;
    g_fault_snapshot.exc_return = exc_return;
    g_fault_snapshot.original_msp = original_msp;
    g_fault_snapshot.original_psp = original_psp;
    g_fault_snapshot.cfsr = SCB->CFSR;
    g_fault_snapshot.hfsr = SCB->HFSR;
    g_fault_snapshot.mmfar = SCB->MMFAR;
    g_fault_snapshot.bfar = SCB->BFAR;
    g_fault_snapshot.mmfar_valid = (g_fault_snapshot.cfsr >> 7U) & 1U;
    g_fault_snapshot.bfar_valid = (g_fault_snapshot.cfsr >> 15U) & 1U;
    g_fault_snapshot.cpacr = SCB->CPACR;
    g_fault_snapshot.vtor = SCB->VTOR;
    g_fault_snapshot.icsr = SCB->ICSR;
    g_fault_snapshot.shcsr = SCB->SHCSR;
    (void)fault_capture_decode_frame(&g_fault_snapshot, (uintptr_t)frame,
                                     exc_return, g_fault_snapshot.cfsr,
                                     FAULT_CAPTURE_SRAM_BEGIN, FAULT_CAPTURE_SRAM_END);
    __DSB();
    g_fault_snapshot.magic = FAULT_CAPTURE_MAGIC;
    __DSB();
    for (;;) { __NOP(); }
}

void fault_capture_nested_halt(void) {
    /* A second private stack avoids overwriting the first helper's locals.
     * Preserve a completed snapshot. If capture was interrupted before its
     * first C write, retain the first exception number without marking it valid. */
    motor_driver_emergency_stop();
    if (g_fault_snapshot.magic != FAULT_CAPTURE_MAGIC) {
        g_fault_snapshot.exception_number = g_fault_capture_guard;
    }
    for (;;) { __NOP(); }
}
