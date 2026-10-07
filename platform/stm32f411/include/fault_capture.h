#ifndef FAULT_CAPTURE_H
#define FAULT_CAPTURE_H

#include <stdbool.h>
#include <stdint.h>

#define FAULT_CAPTURE_MAGIC 0x4641554CUL /* "FAUL": written only after capture completes. */
#define FAULT_CAPTURE_VERSION 1U
#define FAULT_CAPTURE_STACK_BYTES 512
#define FAULT_CAPTURE_SRAM_BEGIN ((uintptr_t)0x20000000UL)
#define FAULT_CAPTURE_SRAM_END ((uintptr_t)0x20020000UL) /* F411CE: 128 KiB SRAM. */
/* MMFSR MUNSTKERR/MSTKERR/MLSPERR and BFSR UNSTKERR/STKERR/LSPERR. */
#define FAULT_CAPTURE_STACK_ERROR_MASK 0x00003838UL

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t exception_number; /* IPSR: NMI=2, HardFault=3, Mem=4, Bus=5, Usage=6. */
    uint32_t exc_return;
    uint32_t original_msp;
    uint32_t original_psp;
    uint32_t frame_address;    /* Hardware SP points to R0 in BOTH basic and extended frames. */
    uint32_t frame_valid;      /* Range/layout are readable; register values can still be corrupt. */
    uint32_t frame_extended;
    uint32_t cfsr;
    uint32_t hfsr;
    uint32_t mmfar;
    uint32_t bfar;
    uint32_t mmfar_valid;      /* Interpret address only when this CFSR validity bit is set. */
    uint32_t bfar_valid;
    uint32_t cpacr;
    uint32_t vtor;
    uint32_t icsr;
    uint32_t shcsr;
    uint32_t stacked_r0;
    uint32_t stacked_r1;
    uint32_t stacked_r2;
    uint32_t stacked_r3;
    uint32_t stacked_r12;
    uint32_t stacked_lr;
    uint32_t stacked_pc;
    uint32_t stacked_xpsr;
} fault_snapshot_t;

/* Inspect directly in CLion/GDB after the processor has halted in the fault loop.
 * SRAM state, not persistent storage: a reset clears this snapshot.
 * A nonzero guard with magic=0 means the first capture could not finish, e.g. NMI
 * preempted it. The guard retains the first claimed exception number. */
extern volatile fault_snapshot_t g_fault_snapshot;
extern volatile uint32_t g_fault_capture_guard;
extern uint32_t g_fault_capture_stack[FAULT_CAPTURE_STACK_BYTES / sizeof(uint32_t)];
extern uint32_t g_fault_nested_stack[FAULT_CAPTURE_STACK_BYTES / sizeof(uint32_t)];

/* Integer-only checked frame decoder, exposed for host verification. Invalid
 * input never dereferences the frame; all stacked values are cleared on failure.
 * T=0 in stacked xPSR is evidence (e.g. INVSTATE), so it is deliberately retained. */
bool fault_capture_decode_frame(volatile fault_snapshot_t *snapshot,
                                uintptr_t frame_address, uint32_t exc_return,
                                uint32_t cfsr, uintptr_t sram_begin, uintptr_t sram_end);

__attribute__((noreturn)) void fault_capture_entry(const uint32_t *frame,
                                                   uint32_t exc_return,
                                                   uint32_t original_msp,
                                                   uint32_t original_psp);
__attribute__((noreturn)) void fault_capture_nested_halt(void);

#endif
