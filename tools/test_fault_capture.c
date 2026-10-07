#include "fault_capture.h"
#include "stm32f4xx.h"
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>

SCB_Type test_fault_scb;
static jmp_buf halted;
static unsigned stops, barriers;
static uint32_t expected_magic_when_stopped;

void motor_driver_emergency_stop(void) {
    assert(g_fault_snapshot.magic == expected_magic_when_stopped);
    ++stops;
}
void test_fault_barrier(void) {
    assert(stops == 1U);
    if (barriers == 0U) assert(g_fault_snapshot.magic == 0U);
    else assert(g_fault_snapshot.magic == FAULT_CAPTURE_MAGIC);
    ++barriers;
}
void test_fault_halt(void) { longjmp(halted, 1); }

static void verify_registers(const fault_snapshot_t *s, const uint32_t *core) {
    assert(s->frame_valid == 1U);
    assert(s->stacked_r0 == core[0] && s->stacked_r1 == core[1]);
    assert(s->stacked_r2 == core[2] && s->stacked_r3 == core[3]);
    assert(s->stacked_r12 == core[4] && s->stacked_lr == core[5]);
    assert(s->stacked_pc == core[6] && s->stacked_xpsr == core[7]);
}
static void verify_invalid(uintptr_t address, uint32_t exc_return, uint32_t cfsr,
                           uintptr_t begin, uintptr_t end) {
    fault_snapshot_t s;
    memset(&s, 0xA5, sizeof(s));
    assert(!fault_capture_decode_frame(&s, address, exc_return, cfsr, begin, end));
    assert(s.frame_valid == 0U);
    assert(s.stacked_r0 == 0U && s.stacked_r1 == 0U && s.stacked_r2 == 0U && s.stacked_r3 == 0U);
    assert(s.stacked_r12 == 0U && s.stacked_lr == 0U && s.stacked_pc == 0U && s.stacked_xpsr == 0U);
}
static void test_decoder(void) {
    uint32_t frame[28];
    for (unsigned n = 0; n < 28U; ++n) frame[n] = 0x1000U + n;
    const uintptr_t begin = (uintptr_t)frame;
    const uintptr_t end = begin + sizeof(frame);
    fault_snapshot_t s;
    for (unsigned kind = 0; kind < 6U; ++kind) {
        static const uint32_t returns[] = {0xFFFFFFF1U, 0xFFFFFFF9U, 0xFFFFFFFDU,
                                           0xFFFFFFE1U, 0xFFFFFFE9U, 0xFFFFFFEDU};
        /* Official ARM DUI0553A Figure 2-3: the common core words are LOWEST
         * in memory. Distinct trailing words catch an incorrect +18 decoder. */
        /* INVSTATE/T=0 is captured, not rejected as unreadable frame data. */
        frame[7] = 0U;
        assert(fault_capture_decode_frame(&s, begin, returns[kind], 0x00020000U, begin, end));
        assert(s.frame_extended == (kind >= 3U ? 1U : 0U));
        verify_registers(&s, frame);
        assert(s.stacked_pc != frame[24] && s.stacked_xpsr != frame[25]);
    }
    /* Only 4-byte source alignment is required when STKALIGN is disabled. */
    assert(fault_capture_decode_frame(&s, begin + 4U, 0xFFFFFFFDU, 0U, begin, end));
    verify_registers(&s, &frame[1]);
    /* A padding word may follow xPSR when STKALIGN=1; it is not a leading word. */
    frame[7] = 0x01000200U;
    assert(fault_capture_decode_frame(&s, begin, 0xFFFFFFF9U, 0U, begin, begin + 32U));
    verify_registers(&s, frame);
    assert(!fault_capture_decode_frame(NULL, begin, 0xFFFFFFF9U, 0U, begin, end));
    verify_invalid(0U, 0xFFFFFFF9U, 0U, begin, end);
    verify_invalid(begin - 4U, 0xFFFFFFF9U, 0U, begin, end);
    verify_invalid(begin + 1U, 0xFFFFFFF9U, 0U, begin, end);
    verify_invalid(end, 0xFFFFFFF9U, 0U, begin, end);
    verify_invalid(end - 28U, 0xFFFFFFF9U, 0U, begin, end);
    verify_invalid(begin, 0xFFFFFFEDU, 0U, begin, begin + 100U);
    verify_invalid(begin, 0xFFFFFFF9U, 0U, end, begin);
    verify_invalid(UINTPTR_MAX - 3U, 0xFFFFFFEDU, 0U, 0U, UINTPTR_MAX);
    verify_invalid(begin, 0xFFFFFFF0U, 0U, begin, end);
    verify_invalid(begin, 0xFFFFFFF5U, 0U, begin, end);
    verify_invalid(begin, 0x08001235U, 0U, begin, end);
    static const unsigned stack_bits[] = {3U, 4U, 5U, 11U, 12U, 13U};
    for (unsigned n = 0; n < sizeof(stack_bits) / sizeof(stack_bits[0]); ++n) {
        /* Even a nominally in-range address is unread if stacking failed. */
        verify_invalid(0x20001000U, 0xFFFFFFFDU, 1U << stack_bits[n],
                       FAULT_CAPTURE_SRAM_BEGIN, FAULT_CAPTURE_SRAM_END);
    }
    assert(stops == 0U && barriers == 0U);
}
static void test_capture(bool valid_address_flags) {
    memset((void *)&g_fault_snapshot, 0, sizeof(g_fault_snapshot));
    g_fault_capture_guard = 3U;
    expected_magic_when_stopped = 0U;
    stops = barriers = 0U;
    test_fault_scb.CFSR = 0x00020000U | (valid_address_flags ? 0x8080U : 0U);
    test_fault_scb.HFSR = 0x40000000U;
    test_fault_scb.MMFAR = test_fault_scb.BFAR = 0xE000EDF8U;
    test_fault_scb.CPACR = 0x00F00000U;
    test_fault_scb.VTOR = 0U;
    test_fault_scb.ICSR = 3U;
    test_fault_scb.SHCSR = 0x12340000U;
    /* Invalid frame pointer intentionally is not mapped into this host process. */
    if (setjmp(halted) == 0) fault_capture_entry((const uint32_t *)(uintptr_t)0xE000EDF8U,
                                                0xFFFFFFFDU, 0x2001FFE0U, 0xE000EDF8U);
    assert(stops == 1U && barriers == 2U);
    assert(g_fault_snapshot.magic == FAULT_CAPTURE_MAGIC && g_fault_snapshot.version == 1U);
    assert(g_fault_snapshot.exception_number == 3U && g_fault_snapshot.exc_return == 0xFFFFFFFDU);
    assert(g_fault_snapshot.original_msp == 0x2001FFE0U && g_fault_snapshot.original_psp == 0xE000EDF8U);
    assert(g_fault_snapshot.frame_address == 0xE000EDF8U && g_fault_snapshot.frame_valid == 0U);
    assert(g_fault_snapshot.cfsr == test_fault_scb.CFSR && g_fault_snapshot.hfsr == 0x40000000U);
    assert(g_fault_snapshot.mmfar == 0xE000EDF8U && g_fault_snapshot.bfar == 0xE000EDF8U);
    assert(g_fault_snapshot.mmfar_valid == valid_address_flags && g_fault_snapshot.bfar_valid == valid_address_flags);
    assert(g_fault_snapshot.cpacr == 0x00F00000U && g_fault_snapshot.vtor == 0U);
    assert(g_fault_snapshot.icsr == 3U && g_fault_snapshot.shcsr == 0x12340000U);
    fault_snapshot_t before;
    memcpy(&before, (const void *)&g_fault_snapshot, sizeof(before));
    expected_magic_when_stopped = FAULT_CAPTURE_MAGIC;
    if (setjmp(halted) == 0) fault_capture_nested_halt();
    assert(stops == 2U && g_fault_capture_guard == 3U);
    assert(memcmp(&before, (const void *)&g_fault_snapshot, sizeof(before)) == 0);
    g_fault_snapshot.magic = 0U;
    g_fault_snapshot.exception_number = 0U;
    expected_magic_when_stopped = 0U;
    if (setjmp(halted) == 0) fault_capture_nested_halt();
    assert(g_fault_snapshot.magic == 0U && g_fault_snapshot.exception_number == 3U);
}
int main(void) {
    test_decoder();
    test_capture(false);
    test_capture(true);
    puts("PASS checked Cortex-M4 basic/extended frames, bounds, stacking errors and first-fault capture");
    return 0;
}
