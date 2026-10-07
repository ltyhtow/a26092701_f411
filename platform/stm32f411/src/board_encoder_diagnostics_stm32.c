#include "board_diagnostics.h"
#include "main.h"

uint32_t board_encoder_levels(void) {
    const uint32_t gpioa = GPIOA->IDR, gpiob = GPIOB->IDR;
    return ((gpioa >> 5U) & 1U) | (((gpiob >> 3U) & 1U) << 1U) |
           ((gpioa & 1U) << 2U) | (((gpioa >> 1U) & 1U) << 3U);
}
