#ifndef TEST_ENCODER_PUSH_MAIN_H
#define TEST_ENCODER_PUSH_MAIN_H
#include <stdint.h>
typedef struct { uint32_t IDR; } GPIO_TypeDef;
extern GPIO_TypeDef test_gpioa, test_gpiob;
#define GPIOA (&test_gpioa)
#define GPIOB (&test_gpiob)
#endif
