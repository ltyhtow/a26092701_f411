#ifndef TEST_EEPROM_HAL_H
#define TEST_EEPROM_HAL_H
#include <stdint.h>

typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef struct { unsigned unused; } I2C_TypeDef;
typedef struct { unsigned unused; } GPIO_TypeDef;
typedef struct {
    uint32_t ClockSpeed, DutyCycle, OwnAddress1, AddressingMode;
    uint32_t DualAddressMode, OwnAddress2, GeneralCallMode, NoStretchMode;
} I2C_InitTypeDef;
typedef struct { I2C_TypeDef *Instance; I2C_InitTypeDef Init; uint32_t ErrorCode; } I2C_HandleTypeDef;
typedef struct { uint32_t Pin, Mode, Pull, Speed, Alternate; } GPIO_InitTypeDef;
extern I2C_TypeDef mock_i2c2;
extern GPIO_TypeDef mock_gpiob;
#define I2C2 (&mock_i2c2)
#define GPIOB (&mock_gpiob)
#define GPIO_PIN_9 (1U << 9)
#define GPIO_PIN_10 (1U << 10)
#define GPIO_MODE_AF_OD 0x12U
#define GPIO_NOPULL 0U
#define GPIO_SPEED_FREQ_LOW 0U
#define GPIO_AF4_I2C2 4U
#define GPIO_AF9_I2C2 9U
#define I2C_DUTYCYCLE_2 0U
#define I2C_ADDRESSINGMODE_7BIT 0x4000U
#define I2C_DUALADDRESS_DISABLE 0U
#define I2C_GENERALCALL_DISABLE 0U
#define I2C_NOSTRETCH_DISABLE 0U
#define I2C_MEMADD_SIZE_8BIT 1U
void mock_eeprom_clock(unsigned operation);
#define __HAL_RCC_GPIOB_CLK_ENABLE() mock_eeprom_clock(0U)
#define __HAL_RCC_I2C2_CLK_ENABLE() mock_eeprom_clock(1U)
#define __HAL_RCC_I2C2_FORCE_RESET() mock_eeprom_clock(2U)
#define __HAL_RCC_I2C2_RELEASE_RESET() mock_eeprom_clock(3U)
uint32_t __get_IPSR(void);
uint32_t __get_PRIMASK(void);
HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *bus);
HAL_StatusTypeDef HAL_I2C_DeInit(I2C_HandleTypeDef *bus);
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *config);
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *bus, uint16_t address,
    uint16_t reg, uint16_t reg_size, uint8_t *data, uint16_t size, uint32_t timeout);
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *bus, uint16_t address,
    uint16_t reg, uint16_t reg_size, uint8_t *data, uint16_t size, uint32_t timeout);
void HAL_Delay(uint32_t milliseconds);
#endif
