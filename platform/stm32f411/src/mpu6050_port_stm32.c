#include "imu_port.h"
#include "driver_mpu6050.h"

#include "i2c.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app_clock.h"

#include <string.h>

static mpu6050_handle_t mpu6050_handle;
static uint8_t mpu6050_ready;
static imu_port_diagnostics_t mpu6050_diagnostics;

static uint8_t record_transport_result(HAL_StatusTypeDef status) {
    mpu6050_diagnostics.transport_status = (uint32_t)status;
    mpu6050_diagnostics.transport_error_codes = hi2c1.ErrorCode;
    return status == HAL_OK ? 0U : 1U;
}

static uint8_t mpu6050_iic_init(void) {
    return 0U;
}

static uint8_t mpu6050_iic_deinit(void) {
    return 0U;
}

static uint8_t mpu6050_iic_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len) {
    const HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, (uint16_t)addr, reg,
                                                    I2C_MEMADD_SIZE_8BIT, buf, len, 20U);
    return record_transport_result(status);
}

static uint8_t mpu6050_iic_write(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len) {
    const HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c1, (uint16_t)addr, reg,
                                                     I2C_MEMADD_SIZE_8BIT, buf, len, 20U);
    return record_transport_result(status);
}

static void mpu6050_delay_ms(uint32_t delay_ms) {
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        vTaskDelay(app_clock_period_ticks(delay_ms));
    } else {
        HAL_Delay(delay_ms);
    }
}

static void mpu6050_debug_print(const char *const fmt, ...) {
    (void)fmt;
}

static void mpu6050_receive_callback(uint8_t type) {
    (void)type;
}

static void mpu6050_dmp_tap_callback(uint8_t count, uint8_t direction) {
    (void)count;
    (void)direction;
}

static void mpu6050_dmp_orient_callback(uint8_t orientation) {
    (void)orientation;
}

uint8_t imu_port_init(void) {
    uint8_t result;
    uint8_t who_am_i = 0xFFU;
    uint8_t target_addr = MPU6050_ADDRESS_AD0_LOW; /* 0xD0 */
    HAL_StatusTypeDef hal_status = HAL_ERROR;

    memset(&mpu6050_diagnostics, 0, sizeof(mpu6050_diagnostics));
    mpu6050_ready = 0U;
    mpu6050_diagnostics.address = target_addr;
    mpu6050_diagnostics.chip_id = 0xFFU;

    /* 优先探测 0xD0 (AD0 接地) */
    hal_status = HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDRESS_AD0_LOW,
                                  0x75U, I2C_MEMADD_SIZE_8BIT,
                                  &who_am_i, 1U, 20U);
    if (hal_status == HAL_OK && who_am_i == 0x68U) {
        target_addr = MPU6050_ADDRESS_AD0_LOW;
    } else {
        /* 若 0xD0 失败，尝试探测 0xD2 (AD0 接 VCC 或浮空) */
        target_addr = MPU6050_ADDRESS_AD0_HIGH;
        hal_status = HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDRESS_AD0_HIGH,
                                      0x75U, I2C_MEMADD_SIZE_8BIT,
                                      &who_am_i, 1U, 20U);
        if (hal_status == HAL_OK && who_am_i == 0x68U) {
            target_addr = MPU6050_ADDRESS_AD0_HIGH;
        }
    }

    mpu6050_diagnostics.address = target_addr;
    mpu6050_diagnostics.chip_id = who_am_i;
    (void)record_transport_result(hal_status);

    if (hal_status != HAL_OK || who_am_i != 0x68U) {
        mpu6050_ready = 0U;
        mpu6050_diagnostics.init_result = 5U;
        return 5U;
    }

    DRIVER_MPU6050_LINK_INIT(&mpu6050_handle, mpu6050_handle);
    DRIVER_MPU6050_LINK_IIC_INIT(&mpu6050_handle, mpu6050_iic_init);
    DRIVER_MPU6050_LINK_IIC_DEINIT(&mpu6050_handle, mpu6050_iic_deinit);
    DRIVER_MPU6050_LINK_IIC_READ(&mpu6050_handle, mpu6050_iic_read);
    DRIVER_MPU6050_LINK_IIC_WRITE(&mpu6050_handle, mpu6050_iic_write);
    DRIVER_MPU6050_LINK_DELAY_MS(&mpu6050_handle, mpu6050_delay_ms);
    DRIVER_MPU6050_LINK_DEBUG_PRINT(&mpu6050_handle, mpu6050_debug_print);
    DRIVER_MPU6050_LINK_RECEIVE_CALLBACK(&mpu6050_handle, mpu6050_receive_callback);
    mpu6050_handle.dmp_tap_callback = mpu6050_dmp_tap_callback;
    mpu6050_handle.dmp_orient_callback = mpu6050_dmp_orient_callback;

    mpu6050_handle.iic_addr = target_addr;
    result = mpu6050_init(&mpu6050_handle);
    mpu6050_diagnostics.init_result = result;
    if (result != 0U) {
        mpu6050_ready = 0U;
        return result;
    }

    /* Device reset leaves PWR_MGMT_1.SLEEP set on MPU6050; wake it before sampling. */
    result = mpu6050_set_sleep(&mpu6050_handle, MPU6050_BOOL_FALSE);
    if (result == 0U) {
        result = mpu6050_set_clock_source(&mpu6050_handle, MPU6050_CLOCK_SOURCE_PLL_X_GYRO);
    }
    if (result == 0U) {
        result = mpu6050_set_sample_rate_divider(&mpu6050_handle, 4U);
    }
    if (result == 0U) {
        result = mpu6050_set_low_pass_filter(&mpu6050_handle, MPU6050_LOW_PASS_FILTER_3);
    }
    if (result == 0U) {
        result = mpu6050_set_gyroscope_range(&mpu6050_handle, MPU6050_GYROSCOPE_RANGE_500DPS);
    }
    if (result == 0U) {
        result = mpu6050_set_accelerometer_range(&mpu6050_handle, MPU6050_ACCELEROMETER_RANGE_2G);
    }
    if (result != 0U) {
        /* Cleanup can itself touch I2C. Report the failing setup operation. */
        mpu6050_diagnostics.init_result = result;
        const imu_port_diagnostics_t failure = mpu6050_diagnostics;
        (void)mpu6050_deinit(&mpu6050_handle);
        mpu6050_diagnostics = failure;
        mpu6050_ready = 0U;
        return result;
    }
    mpu6050_ready = 1U;
    return 0U;
}

uint8_t imu_port_read(imu_sample_t *sample) {
    uint16_t length = 1U;

    if (sample == NULL || mpu6050_ready == 0U) {
        return 1U;
    }
    return mpu6050_read(&mpu6050_handle, &sample->accel_raw, &sample->accel_g,
                        &sample->gyro_raw, &sample->gyro_dps, &length);
}

void imu_port_get_diagnostics(imu_port_diagnostics_t *diagnostics) {
    if (diagnostics != NULL) {
        *diagnostics = mpu6050_diagnostics;
    }
}

uint8_t imu_port_deinit(void) {
    if (mpu6050_ready == 0U) {
        return 0U;
    }
    mpu6050_ready = 0U;
    return mpu6050_deinit(&mpu6050_handle);
}

uint8_t imu_port_is_ready(void) {
    return mpu6050_ready;
}
