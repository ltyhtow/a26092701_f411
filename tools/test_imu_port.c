/* Real STM32 port + real LibDriver + real sampler. Emulate only bus and RTOS. */
#include "imu_port.h"
#include "imu_task.h"
#include "app_clock.h"
#include "task.h"
#include "i2c.h"

#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>

I2C_HandleTypeDef hi2c1;
static uint8_t registers[256];
static uint16_t present_address = 0xD0U;
static int fail_write_register = -1;
static unsigned fail_reads, sample_index, publications;
static int task_running, sample_fault_armed;
static TaskFunction_t task_entry;
static jmp_buf task_exit;

HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *bus, uint16_t address,
    uint16_t reg, uint16_t reg_size, uint8_t *data, uint16_t size, uint32_t timeout) {
    assert(bus == &hi2c1 && reg_size == 1U && timeout == 20U);
    assert((uint32_t)reg + size <= sizeof(registers));
    if (address != present_address) { bus->ErrorCode = 4U; return HAL_ERROR; }
    if (fail_reads != 0U) { --fail_reads; bus->ErrorCode = 0x20U; return HAL_TIMEOUT; }
    bus->ErrorCode = 0U;
    memcpy(data, registers + reg, size);
    return HAL_OK;
}

HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *bus, uint16_t address,
    uint16_t reg, uint16_t reg_size, uint8_t *data, uint16_t size, uint32_t timeout) {
    assert(bus == &hi2c1 && address == present_address && reg_size == 1U && timeout == 20U);
    assert((uint32_t)reg + size <= sizeof(registers));
    if ((int)reg == fail_write_register) {
        fail_write_register = -1;
        bus->ErrorCode = 0x10U;
        return HAL_ERROR;
    }
    bus->ErrorCode = 0U;
    memcpy(registers + reg, data, size);
    /* The reset bit self-clears and the device wakes in sleep mode. */
    if (reg == 0x6BU && (data[0] & 0x80U) != 0U) registers[reg] = 0x40U;
    return HAL_OK;
}

BaseType_t xTaskGetSchedulerState(void) { return 1; }
void vTaskDelay(TickType_t delay) { assert(delay != 0U); }
void HAL_Delay(uint32_t milliseconds) { assert(milliseconds != 0U); }
TickType_t app_clock_period_ticks(uint32_t milliseconds) { return milliseconds; }
uint32_t app_clock_now_ms(void) { return 100U + sample_index * 5U; }
TickType_t xTaskGetTickCount(void) {
    if (task_running && !sample_fault_armed) { fail_reads = 1U; sample_fault_armed = 1; }
    return app_clock_now_ms();
}

BaseType_t xTaskCreate(TaskFunction_t entry, const char *name, unsigned stack,
    void *argument, UBaseType_t priority, TaskHandle_t *handle) {
    assert(strcmp(name, "IMU") == 0 && stack >= 384U && priority == 3U && argument == NULL);
    task_entry = entry;
    *handle = (void *)3;
    return pdPASS;
}

BaseType_t xQueueOverwrite(QueueHandle_t queue, const void *item) {
    const imu_sample_message_t *message = item;
    assert(queue == (void *)1 || queue == (void *)2);
    assert(message->timestamp_ms == 100U + sample_index * 5U);
    if (sample_index == 0U) {
        assert(message->status_flags == 0U && message->error_code != 0U);
        assert(message->diagnostics.transport_status == HAL_TIMEOUT);
        assert(message->diagnostics.transport_error_codes == 0x20U);
    } else {
        assert(message->status_flags == IMU_SAMPLE_STATUS_VALID && message->error_code == 0U);
        assert(message->diagnostics.transport_status == HAL_OK);
        assert(message->diagnostics.transport_error_codes == 0U);
    }
    ++publications;
    return pdPASS;
}

void vTaskDelayUntil(TickType_t *wake, TickType_t delay) {
    assert(delay == 5U && publications == (sample_index + 1U) * 2U);
    *wake += delay;
    if (++sample_index == 2U) longjmp(task_exit, 1);
}

int main(void) {
    registers[0x75U] = 0x68U;
    assert(imu_port_init() == 0U && imu_port_is_ready());
    imu_port_diagnostics_t diagnostic;
    imu_sample_t sample;
    fail_reads = 1U;
    assert(imu_port_read(&sample) != 0U);
    imu_port_get_diagnostics(&diagnostic);
    assert(diagnostic.transport_status == HAL_TIMEOUT && diagnostic.transport_error_codes == 0x20U);
    assert(imu_port_read(&sample) == 0U);
    imu_port_get_diagnostics(&diagnostic);
    assert(diagnostic.transport_status == HAL_OK && diagnostic.transport_error_codes == 0U);
    assert(imu_port_deinit() == 0U);

    fail_write_register = 0x19; /* Fail setup after the sensor has initialized. */
    assert(imu_port_init() != 0U && !imu_port_is_ready());
    imu_port_get_diagnostics(&diagnostic);
    assert(diagnostic.init_result != 0U);
    assert(diagnostic.transport_status == HAL_ERROR && diagnostic.transport_error_codes == 0x10U);
    assert(hi2c1.ErrorCode == 0U); /* Successful cleanup must not erase failure evidence. */

    present_address = 0xD2U;
    assert(imu_port_init() == 0U);
    imu_port_get_diagnostics(&diagnostic);
    assert(diagnostic.address == 0xD2U && diagnostic.transport_status == HAL_OK);
    assert(imu_port_deinit() == 0U);
    TaskHandle_t handle;
    assert(imu_task_start((void *)1, (void *)2, &handle) == pdPASS);
    task_running = 1;
    if (setjmp(task_exit) == 0) task_entry(NULL);
    assert(publications == 4U && sample_index == 2U);
    puts("PASS IMU port: live bus diagnostics, setup failure retention, same-sample publication");
    return 0;
}
