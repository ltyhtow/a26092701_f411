#ifndef IMU_PORT_H
#define IMU_PORT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int16_t accel_raw[3];
    float accel_g[3];
    int16_t gyro_raw[3];
    float gyro_dps[3];
} imu_sample_t;

typedef struct {
    uint8_t address;
    uint8_t chip_id;
    uint16_t init_result;
    uint32_t hal_status;
    uint32_t hal_error_codes;
} imu_port_diagnostics_t;

uint8_t imu_port_init(void);
uint8_t imu_port_read(imu_sample_t *sample);
void imu_port_get_diagnostics(imu_port_diagnostics_t *diagnostics);
uint8_t imu_port_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_PORT_H */
