#ifndef MPU6050_PORT_H
#define MPU6050_PORT_H

/* Compatibility include for code that still names the current sensor port. */

#include <stdint.h>
#include "imu_port.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef imu_sample_t mpu6050_sample_t;
typedef imu_port_diagnostics_t mpu6050_diagnostics_t;

#ifdef __cplusplus
}
#endif

#endif /* MPU6050_PORT_H */
