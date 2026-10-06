# LibDriver MPU6050

The `third_party/libdriver` directory contains the MPU6050 driver from
`libdriver/mpu6050`, released under the MIT license.

The project-specific adapter in `src/mpu6050_port.c` connects LibDriver's
platform-independent I2C callbacks to the generated STM32C5 HAL I2C1 handle.
It implements the generic `imu_port.h` contract used by the sampler and
Fusion layer. A future BMI323 port replaces this adapter and its sensor driver;
the sampler, queues, Fusion task, protocol and balance controller do not need
sensor-specific changes.

The application adapter explicitly clears the MPU6050 sleep bit after the
LibDriver reset sequence. The chip reset default keeps `PWR_MGMT_1.SLEEP`
set, and reading output registers while asleep produces zero samples.

The imported LibDriver source files retain their original MIT license headers.

## xioTechnologies Fusion

The `third_party/fusion` directory contains the upstream C implementation from
`xioTechnologies/Fusion`, imported under the MIT license. The imported revision
and license text are recorded in `third_party/FUSION_THIRD_PARTY.md`.

The project uses `FusionAhrs`, `FusionBias`, `FusionRemap`, and the calibration
helpers from `FusionModel.h`. The application adapter in `src/imu_fusion.c`
owns startup stationary calibration and project-specific sensor units; the
upstream Fusion sources are not modified.

LibDriver's address constants use the left-shifted device address form
(`0xD0`/`0xD2`). The STM32C5 HAL master memory APIs also document that the
7-bit datasheet address must be shifted left before the call, so the adapter
passes LibDriver's address through unchanged.
