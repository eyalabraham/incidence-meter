/*****************************************************************************
* mpu6050.h
*
*  MPU-6050 gyro and accelerometer unit driver
*
*  Created: December 2022
*
*****************************************************************************/

#ifndef __MPU6050_H__
#define __MPU6050_H__

#include    <stdint.h>

/****************************************************************************
  Definitions
****************************************************************************/
#define     MPU_ADD         0x68        // I2C bus address for Gyro/Accel unit

#define     WHO_AM_I        0x75        // MPU identity register, should be equal upper 6-bits of GYRO_ADD
#define     MPU_ID          0x68        // MPU ID in WHO_AM_I register

#define     PWR_MGT_1       0x6b
#define     PWR_MGT_2       0x6c
#define     SMPRT_DIV       0x19        // Sample rate divider = (8KHz / (<div> + 1))
#define     CONFIG          0x1a        // Configuration registers
#define     GYRO_CONFIG     0x1b
#define     ACCEL_CONFIG    0x1c
#define     ACCEL_X         0x3b        // Accelerometers
#define     ACCEL_Y         0x3d
#define     ACCEL_Z         0x3f
#define     GYRO_X          0x43        // Gyro registers
#define     GYRO_Y          0x45
#define     GYRO_Z          0x47

#define     DLPF_CFG        6           // Register 26 - Digital Low Pass Filter configuration
#define     ACCEL_SETUP     0x00        // Register 28 - Accelerometer Configuration full scale range ±2g
#define     GYRO_SETUP      0x08        // Register 27 - Gyroscope Configuration full scale range ±500 deg/s
#define     ACCEL_SCALER    16384.0     // Divisor to scale accelerometer reading
#define     GYRO_SCALER     65.5        // Divisor to scale gyro reading
#define     SMPL_RATE_DIV   7           // For 1,000Hz
#define     UINT2C(v)       ( (v >= 0x8000) ?  -((65535 - v) + 1) : v )  // convert to signed integer

/****************************************************************************
  Function prototypes
****************************************************************************/
int     mpu6050_init(void);
float   mpu6050_get_angle(void);

#endif  /* __MPU6050_H__ */