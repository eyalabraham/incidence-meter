/*****************************************************************************
* mpu6050.c
*
*  MPU-6050 gyro and accelerometer unit driver
*
* Created: December 2022
*
*****************************************************************************/

#include    <math.h>

#include    "inc/i2c_drv.h"
#include    "inc/mpu6050.h"

/****************************************************************************
  Types and definitions
****************************************************************************/
#define     MPU_REGS        sizeof(struct mpuReg_t)  // MPU register data buffer size in bytes

/****************************************************************************
  Local function prototypes
****************************************************************************/

/****************************************************************************
  Module globals
****************************************************************************/
/* Union to allow register burst read
*/
static union mpu6050_t              // Union data structure
{
    struct mpuReg_t
    {
        uint16_t    ACCEL_XOUT;     // Register list
        uint16_t    ACCEL_YOUT;
        uint16_t    ACCEL_ZOUT;
        uint16_t    TEMP_OUT;
        uint16_t    GYRO_XOUT;
        uint16_t    GYRO_YOUT;
    } mpuReg;

    uint8_t     mpuData[MPU_REGS];  // Data buffer reference
} mpu6050;

static float    accel_scale;        // Accelerometer scaler
static float    gyro_scale;         // Gyro scaler

/****************************************************************************
  Module functions
****************************************************************************/

/* ----------------------------------------------------------------------------
* mpu6050_init()
*
*  Intialize the TWI (I2C) and the MPU-6050
*
*  param:  none
*  return: 0=error, -1=initialization ok
*
*/
int mpu6050_init(void)
{
    uint8_t result;

    /* I2C initialization as bus master
    */
    i2c_c_initialize();

    /* Check if MPU-6050 is connected and responding.
       If not, then return error, otherwise continue with setup
    */
    i2c_c_rx_byte(MPU_ADD, WHO_AM_I, &result);
    if ( result != MPU_ID )
        return 0;

    /* Bring MPU-6050 out of sleep mode
       and setup resolution, accuracy, filters
    */
    i2c_c_tx_byte(MPU_ADD, PWR_MGT_1, 0x08);
    i2c_c_tx_byte(MPU_ADD, PWR_MGT_2, 0x00);
    i2c_c_tx_byte(MPU_ADD, SMPRT_DIV, SMPL_RATE_DIV);
    i2c_c_tx_byte(MPU_ADD, CONFIG, DLPF_CFG);
    i2c_c_tx_byte(MPU_ADD, ACCEL_CONFIG, ACCEL_SETUP);
    i2c_c_tx_byte(MPU_ADD, GYRO_CONFIG, GYRO_SETUP);

    accel_scale = (float) ACCEL_SCALER;
    gyro_scale = (float) GYRO_SCALER;

    return -1;
}

/* ----------------------------------------------------------------------------
* mpu6050_get_angle()
*
*  Get device tilt angle
*
*  param:  none
*  return: Angle in floating point
*
*/
float mpu6050_get_angle(void)
{
    int     sensor_err;
    uint8_t tmp;
    float   distance, pitch;

    float accel_x;  // Aaccelerometer readings
    float accel_y;
    float accel_z;

    /* Burst read accelerometer and gyro
    */
    sensor_err = i2c_c_burst_read(MPU_ADD, ACCEL_X, MPU_REGS, mpu6050.mpuData);
    if ( sensor_err == -1 )
        return -199.0;

    /* Swap bytes to convert to little endian
    */
    tmp = mpu6050.mpuData[0];
    mpu6050.mpuData[0] = mpu6050.mpuData[1];
    mpu6050.mpuData[1] = tmp;

    tmp = mpu6050.mpuData[2];
    mpu6050.mpuData[2] = mpu6050.mpuData[3];
    mpu6050.mpuData[3] = tmp;

    tmp = mpu6050.mpuData[4];
    mpu6050.mpuData[4] = mpu6050.mpuData[5];
    mpu6050.mpuData[5] = tmp;

    tmp = mpu6050.mpuData[10];
    mpu6050.mpuData[10] = mpu6050.mpuData[11];
    mpu6050.mpuData[11] = tmp;

    /* Scale readings
    */
    accel_x = (float) UINT2C(mpu6050.mpuReg.ACCEL_XOUT) / (float) accel_scale;
    accel_y = (float) UINT2C(mpu6050.mpuReg.ACCEL_YOUT) / (float) accel_scale;
    accel_z = (float) UINT2C(mpu6050.mpuReg.ACCEL_ZOUT) / (float) accel_scale;

    /* Angle calculation (http://www.hobbytronics.co.uk/accelerometer-info)
    */
    distance = sqrt(accel_z*accel_z + accel_x*accel_x);
    pitch = atan(accel_y / distance) * (float) 57.2957795 * -1.0;   // [rad] to [deg]

    return pitch;
}