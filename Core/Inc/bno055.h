#ifndef BNO055_H
#define BNO055_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* I2C address: 0x28 (7-bit), shifted left by 1 for HAL = 0x50 */
#define BNO055_I2C_ADDR         (0x28 << 1)

/* Register addresses used */
#define BNO055_REG_CHIP_ID      0x00
#define BNO055_REG_OPR_MODE     0x3D
#define BNO055_REG_UNIT_SEL     0x3B
#define BNO055_REG_GYR_DATA_X   0x14  /* 6 bytes: gyro X,Y,Z */
#define BNO055_REG_QUA_DATA_W   0x20  /* 14 bytes: quaternion (8) + linear accel (6) */
#define BNO055_REG_CALIB_STAT   0x35

/* Operating modes */
#define BNO055_OPR_MODE_CONFIG  0x00
#define BNO055_OPR_MODE_NDOF    0x0C

#define BNO055_CHIP_ID_VALUE    0xA0

typedef struct {
    float qw, qx, qy, qz;   /* unit quaternion, dimensionless */
    float ax, ay, az;       /* linear acceleration, m/s^2 (gravity removed) */
    float gx, gy, gz;       /* angular velocity, rad/s */
} BNO055_Data_t;

/**
 * Initializes the BNO055 in NDOF fusion mode.
 * Returns true on success (chip ID verified), false otherwise.
 */
bool BNO055_Init(I2C_HandleTypeDef *hi2c);

/**
 * Reads quaternion, linear acceleration, and angular velocity in one call.
 * Returns true on success.
 */
bool BNO055_ReadData(I2C_HandleTypeDef *hi2c, BNO055_Data_t *out);

#endif /* BNO055_H */
