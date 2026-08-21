/*
 * bno055.c
 *
 *  Created on: Aug 20, 2026
 *      Author: fatma
 */
#include "bno055.h"

/* Fixed-point scale factors from the BNO055 datasheet.
 * Quaternion is always Q14 format regardless of UNIT_SEL.
 * ACC is configured for m/s^2 (100 LSB = 1 m/s^2).
 * GYR is configured for rad/s (900 LSB = 1 rad/s). */
#define QUA_SCALE   (1.0f / 16384.0f)
#define ACC_SCALE   (1.0f / 100.0f)
#define GYR_SCALE   (1.0f / 900.0f)

static I2C_HandleTypeDef *bno_hi2c;

static bool write_reg(uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(bno_hi2c, BNO055_I2C_ADDR, reg,
                              I2C_MEMADD_SIZE_8BIT, &value, 1, 100) == HAL_OK;
}

static bool read_regs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(bno_hi2c, BNO055_I2C_ADDR, reg,
                             I2C_MEMADD_SIZE_8BIT, buf, len, 100) == HAL_OK;
}

bool BNO055_Init(I2C_HandleTypeDef *hi2c)
{
    bno_hi2c = hi2c;

    /* Required power-on stabilization delay before the first I2C
     * transaction — without this, CHIP_ID reads back 0x00 even
     * though HAL reports HAL_OK. Confirmed experimentally. */
    HAL_Delay(700);

    uint8_t chip_id = 0;
    if (!read_regs(BNO055_REG_CHIP_ID, &chip_id, 1)) return false;
    if (chip_id != BNO055_CHIP_ID_VALUE) return false;

    /* Register writes that change operating mode are only accepted
     * while the sensor is in CONFIG mode. Force it there first. */
    if (!write_reg(BNO055_REG_OPR_MODE, BNO055_OPR_MODE_CONFIG)) return false;
    HAL_Delay(25); /* datasheet: mode-switch settling time, ~19 ms max */

    /* UNIT_SEL = 0x02 -> bit1 (GYR_Unit) = 1 (rad/s), bit0 (ACC_Unit) = 0 (m/s^2).
     * Chosen so firmware output units match ROS REP-103 directly,
     * with no unit conversion needed on the host side. */
    if (!write_reg(BNO055_REG_UNIT_SEL, 0x02)) return false;

    /* Switch into NDOF: full 9-DOF sensor fusion (accel + gyro + mag),
     * producing an absolute orientation quaternion. */
    if (!write_reg(BNO055_REG_OPR_MODE, BNO055_OPR_MODE_NDOF)) return false;
    HAL_Delay(25);

    return true;
}

bool BNO055_ReadData(I2C_HandleTypeDef *hi2c, BNO055_Data_t *out)
{
    uint8_t gyr_buf[6];
    uint8_t fusion_buf[14]; /* quaternion (8 bytes) + linear accel (6 bytes), contiguous */

    if (!read_regs(BNO055_REG_GYR_DATA_X, gyr_buf, sizeof(gyr_buf))) return false;
    if (!read_regs(BNO055_REG_QUA_DATA_W, fusion_buf, sizeof(fusion_buf))) return false;

    int16_t gx_raw = (int16_t)(gyr_buf[1] << 8 | gyr_buf[0]);
    int16_t gy_raw = (int16_t)(gyr_buf[3] << 8 | gyr_buf[2]);
    int16_t gz_raw = (int16_t)(gyr_buf[5] << 8 | gyr_buf[4]);

    int16_t qw_raw = (int16_t)(fusion_buf[1]  << 8 | fusion_buf[0]);
    int16_t qx_raw = (int16_t)(fusion_buf[3]  << 8 | fusion_buf[2]);
    int16_t qy_raw = (int16_t)(fusion_buf[5]  << 8 | fusion_buf[4]);
    int16_t qz_raw = (int16_t)(fusion_buf[7]  << 8 | fusion_buf[6]);
    int16_t ax_raw = (int16_t)(fusion_buf[9]  << 8 | fusion_buf[8]);
    int16_t ay_raw = (int16_t)(fusion_buf[11] << 8 | fusion_buf[10]);
    int16_t az_raw = (int16_t)(fusion_buf[13] << 8 | fusion_buf[12]);

    out->qw = qw_raw * QUA_SCALE;
    out->qx = qx_raw * QUA_SCALE;
    out->qy = qy_raw * QUA_SCALE;
    out->qz = qz_raw * QUA_SCALE;

    out->ax = ax_raw * ACC_SCALE;
    out->ay = ay_raw * ACC_SCALE;
    out->az = az_raw * ACC_SCALE;

    out->gx = gx_raw * GYR_SCALE;
    out->gy = gy_raw * GYR_SCALE;
    out->gz = gz_raw * GYR_SCALE;

    return true;
}
