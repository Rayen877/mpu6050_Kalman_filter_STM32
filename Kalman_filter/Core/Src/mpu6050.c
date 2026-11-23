/*
 * mpu6050.c
 *
 *  Created on: Nov 14, 2026
 *      Author: Za7ze7
 */

#include "mpu6050.h"

#define MPU_ADDR          (0x68 << 1)   /* AD0 = GND. Use 0x69 if AD0 = VCC */

#define REG_SMPLRT_DIV    0x19
#define REG_CONFIG        0x1A
#define REG_GYRO_CONFIG   0x1B
#define REG_ACCEL_CONFIG  0x1C
#define REG_ACCEL_XOUT_H  0x3B
#define REG_PWR_MGMT_1    0x6B
#define REG_WHO_AM_I      0x75

#define ACCEL_SENS        16384.0f   /* LSB/g    for +-2 g    */
#define GYRO_SENS         65.5f      /* LSB/dps  for +-500 dps */

static HAL_StatusTypeDef wr(I2C_HandleTypeDef *h, uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(h, MPU_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
}

HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c)
{
    wr(hi2c, REG_PWR_MGMT_1, 0x01);     /* wake up, PLL gyro X */
    HAL_Delay(100);
    wr(hi2c, REG_SMPLRT_DIV, 0x00);     /* 1 kHz / (1+0)       */
    wr(hi2c, REG_CONFIG, 0x03);         /* DLPF ~44 Hz         */
    wr(hi2c, REG_GYRO_CONFIG, 0x08);    /* +-500 dps           */
    wr(hi2c, REG_ACCEL_CONFIG, 0x00);   /* +-2 g               */

    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_Read(I2C_HandleTypeDef *hi2c, MPU6050_Data_t *d)
{
    uint8_t b[14];

    if (HAL_I2C_Mem_Read(hi2c, MPU_ADDR, REG_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, b, 14, 100) != HAL_OK)
        return HAL_ERROR;

    int16_t ax = (int16_t)((b[0]  << 8) | b[1]);
    int16_t ay = (int16_t)((b[2]  << 8) | b[3]);
    int16_t az = (int16_t)((b[4]  << 8) | b[5]);
    /* b[6], b[7] = temperature */
    int16_t gx = (int16_t)((b[8]  << 8) | b[9]);
    int16_t gy = (int16_t)((b[10] << 8) | b[11]);
    int16_t gz = (int16_t)((b[12] << 8) | b[13]);

    d->ax = ax / ACCEL_SENS;
    d->ay = ay / ACCEL_SENS;
    d->az = az / ACCEL_SENS;
    d->gx = gx / GYRO_SENS;
    d->gy = gy / GYRO_SENS;
    d->gz = gz / GYRO_SENS;

    return HAL_OK;
}
