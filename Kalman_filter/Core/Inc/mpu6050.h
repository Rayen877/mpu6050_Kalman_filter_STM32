/*
 * mpu6050.h
 *
 *  Created on: Nov 14, 2026
 *      Author: Za7ze7
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "stm32f4xx_hal.h"

typedef struct {
    float ax, ay, az;   /* g     */
    float gx, gy, gz;   /* deg/s */
} MPU6050_Data_t;

HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef MPU6050_Read(I2C_HandleTypeDef *hi2c, MPU6050_Data_t *d);


#endif /* INC_MPU6050_H_ */
