/*
 * mpu_6050.h
 *
 *  Created on: May 8, 2024
 *      Author: angel
 */

#ifndef INC_MPU_6050_H_
#define INC_MPU_6050_H_


#define MPU_6050_ADDRESS 0b11010000

#define WHO_AM_I_REG 		117
#define PWR_MGMT_1_REG 		107
#define SMPLRT_DIV_REG 		25
#define ACCEL_CONFIG_REG 	28
#define GYRO_CONFIG_REG		27
#define ACCEL_XOUT_H_REG    59


void MPU6050_Init(void);
void MPU6050_Read_Accel(void);

#endif /* INC_MPU_6050_H_ */
