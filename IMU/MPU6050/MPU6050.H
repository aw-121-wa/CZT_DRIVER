#ifndef MPU6050_H
#define MPU6050_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * I2C 地址
 * STM32 HAL 使用左移 1 位后的地址
 * ============================================================ */

#define MPU6050_ADDR_AD0_LOW        (0x68U << 1)
#define MPU6050_ADDR_AD0_HIGH       (0x69U << 1)


/* ============================================================
 * MPU6050 寄存器
 * ============================================================ */

#define MPU6050_REG_SMPLRT_DIV      0x19U
#define MPU6050_REG_CONFIG          0x1AU
#define MPU6050_REG_GYRO_CONFIG     0x1BU
#define MPU6050_REG_ACCEL_CONFIG    0x1CU

#define MPU6050_REG_ACCEL_XOUT_H    0x3BU
#define MPU6050_REG_TEMP_OUT_H      0x41U
#define MPU6050_REG_GYRO_XOUT_H     0x43U

#define MPU6050_REG_PWR_MGMT_1      0x6BU
#define MPU6050_REG_WHO_AM_I        0x75U

#define MPU6050_DEVICE_ID           0x68U


/* ============================================================
 * 默认配置
 * ============================================================ */

#define MPU6050_I2C_TIMEOUT_MS      100U

/* 默认采样率分频值 */
#define MPU6050_DEFAULT_SMPLRT_DIV  0x07U

/*
 * DLPF 配置：
 * 0~6 为有效常用配置。
 * 默认 3。
 */
#define MPU6050_DEFAULT_DLPF_CFG    0x03U


/* ============================================================
 * 量程枚举
 * ============================================================ */

typedef enum
{
    MPU6050_ACCEL_RANGE_2G  = 0,
    MPU6050_ACCEL_RANGE_4G  = 1,
    MPU6050_ACCEL_RANGE_8G  = 2,
    MPU6050_ACCEL_RANGE_16G = 3

} MPU6050_AccelRange_t;


typedef enum
{
    MPU6050_GYRO_RANGE_250DPS  = 0,
    MPU6050_GYRO_RANGE_500DPS  = 1,
    MPU6050_GYRO_RANGE_1000DPS = 2,
    MPU6050_GYRO_RANGE_2000DPS = 3

} MPU6050_GyroRange_t;


/* ============================================================
 * 设备对象
 * ============================================================ */

typedef struct
{
    I2C_HandleTypeDef *hi2c;
    uint16_t address;

    MPU6050_AccelRange_t accel_range;
    MPU6050_GyroRange_t gyro_range;

    float accel_sensitivity;
    float gyro_sensitivity;

    /* 原始数据 */
    int16_t raw_accel_x;
    int16_t raw_accel_y;
    int16_t raw_accel_z;

    int16_t raw_temp;

    int16_t raw_gyro_x;
    int16_t raw_gyro_y;
    int16_t raw_gyro_z;

    /* 换算后的物理量 */
    float accel_x;      /* g */
    float accel_y;      /* g */
    float accel_z;      /* g */

    float temperature;  /* °C */

    float gyro_x;       /* °/s */
    float gyro_y;       /* °/s */
    float gyro_z;       /* °/s */

} MPU6050_Handle_t;


/* ============================================================
 * 公共接口
 * ============================================================ */

/*
 * 初始化 MPU6050。
 *
 * 默认：
 * - 唤醒设备
 * - PLL 时钟源
 * - DLPF = 3
 * - SMPLRT_DIV = 7
 * - 加速度 ±2g
 * - 陀螺仪 ±250 dps
 */
HAL_StatusTypeDef MPU6050_Init(
    MPU6050_Handle_t *dev,
    I2C_HandleTypeDef *hi2c,
    uint16_t address
);


/* 一次连续读取加速度、温度、陀螺仪 14 字节 */
HAL_StatusTypeDef MPU6050_Update(
    MPU6050_Handle_t *dev
);


/* 设置加速度量程 */
HAL_StatusTypeDef MPU6050_SetAccelRange(
    MPU6050_Handle_t *dev,
    MPU6050_AccelRange_t range
);


/* 设置陀螺仪量程 */
HAL_StatusTypeDef MPU6050_SetGyroRange(
    MPU6050_Handle_t *dev,
    MPU6050_GyroRange_t range
);


/* 设置数字低通滤波器 DLPF_CFG，范围 0~6 */
HAL_StatusTypeDef MPU6050_SetDLPF(
    MPU6050_Handle_t *dev,
    uint8_t dlpf_cfg
);


/* 设置采样率分频寄存器 */
HAL_StatusTypeDef MPU6050_SetSampleRateDivider(
    MPU6050_Handle_t *dev,
    uint8_t divider
);


/* 再次检测 WHO_AM_I */
HAL_StatusTypeDef MPU6050_CheckDevice(
    MPU6050_Handle_t *dev
);

#ifdef __cplusplus
}
#endif

#endif /* MPU6050_H */