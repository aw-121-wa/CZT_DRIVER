#include "MPU6050.h"

#include <string.h>


/* ============================================================
 * 私有函数
 * ============================================================ */

static HAL_StatusTypeDef MPU6050_ReadReg(
    MPU6050_Handle_t *dev,
    uint8_t reg,
    uint8_t *data
)
{
    if (dev == NULL ||
        dev->hi2c == NULL ||
        data == NULL)
    {
        return HAL_ERROR;
    }

    return HAL_I2C_Mem_Read(
        dev->hi2c,
        dev->address,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        data,
        1U,
        MPU6050_I2C_TIMEOUT_MS
    );
}


static HAL_StatusTypeDef MPU6050_ReadRegs(
    MPU6050_Handle_t *dev,
    uint8_t reg,
    uint8_t *data,
    uint16_t len
)
{
    if (dev == NULL ||
        dev->hi2c == NULL ||
        data == NULL ||
        len == 0U)
    {
        return HAL_ERROR;
    }

    return HAL_I2C_Mem_Read(
        dev->hi2c,
        dev->address,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        data,
        len,
        MPU6050_I2C_TIMEOUT_MS
    );
}


static HAL_StatusTypeDef MPU6050_WriteReg(
    MPU6050_Handle_t *dev,
    uint8_t reg,
    uint8_t data
)
{
    if (dev == NULL ||
        dev->hi2c == NULL)
    {
        return HAL_ERROR;
    }

    return HAL_I2C_Mem_Write(
        dev->hi2c,
        dev->address,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        &data,
        1U,
        MPU6050_I2C_TIMEOUT_MS
    );
}


static int16_t MPU6050_ReadInt16BE(
    const uint8_t *data
)
{
    return (int16_t)(
        ((uint16_t)data[0] << 8) |
        (uint16_t)data[1]
    );
}


static float MPU6050_GetAccelSensitivity(
    MPU6050_AccelRange_t range
)
{
    switch (range)
    {
        case MPU6050_ACCEL_RANGE_2G:
            return 16384.0f;

        case MPU6050_ACCEL_RANGE_4G:
            return 8192.0f;

        case MPU6050_ACCEL_RANGE_8G:
            return 4096.0f;

        case MPU6050_ACCEL_RANGE_16G:
            return 2048.0f;

        default:
            return 0.0f;
    }
}


static float MPU6050_GetGyroSensitivity(
    MPU6050_GyroRange_t range
)
{
    switch (range)
    {
        case MPU6050_GYRO_RANGE_250DPS:
            return 131.0f;

        case MPU6050_GYRO_RANGE_500DPS:
            return 65.5f;

        case MPU6050_GYRO_RANGE_1000DPS:
            return 32.8f;

        case MPU6050_GYRO_RANGE_2000DPS:
            return 16.4f;

        default:
            return 0.0f;
    }
}


/* ============================================================
 * 设备检测
 * ============================================================ */

HAL_StatusTypeDef MPU6050_CheckDevice(
    MPU6050_Handle_t *dev
)
{
    if (dev == NULL ||
        dev->hi2c == NULL)
    {
        return HAL_ERROR;
    }

    uint8_t who_am_i = 0U;

    HAL_StatusTypeDef status =
        MPU6050_ReadReg(
            dev,
            MPU6050_REG_WHO_AM_I,
            &who_am_i
        );

    if (status != HAL_OK)
    {
        return status;
    }

    if (who_am_i != MPU6050_DEVICE_ID)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}


/* ============================================================
 * 加速度量程
 * ============================================================ */

HAL_StatusTypeDef MPU6050_SetAccelRange(
    MPU6050_Handle_t *dev,
    MPU6050_AccelRange_t range
)
{
    if (dev == NULL ||
        range > MPU6050_ACCEL_RANGE_16G)
    {
        return HAL_ERROR;
    }

    uint8_t reg_value = 0U;

    HAL_StatusTypeDef status =
        MPU6050_ReadReg(
            dev,
            MPU6050_REG_ACCEL_CONFIG,
            &reg_value
        );

    if (status != HAL_OK)
    {
        return status;
    }

    /*
     * AFS_SEL 位于 [4:3]
     *
     * 先清除 bit4 和 bit3，
     * 再写入新量程。
     */
    reg_value &= (uint8_t)~(0x18U);

    reg_value |=
        (uint8_t)((uint8_t)range << 3U);

    status =
        MPU6050_WriteReg(
            dev,
            MPU6050_REG_ACCEL_CONFIG,
            reg_value
        );

    if (status != HAL_OK)
    {
        return status;
    }

    dev->accel_range = range;

    dev->accel_sensitivity =
        MPU6050_GetAccelSensitivity(range);

    return HAL_OK;
}


/* ============================================================
 * 陀螺仪量程
 * ============================================================ */

HAL_StatusTypeDef MPU6050_SetGyroRange(
    MPU6050_Handle_t *dev,
    MPU6050_GyroRange_t range
)
{
    if (dev == NULL ||
        range > MPU6050_GYRO_RANGE_2000DPS)
    {
        return HAL_ERROR;
    }

    uint8_t reg_value = 0U;

    HAL_StatusTypeDef status =
        MPU6050_ReadReg(
            dev,
            MPU6050_REG_GYRO_CONFIG,
            &reg_value
        );

    if (status != HAL_OK)
    {
        return status;
    }

    /*
     * FS_SEL 位于 [4:3]
     */
    reg_value &= (uint8_t)~(0x18U);

    reg_value |=
        (uint8_t)((uint8_t)range << 3U);

    status =
        MPU6050_WriteReg(
            dev,
            MPU6050_REG_GYRO_CONFIG,
            reg_value
        );

    if (status != HAL_OK)
    {
        return status;
    }

    dev->gyro_range = range;

    dev->gyro_sensitivity =
        MPU6050_GetGyroSensitivity(range);

    return HAL_OK;
}


/* ============================================================
 * DLPF
 * ============================================================ */

HAL_StatusTypeDef MPU6050_SetDLPF(
    MPU6050_Handle_t *dev,
    uint8_t dlpf_cfg
)
{
    if (dev == NULL ||
        dlpf_cfg > 6U)
    {
        return HAL_ERROR;
    }

    uint8_t reg_value = 0U;

    HAL_StatusTypeDef status =
        MPU6050_ReadReg(
            dev,
            MPU6050_REG_CONFIG,
            &reg_value
        );

    if (status != HAL_OK)
    {
        return status;
    }

    /*
     * DLPF_CFG 位于 CONFIG[2:0]
     */
    reg_value &= (uint8_t)~0x07U;
    reg_value |= (uint8_t)(dlpf_cfg & 0x07U);

    return MPU6050_WriteReg(
        dev,
        MPU6050_REG_CONFIG,
        reg_value
    );
}


/* ============================================================
 * Sample Rate Divider
 * ============================================================ */

HAL_StatusTypeDef MPU6050_SetSampleRateDivider(
    MPU6050_Handle_t *dev,
    uint8_t divider
)
{
    if (dev == NULL)
    {
        return HAL_ERROR;
    }

    return MPU6050_WriteReg(
        dev,
        MPU6050_REG_SMPLRT_DIV,
        divider
    );
}


/* ============================================================
 * 初始化
 * ============================================================ */

HAL_StatusTypeDef MPU6050_Init(
    MPU6050_Handle_t *dev,
    I2C_HandleTypeDef *hi2c,
    uint16_t address
)
{
    if (dev == NULL ||
        hi2c == NULL)
    {
        return HAL_ERROR;
    }

    if (address != MPU6050_ADDR_AD0_LOW &&
        address != MPU6050_ADDR_AD0_HIGH)
    {
        return HAL_ERROR;
    }

    memset(dev, 0, sizeof(*dev));

    dev->hi2c = hi2c;
    dev->address = address;


    /* ========================================
     * 1. 检测设备
     * ======================================== */

    HAL_StatusTypeDef status =
        MPU6050_CheckDevice(dev);

    if (status != HAL_OK)
    {
        return status;
    }


    /* ========================================
     * 2. 唤醒 + 使用 X 轴陀螺仪 PLL
     *
     * PWR_MGMT_1:
     * SLEEP = 0
     * CLKSEL = 001
     * ======================================== */

    status =
        MPU6050_WriteReg(
            dev,
            MPU6050_REG_PWR_MGMT_1,
            0x01U
        );

    if (status != HAL_OK)
    {
        return status;
    }

    HAL_Delay(10U);


    /* ========================================
     * 3. 配置数字低通滤波
     * ======================================== */

    status =
        MPU6050_SetDLPF(
            dev,
            MPU6050_DEFAULT_DLPF_CFG
        );

    if (status != HAL_OK)
    {
        return status;
    }


    /* ========================================
     * 4. 配置采样率分频
     * ======================================== */

    status =
        MPU6050_SetSampleRateDivider(
            dev,
            MPU6050_DEFAULT_SMPLRT_DIV
        );

    if (status != HAL_OK)
    {
        return status;
    }


    /* ========================================
     * 5. 默认加速度 ±2g
     * ======================================== */

    status =
        MPU6050_SetAccelRange(
            dev,
            MPU6050_ACCEL_RANGE_2G
        );

    if (status != HAL_OK)
    {
        return status;
    }


    /* ========================================
     * 6. 默认陀螺仪 ±250 °/s
     * ======================================== */

    status =
        MPU6050_SetGyroRange(
            dev,
            MPU6050_GYRO_RANGE_250DPS
        );

    if (status != HAL_OK)
    {
        return status;
    }

    return HAL_OK;
}


/* ============================================================
 * 数据更新
 * ============================================================ */

HAL_StatusTypeDef MPU6050_Update(
    MPU6050_Handle_t *dev
)
{
    if (dev == NULL ||
        dev->hi2c == NULL ||
        dev->accel_sensitivity <= 0.0f ||
        dev->gyro_sensitivity <= 0.0f)
    {
        return HAL_ERROR;
    }

    /*
     * 从 ACCEL_XOUT_H 开始连续读取：
     *
     * 0  ACCEL_X_H
     * 1  ACCEL_X_L
     * 2  ACCEL_Y_H
     * 3  ACCEL_Y_L
     * 4  ACCEL_Z_H
     * 5  ACCEL_Z_L
     * 6  TEMP_H
     * 7  TEMP_L
     * 8  GYRO_X_H
     * 9  GYRO_X_L
     * 10 GYRO_Y_H
     * 11 GYRO_Y_L
     * 12 GYRO_Z_H
     * 13 GYRO_Z_L
     */
    uint8_t buffer[14];

    HAL_StatusTypeDef status =
        MPU6050_ReadRegs(
            dev,
            MPU6050_REG_ACCEL_XOUT_H,
            buffer,
            sizeof(buffer)
        );

    if (status != HAL_OK)
    {
        return status;
    }


    /* ========================================
     * 原始数据
     * ======================================== */

    dev->raw_accel_x =
        MPU6050_ReadInt16BE(&buffer[0]);

    dev->raw_accel_y =
        MPU6050_ReadInt16BE(&buffer[2]);

    dev->raw_accel_z =
        MPU6050_ReadInt16BE(&buffer[4]);

    dev->raw_temp =
        MPU6050_ReadInt16BE(&buffer[6]);

    dev->raw_gyro_x =
        MPU6050_ReadInt16BE(&buffer[8]);

    dev->raw_gyro_y =
        MPU6050_ReadInt16BE(&buffer[10]);

    dev->raw_gyro_z =
        MPU6050_ReadInt16BE(&buffer[12]);


    /* ========================================
     * 加速度，单位 g
     * ======================================== */

    dev->accel_x =
        (float)dev->raw_accel_x /
        dev->accel_sensitivity;

    dev->accel_y =
        (float)dev->raw_accel_y /
        dev->accel_sensitivity;

    dev->accel_z =
        (float)dev->raw_accel_z /
        dev->accel_sensitivity;


    /* ========================================
     * 温度，单位 °C
     *
     * Temp = raw / 340 + 36.53
     * ======================================== */

    dev->temperature =
        (float)dev->raw_temp /
        340.0f +
        36.53f;


    /* ========================================
     * 陀螺仪，单位 °/s
     * ======================================== */

    dev->gyro_x =
        (float)dev->raw_gyro_x /
        dev->gyro_sensitivity;

    dev->gyro_y =
        (float)dev->raw_gyro_y /
        dev->gyro_sensitivity;

    dev->gyro_z =
        (float)dev->raw_gyro_z /
        dev->gyro_sensitivity;

    return HAL_OK;
}