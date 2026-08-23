#include "HWT101.h"

#include <string.h>


/* =========================================================
 * HWT101 协议定义
 * ========================================================= */

#define HWT101_FRAME_HEAD       0x55

#define HWT101_FRAME_GYRO       0x52
#define HWT101_FRAME_ANGLE      0x53

#define HWT101_CMD_HEAD_1       0xFF
#define HWT101_CMD_HEAD_2       0xAA

#define HWT101_REG_SAVE         0x00
#define HWT101_REG_BAUD         0x04
#define HWT101_REG_UNLOCK       0x69

#define HWT101_UNLOCK_VALUE     0xB588


/* =========================================================
 * 内部工具函数
 * ========================================================= */

/**
 * @brief 读取小端 int16_t
 *
 * HWT101：
 * data[0] = Low
 * data[1] = High
 */
static int16_t HWT101_ReadInt16LE(const uint8_t *data)
{
    return (int16_t)(
        ((uint16_t)data[1] << 8) |
        ((uint16_t)data[0])
    );
}


/**
 * @brief 校验 HWT101 11 字节数据帧
 */
static uint8_t HWT101_Checksum(const uint8_t *frame)
{
    uint8_t sum = 0;

    for (uint8_t i = 0; i < 10; i++)
    {
        sum += frame[i];
    }

    return (sum == frame[10]);
}


/**
 * @brief 判断是否可能是一个 HWT101 数据帧类型
 *
 * HWT101 常见输出：
 * 0x50 ~ 0x5A
 */
static uint8_t HWT101_IsValidFrameType(uint8_t type)
{
    if (type >= 0x50 && type <= 0x5A)
    {
        return 1;
    }

    return 0;
}


/**
 * @brief 解析完整 11 字节帧
 */
static void HWT101_ParseFrame(
    HWT101_Handle_t *dev,
    const uint8_t *frame
)
{
    int16_t raw_x;
    int16_t raw_y;
    int16_t raw_z;

    raw_x = HWT101_ReadInt16LE(&frame[2]);
    raw_y = HWT101_ReadInt16LE(&frame[4]);
    raw_z = HWT101_ReadInt16LE(&frame[6]);

    switch (frame[1])
    {
        /* -----------------------------
         * 角速度
         * ----------------------------- */
        case HWT101_FRAME_GYRO:

            dev->data.gyro_x =
                (float)raw_x / 32768.0f * 2000.0f;

            dev->data.gyro_y =
                (float)raw_y / 32768.0f * 2000.0f;

            dev->data.gyro_z =
                (float)raw_z / 32768.0f * 2000.0f;

            break;


        /* -----------------------------
         * 欧拉角
         * ----------------------------- */
        case HWT101_FRAME_ANGLE:

            dev->data.roll =
                (float)raw_x / 32768.0f * 180.0f;

            dev->data.pitch =
                (float)raw_y / 32768.0f * 180.0f;

            dev->data.yaw =
                (float)raw_z / 32768.0f * 180.0f;

            break;


        default:
            /*
             * 其它合法 HWT101 帧：
             * 0x51 加速度
             * 0x54 磁场
             * ...
             *
             * 当前不需要，直接忽略。
             */
            break;
    }
}


/**
 * @brief 校验失败后寻找当前缓存中下一个 0x55
 */
static void HWT101_SearchNextHeader(HWT101_Handle_t *dev)
{
    uint8_t i;

    for (i = 1; i < HWT101_FRAME_SIZE; i++)
    {
        if (dev->frame_buffer[i] == HWT101_FRAME_HEAD)
        {
            uint8_t remain =
                HWT101_FRAME_SIZE - i;

            memmove(
                dev->frame_buffer,
                &dev->frame_buffer[i],
                remain
            );

            dev->frame_index = remain;

            return;
        }
    }

    dev->frame_index = 0;
}


/* =========================================================
 * 自动搜帧
 * ========================================================= */

/*处理接收到的字节数据，自动搜帧并解析HWT101数据帧*/
void HWT101_ProcessBytes(
    HWT101_Handle_t *dev,
    const uint8_t *data,
    uint16_t len
)
{
    if (dev == NULL || data == NULL)
    {
        return;
    }

    for (uint16_t i = 0; i < len; i++)
    {
        uint8_t byte = data[i];


        /* =============================================
         * 状态 0：
         * 等待帧头 0x55
         * ============================================= */
        if (dev->frame_index == 0)
        {
            if (byte == HWT101_FRAME_HEAD)
            {
                dev->frame_buffer[0] = byte;
                dev->frame_index = 1;
            }

            continue;
        }


        /* =============================================
         * 已经找到帧头
         * 继续存数据
         * ============================================= */

        dev->frame_buffer[dev->frame_index++] = byte;


        /* =============================================
         * 第二个字节必须是合理的数据类型
         * ============================================= */

        if (dev->frame_index == 2)
        {
            if (!HWT101_IsValidFrameType(
                    dev->frame_buffer[1]))
            {
                /*
                 * 如果当前 byte 又是 0x55，
                 * 那它可能正好是新的帧头。
                 */
                if (byte == HWT101_FRAME_HEAD)
                {
                    dev->frame_buffer[0] =
                        HWT101_FRAME_HEAD;

                    dev->frame_index = 1;
                }
                else
                {
                    dev->frame_index = 0;
                }

                continue;
            }
        }


        /* =============================================
         * 收满 11 字节
         * ============================================= */

        if (dev->frame_index >= HWT101_FRAME_SIZE)
        {
            if (HWT101_Checksum(dev->frame_buffer))
            {
                HWT101_ParseFrame(
                    dev,
                    dev->frame_buffer
                );

                dev->valid_frame_count++;

                /*
                 * 一个完整帧解析完毕，
                 * 重新等待下一个 0x55
                 */
                dev->frame_index = 0;
            }
            else
            {
                dev->checksum_error_count++;

                /*
                 * 校验失败不能简单全部丢掉。
                 *
                 * 在这11字节里面继续寻找下一个0x55，
                 * 实现快速重新同步。
                 */
                HWT101_SearchNextHeader(dev);
            }
        }
    }
}


/* =========================================================
 * DMA 接收
 * ========================================================= */

/*启动DMA接收，开始接收HWT101数据*/
HAL_StatusTypeDef HWT101_StartReceive(
    HWT101_Handle_t *dev
)
{
    if (dev == NULL ||
        dev->huart == NULL ||
        dev->huart->hdmarx == NULL)
    {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status;

    status = HAL_UARTEx_ReceiveToIdle_DMA(
        dev->huart,
        dev->rx_buffer,
        HWT101_RX_BUFFER_SIZE
    );

    if (status == HAL_OK)
    {
        /*
         * 不需要 DMA Half Transfer 中断，
         * 减少一次无意义回调。
         */
        __HAL_DMA_DISABLE_IT(
            dev->huart->hdmarx,
            DMA_IT_HT
        );
    }

    return status;
}


/* =========================================================
 * 初始化
 * ========================================================= */

/*初始化HWT101设备，配置UART并启动DMA接收*/
HAL_StatusTypeDef HWT101_Init(
    HWT101_Handle_t *dev,
    UART_HandleTypeDef *huart
)
{
    if (dev == NULL || huart == NULL)
    {
        return HAL_ERROR;
    }

    memset(dev, 0, sizeof(HWT101_Handle_t));

    dev->huart = huart;

    return HWT101_StartReceive(dev);
}


/* =========================================================
 * UART ReceiveToIdle 回调
 * ========================================================= */

/*UART接收空闲回调函数，处理接收到的数据并重新启动DMA接收*/
void HWT101_RxEventCallback(
    HWT101_Handle_t *dev,
    UART_HandleTypeDef *huart,
    uint16_t size
)
{
    if (dev == NULL ||
        huart == NULL ||
        huart != dev->huart)
    {
        return;
    }

    if (size > HWT101_RX_BUFFER_SIZE)
    {
        size = HWT101_RX_BUFFER_SIZE;
    }

    /*
     * DMA只负责收。
     *
     * 协议解析全部交给 ProcessBytes。
     */
    HWT101_ProcessBytes(
        dev,
        dev->rx_buffer,
        size
    );

    /*
     * Normal DMA模式下重新启动下一轮。
     */
    HWT101_StartReceive(dev);
}


/* =========================================================
 * UART 错误恢复
 * ========================================================= */

/*UART错误回调函数，中止接收并重新启动DMA接收*/
void HWT101_ErrorCallback(
    HWT101_Handle_t *dev,
    UART_HandleTypeDef *huart
)
{
    if (dev == NULL ||
        huart == NULL ||
        huart != dev->huart)
    {
        return;
    }

    HAL_UART_AbortReceive(dev->huart);

    dev->frame_index = 0;

    HWT101_StartReceive(dev);
}


/* =========================================================
 * HWT101 配置命令
 * ========================================================= */

/**
 * @brief 写 HWT101 寄存器
 *
 * 协议：
 *
 * FF AA REG DATA_L DATA_H
 */
static HAL_StatusTypeDef HWT101_WriteRegister(
    HWT101_Handle_t *dev,
    uint8_t reg,
    uint16_t value
)
{
    uint8_t cmd[5];

    cmd[0] = HWT101_CMD_HEAD_1;
    cmd[1] = HWT101_CMD_HEAD_2;
    cmd[2] = reg;
    cmd[3] = (uint8_t)(value & 0xFF);
    cmd[4] = (uint8_t)(value >> 8);

    return HAL_UART_Transmit(
        dev->huart,
        cmd,
        sizeof(cmd),
        100
    );
}


/**
 * @brief HWT101协议波特率值 -> 实际 UART baudrate
 */
static uint32_t HWT101_GetBaudValue(
    HWT101_Baud_t baud
)
{
    switch (baud)
    {
        case HWT101_BAUD_2400:
            return 2400;

        case HWT101_BAUD_4800:
            return 4800;

        case HWT101_BAUD_9600:
            return 9600;

        case HWT101_BAUD_19200:
            return 19200;

        case HWT101_BAUD_38400:
            return 38400;

        case HWT101_BAUD_57600:
            return 57600;

        case HWT101_BAUD_115200:
            return 115200;

        case HWT101_BAUD_230400:
            return 230400;

        case HWT101_BAUD_460800:
            return 460800;

        case HWT101_BAUD_921600:
            return 921600;

        default:
            return 0;
    }
}


/* =========================================================
 * 修改 HWT101 波特率
 * ========================================================= */

/*设置HWT101设备的波特率，包括解锁寄存器、修改波特率、保存配置等步骤*/
HAL_StatusTypeDef HWT101_SetBaudrate(
    HWT101_Handle_t *dev,
    HWT101_Baud_t baud
)
{
    if (dev == NULL || dev->huart == NULL)
    {
        return HAL_ERROR;
    }

    uint32_t new_baud =
        HWT101_GetBaudValue(baud);

    if (new_baud == 0)
    {
        return HAL_ERROR;
    }


    /*
     * 修改波特率时暂停 DMA。
     */
    HAL_UART_DMAStop(dev->huart);


    /* =====================================
     * 1. 解锁寄存器
     *
     * FF AA 69 88 B5
     * ===================================== */

    if (HWT101_WriteRegister(
            dev,
            HWT101_REG_UNLOCK,
            HWT101_UNLOCK_VALUE) != HAL_OK)
    {
        return HAL_ERROR;
    }

    HAL_Delay(200);


    /* =====================================
     * 2. 修改 HWT101 波特率
     *
     * FF AA 04 BAUD 00
     * ===================================== */

    if (HWT101_WriteRegister(
            dev,
            HWT101_REG_BAUD,
            (uint16_t)baud) != HAL_OK)
    {
        return HAL_ERROR;
    }

    HAL_Delay(200);


    /*
     * HWT101 此时已经切换到新波特率。
     *
     * STM32也必须同步切换。
     */


    /* =====================================
     * 3. 修改 STM32 UART 波特率
     * ===================================== */

    if (HAL_UART_DeInit(dev->huart) != HAL_OK)
    {
        return HAL_ERROR;
    }

    dev->huart->Init.BaudRate = new_baud;

    if (HAL_UART_Init(dev->huart) != HAL_OK)
    {
        return HAL_ERROR;
    }

    HAL_Delay(100);


    /* =====================================
     * 4. 新波特率下重新解锁
     * ===================================== */

    if (HWT101_WriteRegister(
            dev,
            HWT101_REG_UNLOCK,
            HWT101_UNLOCK_VALUE) != HAL_OK)
    {
        return HAL_ERROR;
    }

    HAL_Delay(200);


    /* =====================================
     * 5. 保存
     *
     * FF AA 00 00 00
     * ===================================== */

    if (HWT101_WriteRegister(
            dev,
            HWT101_REG_SAVE,
            0x0000) != HAL_OK)
    {
        return HAL_ERROR;
    }

    HAL_Delay(200);


    /*
     * 清解析状态。
     */
    dev->frame_index = 0;


    /*
     * 恢复DMA。
     */
    return HWT101_StartReceive(dev);
}