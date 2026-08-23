#include "by8001.h"


#define BY8001_HEADER       0x7EU
#define BY8001_END          0xEFU

#define BY8001_CMD_PLAY     0x41U
#define BY8001_CMD_VOLUME   0x06U

#define BY8001_PLAY_LEN     0x05U
#define BY8001_VOLUME_LEN   0x03U

#define BY8001_TIMEOUT_MS   50U


/*
 * XOR校验
 *
 * checksum =
 * LEN ^ CMD ^ PARAM1 ^ PARAM2 ...
 */
/*计算BY8001协议数据的XOR校验和*/
static uint8_t BY8001_Checksum(
    uint8_t length,
    const uint8_t *data
)
{
    uint8_t checksum = length;

    /*
     * length - 1
     * 表示 CMD + 参数的数量
     */
    for (uint8_t i = 0;
         i < length - 1U;
         i++)
    {
        checksum ^= data[i];
    }

    return checksum;
}


/*
 * UART发送
 */
/*通过UART发送数据到BY8001模块*/
static HAL_StatusTypeDef BY8001_Send(
    BY8001_Handle_t *dev,
    uint8_t *data,
    uint16_t len
)
{
    if (dev == NULL ||
        dev->huart == NULL ||
        data == NULL)
    {
        return HAL_ERROR;
    }

    return HAL_UART_Transmit(
        dev->huart,
        data,
        len,
        dev->timeout_ms
    );
}


/*初始化BY8001模块，配置UART和超时时间*/
void BY8001_Init(
    BY8001_Handle_t *dev,
    UART_HandleTypeDef *huart
)
{
    if (dev == NULL)
        return;

    dev->huart = huart;
    dev->timeout_ms = BY8001_TIMEOUT_MS;
}


/*
 * 播放指定曲目
 *
 * 7E 05 41 XX XX CHECK EF
 */
/*播放指定编号的音频曲目*/
HAL_StatusTypeDef BY8001_Play(
    BY8001_Handle_t *dev,
    uint16_t index
)
{
    if (dev == NULL || index == 0U)
        return HAL_ERROR;

    uint8_t frame[7];

    uint8_t cmd[3];

    cmd[0] = BY8001_CMD_PLAY;

    /*
     * 曲目编号：
     * 高字节在前
     * 低字节在后
     */
    cmd[1] =
        (uint8_t)(index >> 8);

    cmd[2] =
        (uint8_t)(index & 0xFFU);


    frame[0] = BY8001_HEADER;

    frame[1] = BY8001_PLAY_LEN;

    frame[2] = cmd[0];

    frame[3] = cmd[1];

    frame[4] = cmd[2];

    frame[5] =
        BY8001_Checksum(
            BY8001_PLAY_LEN,
            cmd
        );

    frame[6] = BY8001_END;


    return BY8001_Send(
        dev,
        frame,
        sizeof(frame)
    );
}


/*
 * 设置音量
 *
 * 7E 03 06 VOL CHECK EF
 */
/*设置BY8001模块的音量级别*/
HAL_StatusTypeDef BY8001_SetVolume(
    BY8001_Handle_t *dev,
    uint8_t volume
)
{
    if (dev == NULL)
        return HAL_ERROR;

    if (volume > BY8001_VOLUME_MAX)
    {
        volume = BY8001_VOLUME_MAX;
    }


    uint8_t frame[6];

    uint8_t cmd[2];

    cmd[0] = BY8001_CMD_VOLUME;

    cmd[1] = volume;


    frame[0] = BY8001_HEADER;

    frame[1] = BY8001_VOLUME_LEN;

    frame[2] = cmd[0];

    frame[3] = cmd[1];

    frame[4] =
        BY8001_Checksum(
            BY8001_VOLUME_LEN,
            cmd
        );

    frame[5] = BY8001_END;


    return BY8001_Send(
        dev,
        frame,
        sizeof(frame)
    );
}