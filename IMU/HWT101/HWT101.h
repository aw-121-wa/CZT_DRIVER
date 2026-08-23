#ifndef HWT101_H
#define HWT101_H

#include "uart.h"
#include <stdint.h>

#define HWT101_FRAME_SIZE       11U
#define HWT101_RX_BUFFER_SIZE   128U

/* HWT101 波特率协议值 */
typedef enum
{
    HWT101_BAUD_2400   = 0x00,
    HWT101_BAUD_4800   = 0x01,
    HWT101_BAUD_9600   = 0x02,
    HWT101_BAUD_19200  = 0x03,
    HWT101_BAUD_38400  = 0x04,
    HWT101_BAUD_57600  = 0x05,
    HWT101_BAUD_115200 = 0x06,
    HWT101_BAUD_230400 = 0x07,
    HWT101_BAUD_460800 = 0x08,
    HWT101_BAUD_921600 = 0x09
} HWT101_Baud_t;


/* IMU 数据 */
typedef struct
{
    float roll;
    float pitch;
    float yaw;

    float gyro_x;
    float gyro_y;
    float gyro_z;

} HWT101_Data_t;


/* HWT101 设备对象 */
typedef struct
{
    /* 使用哪个 UART */
    UART_HandleTypeDef *huart;

    /* 解析后的数据 */
    volatile HWT101_Data_t data;

    /* DMA 接收区 */
    uint8_t rx_buffer[HWT101_RX_BUFFER_SIZE];

    /* 自动搜帧使用 */
    uint8_t frame_buffer[HWT101_FRAME_SIZE];
    uint8_t frame_index;

    /* 调试统计 */
    volatile uint32_t valid_frame_count;
    volatile uint32_t checksum_error_count;

} HWT101_Handle_t;


/* 初始化 */
HAL_StatusTypeDef HWT101_Init(
    HWT101_Handle_t *dev,
    UART_HandleTypeDef *huart
);

/* 启动 DMA + IDLE 接收 */
HAL_StatusTypeDef HWT101_StartReceive(
    HWT101_Handle_t *dev
);

/* HAL UART ReceiveToIdle 回调入口 */
void HWT101_RxEventCallback(
    HWT101_Handle_t *dev,
    UART_HandleTypeDef *huart,
    uint16_t size
);

/* UART 错误恢复 */
void HWT101_ErrorCallback(
    HWT101_Handle_t *dev,
    UART_HandleTypeDef *huart
);

/* 纯协议解析，可单独测试 */
void HWT101_ProcessBytes(
    HWT101_Handle_t *dev,
    const uint8_t *data,
    uint16_t len
);

/* 修改 HWT101 + STM32 UART 波特率 */
HAL_StatusTypeDef HWT101_SetBaudrate(
    HWT101_Handle_t *dev,
    HWT101_Baud_t baud
);

#endif