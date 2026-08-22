#ifndef BY8001_H
#define BY8001_H

#include "main.h"
#include <stdint.h>

#define BY8001_VOLUME_MAX 30U

typedef struct
{
    UART_HandleTypeDef *huart;
    uint32_t timeout_ms;

} BY8001_Handle_t;


/* 初始化 */
void BY8001_Init(
    BY8001_Handle_t *dev,
    UART_HandleTypeDef *huart
);


/* 播放指定曲目 */
HAL_StatusTypeDef BY8001_Play(
    BY8001_Handle_t *dev,
    uint16_t index
);


/* 设置音量 0~30 */
HAL_StatusTypeDef BY8001_SetVolume(
    BY8001_Handle_t *dev,
    uint8_t volume
);

#endif