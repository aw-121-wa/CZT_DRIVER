#ifndef LSC16_H
#define LSC16_H

#include "main.h"
#include <stdint.h>

/* ===================== 基本参数 ===================== */

#define LSC16_BAUDRATE             9600U
#define LSC16_MAX_SERVOS           16U
#define LSC16_POSITION_MIN         500U
#define LSC16_POSITION_MAX         2500U
#define LSC16_MOVE_TIME_MAX_MS     30000U

#define LSC16_ACTION_REPEAT_FOREVER 0U
#define LSC16_ACTION_ALL            0xFFU

#define LSC16_RX_DMA_BUFFER_SIZE   32U
#define LSC16_MAX_FRAME_SIZE       64U


/* ===================== 舵机数据 ===================== */

typedef struct
{
    uint8_t  id;
    uint16_t position;
} LSC16_Servo_t;


/* ===================== LSC16 设备对象 ===================== */

typedef struct
{
    UART_HandleTypeDef *huart;

    /* DMA 接收缓存 */
    uint8_t rx_dma_buffer[LSC16_RX_DMA_BUFFER_SIZE];

    /* 自动搜帧缓存 */
    uint8_t frame_buffer[LSC16_MAX_FRAME_SIZE];
    uint8_t frame_index;
    uint8_t expected_frame_size;

    /* 控制器状态 */
    volatile uint8_t  action_running;
    volatile uint8_t  action_done;
    volatile uint8_t  current_action_group;
    volatile uint16_t current_action_times;
    volatile uint16_t battery_mv;

    /* 调试统计 */
    volatile uint8_t  last_rx_cmd;
    volatile uint32_t rx_frame_count;
    volatile uint32_t rx_error_count;

} LSC16_Handle_t;


/* ===================== 初始化 / 接收 ===================== */

HAL_StatusTypeDef LSC16_Init(
    LSC16_Handle_t *dev,
    UART_HandleTypeDef *huart
);

HAL_StatusTypeDef LSC16_StartReceiveDMA(
    LSC16_Handle_t *dev
);

void LSC16_RxEventCallback(
    LSC16_Handle_t *dev,
    UART_HandleTypeDef *huart,
    uint16_t size
);

void LSC16_ErrorCallback(
    LSC16_Handle_t *dev,
    UART_HandleTypeDef *huart
);

/* 可脱离 DMA 单独喂入任意字节流，内部自动搜帧 */
void LSC16_ProcessRxBytes(
    LSC16_Handle_t *dev,
    const uint8_t *data,
    uint16_t len
);


/* ===================== PWM 舵机控制 ===================== */

HAL_StatusTypeDef LSC16_MoveServo(
    LSC16_Handle_t *dev,
    uint8_t id,
    uint16_t position,
    uint16_t time_ms
);

HAL_StatusTypeDef LSC16_MoveServos(
    LSC16_Handle_t *dev,
    const LSC16_Servo_t *servos,
    uint8_t count,
    uint16_t time_ms
);


/* ===================== 动作组控制 ===================== */

HAL_StatusTypeDef LSC16_RunActionGroup(
    LSC16_Handle_t *dev,
    uint8_t group,
    uint16_t times
);

HAL_StatusTypeDef LSC16_StopActionGroup(
    LSC16_Handle_t *dev
);

HAL_StatusTypeDef LSC16_SetActionGroupSpeed(
    LSC16_Handle_t *dev,
    uint8_t group,
    uint16_t percent
);


/* ===================== 查询 ===================== */

/* 异步发送查询；收到回复后 dev->battery_mv 自动更新 */
HAL_StatusTypeDef LSC16_RequestBatteryVoltage(
    LSC16_Handle_t *dev
);


/* ===================== 状态辅助 ===================== */

static inline uint8_t LSC16_IsActionRunning(const LSC16_Handle_t *dev)
{
    return (dev != NULL) ? dev->action_running : 0U;
}

static inline uint8_t LSC16_IsActionDone(const LSC16_Handle_t *dev)
{
    return (dev != NULL) ? dev->action_done : 0U;
}

static inline void LSC16_ClearActionDone(LSC16_Handle_t *dev)
{
    if (dev != NULL)
        dev->action_done = 0U;
}

#endif /* LSC16_H */