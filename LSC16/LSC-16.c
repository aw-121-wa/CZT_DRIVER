#include "lsc16.h"

#include <string.h>

/* ===================== 协议定义 ===================== */

#define LSC16_HEADER                  0x55U

#define LSC16_CMD_SERVO_MOVE         0x03U
#define LSC16_CMD_ACTION_GROUP_RUN   0x06U
#define LSC16_CMD_ACTION_GROUP_STOP  0x07U
#define LSC16_CMD_ACTION_GROUP_DONE  0x08U
#define LSC16_CMD_ACTION_GROUP_SPEED 0x0BU
#define LSC16_CMD_BATTERY_VOLTAGE    0x0FU

#define LSC16_TX_TIMEOUT_MS           100U


/* ===================== 内部工具函数 ===================== */

/*读取小端格式的16位无符号整数*/
static uint16_t lsc16_read_u16_le(const uint8_t *p)
{
    return (uint16_t)p[0] |
           ((uint16_t)p[1] << 8);
}


/*通过UART发送数据到LSC-16控制器*/
static HAL_StatusTypeDef lsc16_send(
    LSC16_Handle_t *dev,
    const uint8_t *data,
    uint16_t len
)
{
    if (dev == NULL ||
        dev->huart == NULL ||
        data == NULL ||
        len == 0U)
    {
        return HAL_ERROR;
    }

    return HAL_UART_Transmit(
        dev->huart,
        (uint8_t *)data,
        len,
        LSC16_TX_TIMEOUT_MS
    );
}


/*
 * LSC 协议：
 *
 * 55 55 Length Cmd Parameters...
 *
 * Length = 参数数量 + Cmd(1) + Length字段自身(1)
 *
 * 所以：
 * 总帧字节数 = Length + 2个帧头
 */
/*解析LSC-16协议帧，处理动作组状态和电池电压等响应*/
static void lsc16_parse_frame(
    LSC16_Handle_t *dev,
    const uint8_t *frame,
    uint8_t frame_size
)
{
    if (dev == NULL ||
        frame == NULL ||
        frame_size < 4U)
    {
        return;
    }

    uint8_t length = frame[2];
    uint8_t cmd    = frame[3];

    if ((uint8_t)(length + 2U) != frame_size)
    {
        dev->rx_error_count++;
        return;
    }

    dev->last_rx_cmd = cmd;
    dev->rx_frame_count++;

    switch (cmd)
    {
        /* 控制器开始执行动作组时会回传 0x06 */
        case LSC16_CMD_ACTION_GROUP_RUN:
        {
            if (length == 5U)
            {
                dev->current_action_group = frame[4];
                dev->current_action_times =
                    lsc16_read_u16_le(&frame[5]);

                dev->action_running = 1U;
                dev->action_done    = 0U;
            }
            break;
        }


        /* 动作组被强制停止 */
        case LSC16_CMD_ACTION_GROUP_STOP:
        {
            if (length == 2U)
            {
                dev->action_running = 0U;
                dev->action_done    = 0U;
            }
            break;
        }


        /* 动作组自然执行完成 */
        case LSC16_CMD_ACTION_GROUP_DONE:
        {
            if (length == 5U)
            {
                dev->current_action_group = frame[4];
                dev->current_action_times =
                    lsc16_read_u16_le(&frame[5]);

                dev->action_running = 0U;
                dev->action_done    = 1U;
            }
            break;
        }


        /* 电池电压回复，单位 mV */
        case LSC16_CMD_BATTERY_VOLTAGE:
        {
            if (length == 4U)
            {
                dev->battery_mv =
                    lsc16_read_u16_le(&frame[4]);
            }
            break;
        }


        default:
            break;
    }
}


/* ===================== 自动搜帧 ===================== */

/*处理接收到的字节数据，自动搜帧并解析LSC-16协议*/
void LSC16_ProcessRxBytes(
    LSC16_Handle_t *dev,
    const uint8_t *data,
    uint16_t len
)
{
    if (dev == NULL || data == NULL)
        return;

    for (uint16_t i = 0U; i < len; i++)
    {
        uint8_t byte = data[i];

        /*
         * 状态0：
         * 等第一个 0x55
         */
        if (dev->frame_index == 0U)
        {
            if (byte == LSC16_HEADER)
            {
                dev->frame_buffer[0] = byte;
                dev->frame_index = 1U;
            }

            continue;
        }


        /*
         * 状态1：
         * 等第二个 0x55
         */
        if (dev->frame_index == 1U)
        {
            if (byte == LSC16_HEADER)
            {
                dev->frame_buffer[1] = byte;
                dev->frame_index = 2U;
            }
            else
            {
                dev->frame_index = 0U;
            }

            continue;
        }


        /*
         * 后续字节写入帧缓存
         */
        if (dev->frame_index >= LSC16_MAX_FRAME_SIZE)
        {
            dev->frame_index = 0U;
            dev->expected_frame_size = 0U;
            dev->rx_error_count++;
            continue;
        }

        dev->frame_buffer[dev->frame_index++] = byte;


        /*
         * 收到 Length 字段
         */
        if (dev->frame_index == 3U)
        {
            uint8_t length = dev->frame_buffer[2];

            /*
             * 最短合法帧：
             * 55 55 02 Cmd
             *
             * 总长度 = Length + 2
             */
            uint16_t total_size =
                (uint16_t)length + 2U;

            if (length < 2U ||
                total_size > LSC16_MAX_FRAME_SIZE)
            {
                dev->frame_index = 0U;
                dev->expected_frame_size = 0U;
                dev->rx_error_count++;
                continue;
            }

            dev->expected_frame_size =
                (uint8_t)total_size;
        }


        /*
         * 收满一帧
         */
        if (dev->expected_frame_size != 0U &&
            dev->frame_index >= dev->expected_frame_size)
        {
            lsc16_parse_frame(
                dev,
                dev->frame_buffer,
                dev->expected_frame_size
            );

            dev->frame_index = 0U;
            dev->expected_frame_size = 0U;
        }
    }
}


/* ===================== DMA 接收 ===================== */

/*启动DMA接收，开始接收LSC-16控制器数据*/
HAL_StatusTypeDef LSC16_StartReceiveDMA(
    LSC16_Handle_t *dev
)
{
    if (dev == NULL ||
        dev->huart == NULL ||
        dev->huart->hdmarx == NULL)
    {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status =
        HAL_UARTEx_ReceiveToIdle_DMA(
            dev->huart,
            dev->rx_dma_buffer,
            LSC16_RX_DMA_BUFFER_SIZE
        );

    if (status == HAL_OK)
    {
        /*
         * 只需要 IDLE / DMA完成事件，
         * 关闭 Half Transfer 回调降低干扰。
         */
        __HAL_DMA_DISABLE_IT(
            dev->huart->hdmarx,
            DMA_IT_HT
        );
    }

    return status;
}


/*UART接收空闲回调函数，处理接收到的数据并重新启动DMA接收*/
void LSC16_RxEventCallback(
    LSC16_Handle_t *dev,
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

    if (size > LSC16_RX_DMA_BUFFER_SIZE)
        size = LSC16_RX_DMA_BUFFER_SIZE;

    LSC16_ProcessRxBytes(
        dev,
        dev->rx_dma_buffer,
        size
    );

    /*
     * 默认按 DMA Normal 模式设计。
     */
    (void)LSC16_StartReceiveDMA(dev);
}


/*UART错误回调函数，中止接收并重新启动DMA接收*/
void LSC16_ErrorCallback(
    LSC16_Handle_t *dev,
    UART_HandleTypeDef *huart
)
{
    if (dev == NULL ||
        huart == NULL ||
        huart != dev->huart)
    {
        return;
    }

    (void)HAL_UART_AbortReceive(dev->huart);

    dev->frame_index = 0U;
    dev->expected_frame_size = 0U;

    (void)LSC16_StartReceiveDMA(dev);
}


/* ===================== 初始化 ===================== */

/*初始化LSC-16设备，配置UART并启动DMA接收*/
HAL_StatusTypeDef LSC16_Init(
    LSC16_Handle_t *dev,
    UART_HandleTypeDef *huart
)
{
    if (dev == NULL || huart == NULL)
        return HAL_ERROR;

    memset(dev, 0, sizeof(*dev));

    dev->huart = huart;

    /*
     * LSC-16 官方串口协议使用 9600 baud。
     * 这里不偷偷修改 CubeMX 配置，而是直接检查。
     */
    if (huart->Init.BaudRate != LSC16_BAUDRATE)
        return HAL_ERROR;

    /*
     * RX DMA不是发送命令的必要条件。
     * 如果配置了 RX DMA，则自动启动状态接收。
     */
    if (huart->hdmarx != NULL)
        return LSC16_StartReceiveDMA(dev);

    return HAL_OK;
}


/* ===================== PWM 舵机控制 ===================== */

/*控制多个PWM舵机移动到指定位置*/
HAL_StatusTypeDef LSC16_MoveServos(
    LSC16_Handle_t *dev,
    const LSC16_Servo_t *servos,
    uint8_t count,
    uint16_t time_ms
)
{
    if (dev == NULL ||
        servos == NULL ||
        count == 0U ||
        count > LSC16_MAX_SERVOS ||
        time_ms > LSC16_MOVE_TIME_MAX_MS)
    {
        return HAL_ERROR;
    }

    /*
     * 最大16个PWM舵机：
     *
     * 总帧长度 =
     * 2(header)
     * + 1(length)
     * + 1(cmd)
     * + 1(count)
     * + 2(time)
     * + count * 3
     *
     * count=16 -> 55 bytes
     */
    uint8_t frame[LSC16_MAX_FRAME_SIZE];

    uint16_t index = 0U;

    frame[index++] = 0x55U;
    frame[index++] = 0x55U;

    frame[index++] =
        (uint8_t)(count * 3U + 5U);

    frame[index++] = LSC16_CMD_SERVO_MOVE;

    frame[index++] = count;

    frame[index++] =
        (uint8_t)(time_ms & 0xFFU);

    frame[index++] =
        (uint8_t)(time_ms >> 8U);

    for (uint8_t i = 0U; i < count; i++)
    {
        /*
         * 官方协议ID字段是1字节。
         * LSC-16实际只接16路舵机，
         * 这里仍保留协议级 1~254 合法范围。
         */
        if (servos[i].id < 1U ||
            servos[i].id > 254U)
        {
            return HAL_ERROR;
        }

        /*
         * PWM舵机官方推荐位置：
         * 500~2500
         */
        if (servos[i].position < LSC16_POSITION_MIN ||
            servos[i].position > LSC16_POSITION_MAX)
        {
            return HAL_ERROR;
        }

        frame[index++] = servos[i].id;

        frame[index++] =
            (uint8_t)(servos[i].position & 0xFFU);

        frame[index++] =
            (uint8_t)(servos[i].position >> 8U);
    }

    return lsc16_send(
        dev,
        frame,
        index
    );
}


/*控制单个PWM舵机移动到指定位置*/
HAL_StatusTypeDef LSC16_MoveServo(
    LSC16_Handle_t *dev,
    uint8_t id,
    uint16_t position,
    uint16_t time_ms
)
{
    LSC16_Servo_t servo;

    servo.id       = id;
    servo.position = position;

    return LSC16_MoveServos(
        dev,
        &servo,
        1U,
        time_ms
    );
}


/* ===================== 动作组 ===================== */

/*运行指定动作组，可设置执行次数*/
HAL_StatusTypeDef LSC16_RunActionGroup(
    LSC16_Handle_t *dev,
    uint8_t group,
    uint16_t times
)
{
    uint8_t frame[7];

    frame[0] = 0x55U;
    frame[1] = 0x55U;
    frame[2] = 0x05U;
    frame[3] = LSC16_CMD_ACTION_GROUP_RUN;

    frame[4] = group;

    frame[5] =
        (uint8_t)(times & 0xFFU);

    frame[6] =
        (uint8_t)(times >> 8U);

    /*
     * 发送时先清除旧完成标志。
     * 真正 action_running=1 由控制器的0x06回包确认。
     */
    if (dev != NULL)
        dev->action_done = 0U;

    return lsc16_send(
        dev,
        frame,
        sizeof(frame)
    );
}


/*停止当前正在执行的动作组*/
HAL_StatusTypeDef LSC16_StopActionGroup(
    LSC16_Handle_t *dev
)
{
    const uint8_t frame[4] =
    {
        0x55U,
        0x55U,
        0x02U,
        LSC16_CMD_ACTION_GROUP_STOP
    };

    return lsc16_send(
        dev,
        frame,
        sizeof(frame)
    );
}


/*设置指定动作组的执行速度百分比*/
HAL_StatusTypeDef LSC16_SetActionGroupSpeed(
    LSC16_Handle_t *dev,
    uint8_t group,
    uint16_t percent
)
{
    uint8_t frame[7];

    frame[0] = 0x55U;
    frame[1] = 0x55U;
    frame[2] = 0x05U;
    frame[3] = LSC16_CMD_ACTION_GROUP_SPEED;

    frame[4] = group;

    frame[5] =
        (uint8_t)(percent & 0xFFU);

    frame[6] =
        (uint8_t)(percent >> 8U);

    return lsc16_send(
        dev,
        frame,
        sizeof(frame)
    );
}


/* ===================== 电池电压 ===================== */

/*请求LSC-16控制器返回电池电压*/
HAL_StatusTypeDef LSC16_RequestBatteryVoltage(
    LSC16_Handle_t *dev
)
{
    const uint8_t frame[4] =
    {
        0x55U,
        0x55U,
        0x02U,
        LSC16_CMD_BATTERY_VOLTAGE
    };

    return lsc16_send(
        dev,
        frame,
        sizeof(frame)
    );
}