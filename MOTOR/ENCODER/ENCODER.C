#include "ENCODER.h"

#include <string.h>


/* ============================================================
 * 内部函数
 * ============================================================ */

/*
 * 根据 TIM 的 ARR 自动处理 CNT 回绕。
 *
 * 要求：
 * 两次 Encoder_Update() 之间的真实计数变化
 * 不得超过计数器量程的一半。
 *
 * 对常见 16 位 TIM：
 * ARR = 65535
 * 安全单次变化范围约 ±32767 count。
 */
static int32_t Encoder_CalcDelta(
    Encoder_Handle_t *dev,
    uint32_t current_count
)
{
    uint64_t modulus =
        (uint64_t)__HAL_TIM_GET_AUTORELOAD(dev->htim) + 1ULL;

    int64_t delta =
        (int64_t)current_count -
        (int64_t)dev->last_raw_count;

    int64_t half =
        (int64_t)(modulus / 2ULL);

    if (delta > half)
    {
        delta -= (int64_t)modulus;
    }
    else if (delta < -half)
    {
        delta += (int64_t)modulus;
    }

    return (int32_t)delta;
}


/* ============================================================
 * 初始化
 * ============================================================ */
HAL_StatusTypeDef Encoder_Init(
    Encoder_Handle_t *dev,
    TIM_HandleTypeDef *htim,
    Encoder_Direction_t direction
)
{
    if (dev == NULL || htim == NULL)
    {
        return HAL_ERROR;
    }

    if (direction != ENCODER_DIR_NORMAL &&
        direction != ENCODER_DIR_REVERSE)
    {
        return HAL_ERROR;
    }

    memset(dev, 0, sizeof(*dev));

    dev->htim = htim;
    dev->direction = direction;

    /*
     * 从 0 开始计数。
     */
    __HAL_TIM_SET_COUNTER(dev->htim, 0U);

    HAL_StatusTypeDef status =
        HAL_TIM_Encoder_Start(
            dev->htim,
            TIM_CHANNEL_ALL
        );

    if (status != HAL_OK)
    {
        return status;
    }

    dev->last_raw_count =
        __HAL_TIM_GET_COUNTER(dev->htim);

    dev->last_update_tick =
        HAL_GetTick();

    return HAL_OK;
}

/*更新编码器状态，计算速度、距离和RPM*/
void Encoder_Update(
    Encoder_Handle_t *dev
)
{
    if (dev == NULL || dev->htim == NULL)
    {
        return;
    }

    uint32_t now_tick =
        HAL_GetTick();

    uint32_t current_count =
        __HAL_TIM_GET_COUNTER(dev->htim);

    int32_t raw_delta =
        Encoder_CalcDelta(
            dev,
            current_count
        );

    /*
     * 用软件方向统一左右轮安装差异。
     */
    int32_t delta =
        raw_delta * (int32_t)dev->direction;

    dev->last_raw_count =
        current_count;

    dev->delta_count =
        delta;

    dev->total_count +=
        (int64_t)delta;


    /* ========================
     * 累计距离
     * ======================== */

    dev->distance_mm =
        (float)dev->total_count *
        ENCODER_MM_PER_COUNT;


    /* ========================
     * 速度 / RPM
     * ======================== */

    uint32_t dt_ms =
        now_tick - dev->last_update_tick;

    if (dt_ms > 0U)
    {
        float dt_s =
            (float)dt_ms / 1000.0f;

        float delta_distance_mm =
            (float)delta *
            ENCODER_MM_PER_COUNT;

        dev->speed_mm_s =
            delta_distance_mm / dt_s;


        /*
         * delta_count / counts_per_rev
         * = 本周期轮子转过的圈数
         *
         * / dt_s
         * = rps
         *
         * * 60
         * = rpm
         */
        dev->wheel_rpm =
            ((float)delta /
             ENCODER_COUNTS_PER_WHEEL_REV)
            / dt_s
            * 60.0f;
    }

    dev->last_update_tick =
        now_tick;
}

/*重置编码器，清零计数器和所有状态变量*/
void Encoder_Reset(
    Encoder_Handle_t *dev
)
{
    if (dev == NULL || dev->htim == NULL)
    {
        return;
    }

    __HAL_TIM_SET_COUNTER(
        dev->htim,
        0U
    );

    dev->last_raw_count = 0U;

    dev->delta_count = 0;
    dev->total_count = 0;

    dev->distance_mm = 0.0f;
    dev->speed_mm_s  = 0.0f;
    dev->wheel_rpm   = 0.0f;

    dev->last_update_tick =
        HAL_GetTick();
}

/*获取本周期编码器计数变化量*/
int32_t Encoder_GetDeltaCount(
    const Encoder_Handle_t *dev
)
{
    if (dev == NULL)
        return 0;

    return dev->delta_count;
}


/*获取编码器总计数值*/
int64_t Encoder_GetTotalCount(
    const Encoder_Handle_t *dev
)
{
    if (dev == NULL)
        return 0;

    return dev->total_count;
}


/*获取累计距离，单位毫米*/
float Encoder_GetDistanceMM(
    const Encoder_Handle_t *dev
)
{
    if (dev == NULL)
        return 0.0f;

    return dev->distance_mm;
}


/*获取累计距离，单位厘米*/
float Encoder_GetDistanceCM(
    const Encoder_Handle_t *dev
)
{
    if (dev == NULL)
        return 0.0f;

    return dev->distance_mm / 10.0f;
}


/*获取当前速度，单位毫米/秒*/
float Encoder_GetSpeedMMps(
    const Encoder_Handle_t *dev
)
{
    if (dev == NULL)
        return 0.0f;

    return dev->speed_mm_s;
}


/*获取车轮转速，单位转/分钟*/
float Encoder_GetWheelRPM(
    const Encoder_Handle_t *dev
)
{
    if (dev == NULL)
        return 0.0f;

    return dev->wheel_rpm;
}


/*将编码器计数转换为距离，单位毫米*/
float Encoder_CountToDistanceMM(
    int64_t count
)
{
    return (float)count *
           ENCODER_MM_PER_COUNT;
}


/*将距离转换为编码器计数，单位毫米*/
int64_t Encoder_DistanceMMToCount(
    float distance_mm
)
{
    if (ENCODER_MM_PER_COUNT <= 0.0f)
    {
        return 0;
    }

    float count =
        distance_mm /
        ENCODER_MM_PER_COUNT;

    /*
     * 四舍五入。
     */
    if (count >= 0.0f)
    {
        return (int64_t)(count + 0.5f);
    }
    else
    {
        return (int64_t)(count - 0.5f);
    }
}