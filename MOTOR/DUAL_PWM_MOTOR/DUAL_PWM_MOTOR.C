#include "DUAL_PWM_MOTOR.h"

/* 设置某一个 PWM 通道的占空比，duty 范围 0~100 */
static void DualPWM_Motor_SetDuty(
    DualPWM_Motor_Handle_t *dev,
    uint32_t channel,
    float duty
)
{
    if (dev == NULL || dev->htim == NULL)
    {
        return;
    }

    if (duty < 0.0f)
    {
        duty = 0.0f;
    }
    else if (duty > 100.0f)
    {
        duty = 100.0f;
    }

    uint32_t arr =
        __HAL_TIM_GET_AUTORELOAD(dev->htim);

    uint32_t compare =
        (uint32_t)(((float)arr * duty) / 100.0f);

    __HAL_TIM_SET_COMPARE(
        dev->htim,
        channel,
        compare
    );
}

HAL_StatusTypeDef DualPWM_Motor_Init(
    DualPWM_Motor_Handle_t *dev,
    TIM_HandleTypeDef *htim,
    uint32_t forward_channel,
    uint32_t reverse_channel
)
{
    if (dev == NULL || htim == NULL)
    {
        return HAL_ERROR;
    }

    if (forward_channel == reverse_channel)
    {
        return HAL_ERROR;
    }

    dev->htim = htim;
    dev->forward_channel = forward_channel;
    dev->reverse_channel = reverse_channel;
    dev->output = 0.0f;

    /* 启动前先清零两个通道，避免意外输出 */
    DualPWM_Motor_SetDuty(
        dev,
        dev->forward_channel,
        0.0f
    );

    DualPWM_Motor_SetDuty(
        dev,
        dev->reverse_channel,
        0.0f
    );

    HAL_StatusTypeDef status =
        HAL_TIM_PWM_Start(
            dev->htim,
            dev->forward_channel
        );

    if (status != HAL_OK)
    {
        return status;
    }

    status =
        HAL_TIM_PWM_Start(
            dev->htim,
            dev->reverse_channel
        );

    if (status != HAL_OK)
    {
        HAL_TIM_PWM_Stop(
            dev->htim,
            dev->forward_channel
        );

        return status;
    }

    return HAL_OK;
}

HAL_StatusTypeDef DualPWM_Motor_SetOutput(
    DualPWM_Motor_Handle_t *dev,
    float output
)
{
    if (dev == NULL || dev->htim == NULL)
    {
        return HAL_ERROR;
    }

    if (output > DUAL_PWM_MOTOR_OUTPUT_MAX)
    {
        output = DUAL_PWM_MOTOR_OUTPUT_MAX;
    }
    else if (output < DUAL_PWM_MOTOR_OUTPUT_MIN)
    {
        output = DUAL_PWM_MOTOR_OUTPUT_MIN;
    }

    if (output > 0.0f)
    {
        /* 正转：先关闭反向，再打开正向 */
        DualPWM_Motor_SetDuty(
            dev,
            dev->reverse_channel,
            0.0f
        );

        DualPWM_Motor_SetDuty(
            dev,
            dev->forward_channel,
            output
        );
    }
    else if (output < 0.0f)
    {
        /* 反转：先关闭正向，再打开反向 */
        DualPWM_Motor_SetDuty(
            dev,
            dev->forward_channel,
            0.0f
        );

        DualPWM_Motor_SetDuty(
            dev,
            dev->reverse_channel,
            -output
        );
    }
    else
    {
        DualPWM_Motor_Stop(dev);
        return HAL_OK;
    }

    dev->output = output;

    return HAL_OK;
}

void DualPWM_Motor_Stop(
    DualPWM_Motor_Handle_t *dev
)
{
    if (dev == NULL || dev->htim == NULL)
    {
        return;
    }

    DualPWM_Motor_SetDuty(
        dev,
        dev->forward_channel,
        0.0f
    );

    DualPWM_Motor_SetDuty(
        dev,
        dev->reverse_channel,
        0.0f
    );

    dev->output = 0.0f;
}

float DualPWM_Motor_GetOutput(
    const DualPWM_Motor_Handle_t *dev
)
{
    if (dev == NULL)
    {
        return 0.0f;
    }

    return dev->output;
}

HAL_StatusTypeDef DualPWM_Motor_DeInit(
    DualPWM_Motor_Handle_t *dev
)
{
    if (dev == NULL || dev->htim == NULL)
    {
        return HAL_ERROR;
    }

    DualPWM_Motor_Stop(dev);

    HAL_StatusTypeDef status_forward =
        HAL_TIM_PWM_Stop(
            dev->htim,
            dev->forward_channel
        );

    HAL_StatusTypeDef status_reverse =
        HAL_TIM_PWM_Stop(
            dev->htim,
            dev->reverse_channel
        );

    dev->output = 0.0f;

    if (status_forward != HAL_OK)
    {
        return status_forward;
    }

    if (status_reverse != HAL_OK)
    {
        return status_reverse;
    }

    return HAL_OK;
}