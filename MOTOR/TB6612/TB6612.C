#include "TB6612.h"

/* ============================================================
 * 私有函数
 * ============================================================ */

static void TB6612_SetDuty(
    TB6612_Handle_t *dev,
    float duty
)
{
    if (dev == NULL ||
        dev->htim == NULL)
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
        __HAL_TIM_GET_AUTORELOAD(
            dev->htim
        );

    uint32_t compare =
        (uint32_t)(
            ((float)(arr + 1U) * duty) /
            100.0f
        );

    __HAL_TIM_SET_COMPARE(
        dev->htim,
        dev->channel,
        compare
    );
}

static void TB6612_SetDirectionPins(
    TB6612_Handle_t *dev,
    GPIO_PinState in1_state,
    GPIO_PinState in2_state
)
{
    if (dev == NULL ||
        dev->in1_port == NULL ||
        dev->in2_port == NULL)
    {
        return;
    }

    HAL_GPIO_WritePin(
        dev->in1_port,
        dev->in1_pin,
        in1_state
    );

    HAL_GPIO_WritePin(
        dev->in2_port,
        dev->in2_pin,
        in2_state
    );
}

/* ============================================================
 * 初始化
 * ============================================================ */

HAL_StatusTypeDef TB6612_Init(
    TB6612_Handle_t *dev,
    TIM_HandleTypeDef *htim,
    uint32_t channel,
    GPIO_TypeDef *in1_port,
    uint16_t in1_pin,
    GPIO_TypeDef *in2_port,
    uint16_t in2_pin,
    GPIO_TypeDef *stby_port,
    uint16_t stby_pin
)
{
    if (dev == NULL ||
        htim == NULL ||
        in1_port == NULL ||
        in2_port == NULL)
    {
        return HAL_ERROR;
    }

    dev->htim = htim;
    dev->channel = channel;

    dev->in1_port = in1_port;
    dev->in1_pin = in1_pin;

    dev->in2_port = in2_port;
    dev->in2_pin = in2_pin;

    dev->stby_port = stby_port;
    dev->stby_pin = stby_pin;

    dev->speed = 0.0f;
    dev->state = TB6612_STATE_STOP;

    TB6612_Enable(dev);

    HAL_StatusTypeDef status =
        HAL_TIM_PWM_Start(
            dev->htim,
            dev->channel
        );

    if (status != HAL_OK)
    {
        return status;
    }

    TB6612_Stop(dev);

    return HAL_OK;
}

/* ============================================================
 * 正转 / 反转
 * ============================================================ */

HAL_StatusTypeDef TB6612_Forward(
    TB6612_Handle_t *dev,
    float duty
)
{
    if (dev == NULL ||
        dev->htim == NULL)
    {
        return HAL_ERROR;
    }

    if (duty < 0.0f)
    {
        duty = -duty;
    }

    if (duty > 100.0f)
    {
        duty = 100.0f;
    }

    TB6612_Enable(dev);

    TB6612_SetDirectionPins(
        dev,
        GPIO_PIN_SET,
        GPIO_PIN_RESET
    );

    TB6612_SetDuty(
        dev,
        duty
    );

    dev->speed = duty;
    dev->state = TB6612_STATE_FORWARD;

    return HAL_OK;
}

HAL_StatusTypeDef TB6612_Reverse(
    TB6612_Handle_t *dev,
    float duty
)
{
    if (dev == NULL ||
        dev->htim == NULL)
    {
        return HAL_ERROR;
    }

    if (duty < 0.0f)
    {
        duty = -duty;
    }

    if (duty > 100.0f)
    {
        duty = 100.0f;
    }

    TB6612_Enable(dev);

    TB6612_SetDirectionPins(
        dev,
        GPIO_PIN_RESET,
        GPIO_PIN_SET
    );

    TB6612_SetDuty(
        dev,
        duty
    );

    dev->speed = -duty;
    dev->state = TB6612_STATE_REVERSE;

    return HAL_OK;
}

/* ============================================================
 * 有符号速度
 * ============================================================ */

HAL_StatusTypeDef TB6612_SetSpeed(
    TB6612_Handle_t *dev,
    float speed
)
{
    if (dev == NULL)
    {
        return HAL_ERROR;
    }

    if (speed > TB6612_SPEED_MAX)
    {
        speed = TB6612_SPEED_MAX;
    }
    else if (speed < TB6612_SPEED_MIN)
    {
        speed = TB6612_SPEED_MIN;
    }

    if (speed > 0.0f)
    {
        return TB6612_Forward(
            dev,
            speed
        );
    }

    if (speed < 0.0f)
    {
        return TB6612_Reverse(
            dev,
            -speed
        );
    }

    TB6612_Stop(dev);

    return HAL_OK;
}

/* ============================================================
 * Stop / Brake
 * ============================================================ */

void TB6612_Stop(
    TB6612_Handle_t *dev
)
{
    if (dev == NULL)
    {
        return;
    }

    TB6612_SetDuty(
        dev,
        0.0f
    );

    TB6612_SetDirectionPins(
        dev,
        GPIO_PIN_RESET,
        GPIO_PIN_RESET
    );

    dev->speed = 0.0f;
    dev->state = TB6612_STATE_STOP;
}

void TB6612_Brake(
    TB6612_Handle_t *dev
)
{
    if (dev == NULL)
    {
        return;
    }

    TB6612_Enable(dev);

    TB6612_SetDirectionPins(
        dev,
        GPIO_PIN_SET,
        GPIO_PIN_SET
    );

    TB6612_SetDuty(
        dev,
        100.0f
    );

    dev->speed = 0.0f;
    dev->state = TB6612_STATE_BRAKE;
}

/* ============================================================
 * STBY
 * ============================================================ */

void TB6612_Enable(
    TB6612_Handle_t *dev
)
{
    if (dev == NULL)
    {
        return;
    }

    if (dev->stby_port != NULL)
    {
        HAL_GPIO_WritePin(
            dev->stby_port,
            dev->stby_pin,
            GPIO_PIN_SET
        );
    }
}

void TB6612_Disable(
    TB6612_Handle_t *dev
)
{
    if (dev == NULL)
    {
        return;
    }

    TB6612_SetDuty(
        dev,
        0.0f
    );

    TB6612_SetDirectionPins(
        dev,
        GPIO_PIN_RESET,
        GPIO_PIN_RESET
    );

    if (dev->stby_port != NULL)
    {
        HAL_GPIO_WritePin(
            dev->stby_port,
            dev->stby_pin,
            GPIO_PIN_RESET
        );
    }

    dev->speed = 0.0f;
    dev->state = TB6612_STATE_DISABLED;
}

/* ============================================================
 * 状态读取
 * ============================================================ */

float TB6612_GetSpeed(
    const TB6612_Handle_t *dev
)
{
    if (dev == NULL)
    {
        return 0.0f;
    }

    return dev->speed;
}

TB6612_State_t TB6612_GetState(
    const TB6612_Handle_t *dev
)
{
    if (dev == NULL)
    {
        return TB6612_STATE_DISABLED;
    }

    return dev->state;
}