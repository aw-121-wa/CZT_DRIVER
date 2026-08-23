#ifndef TB6612_H
#define TB6612_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TB6612_SPEED_MAX          100.0f
#define TB6612_SPEED_MIN         (-100.0f)

typedef enum
{
    TB6612_STATE_STOP = 0,
    TB6612_STATE_FORWARD,
    TB6612_STATE_REVERSE,
    TB6612_STATE_BRAKE,
    TB6612_STATE_DISABLED
} TB6612_State_t;

typedef struct
{
    TIM_HandleTypeDef *htim;
    uint32_t channel;

    GPIO_TypeDef *in1_port;
    uint16_t in1_pin;

    GPIO_TypeDef *in2_port;
    uint16_t in2_pin;

    /*
     * STBY 可选：
     * - MCU 控制 STBY：传入对应 GPIO
     * - 模块已硬件拉高：stby_port = NULL, stby_pin = 0
     */
    GPIO_TypeDef *stby_port;
    uint16_t stby_pin;

    float speed;               /* -100 ~ +100 */
    TB6612_State_t state;

} TB6612_Handle_t;

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
);

HAL_StatusTypeDef TB6612_SetSpeed(
    TB6612_Handle_t *dev,
    float speed
);

HAL_StatusTypeDef TB6612_Forward(
    TB6612_Handle_t *dev,
    float duty
);

HAL_StatusTypeDef TB6612_Reverse(
    TB6612_Handle_t *dev,
    float duty
);

void TB6612_Stop(
    TB6612_Handle_t *dev
);

void TB6612_Brake(
    TB6612_Handle_t *dev
);

void TB6612_Enable(
    TB6612_Handle_t *dev
);

void TB6612_Disable(
    TB6612_Handle_t *dev
);

float TB6612_GetSpeed(
    const TB6612_Handle_t *dev
);

TB6612_State_t TB6612_GetState(
    const TB6612_Handle_t *dev
);

#ifdef __cplusplus
}
#endif

#endif /* TB6612_H */