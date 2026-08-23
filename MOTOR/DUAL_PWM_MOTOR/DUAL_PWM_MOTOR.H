#ifndef DUAL_PWM_MOTOR_H
#define DUAL_PWM_MOTOR_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DUAL_PWM_MOTOR_OUTPUT_MAX   100.0f
#define DUAL_PWM_MOTOR_OUTPUT_MIN  (-100.0f)

typedef struct
{
    TIM_HandleTypeDef *htim;
    uint32_t forward_channel;
    uint32_t reverse_channel;
    float output;
} DualPWM_Motor_Handle_t;

HAL_StatusTypeDef DualPWM_Motor_Init(
    DualPWM_Motor_Handle_t *dev,
    TIM_HandleTypeDef *htim,
    uint32_t forward_channel,
    uint32_t reverse_channel
);

HAL_StatusTypeDef DualPWM_Motor_SetOutput(
    DualPWM_Motor_Handle_t *dev,
    float output
);

void DualPWM_Motor_Stop(
    DualPWM_Motor_Handle_t *dev
);

float DualPWM_Motor_GetOutput(
    const DualPWM_Motor_Handle_t *dev
);

HAL_StatusTypeDef DualPWM_Motor_DeInit(
    DualPWM_Motor_Handle_t *dev
);

#ifdef __cplusplus
}
#endif

#endif /* DUAL_PWM_MOTOR_H */