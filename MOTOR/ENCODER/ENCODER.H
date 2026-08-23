#ifndef ENCODER_H
#define ENCODER_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 机械参数
 * 直接在工程中修改这些宏即可
 * ============================================================ */

/* 轮子直径，单位 mm。示例值 65 mm，按实际轮径修改 */
#ifndef ENCODER_WHEEL_DIAMETER_MM
#define ENCODER_WHEEL_DIAMETER_MM          65.0f
#endif

/*
 * 编码器线数：单通道每转产生的线数。
 * 例如 13 线磁编码器就填 13。
 */
#ifndef ENCODER_LINES_PER_MOTOR_REV
#define ENCODER_LINES_PER_MOTOR_REV        13.0f
#endif

/*
 * STM32 正交编码器一般按 A/B 两相四倍频计数。
 * 若你的计数方式不是四倍频，改这里。
 */
#ifndef ENCODER_QUADRATURE_MULTIPLIER
#define ENCODER_QUADRATURE_MULTIPLIER      4.0f
#endif

/*
 * 减速比：
 * 电机轴转 28 圈，轮子转 1 圈 -> 28。
 * 若编码器直接装在轮轴上，设为 1。
 */
#ifndef ENCODER_GEAR_RATIO
#define ENCODER_GEAR_RATIO                 1.0f
#endif

#ifndef ENCODER_PI
#define ENCODER_PI                         3.14159265358979323846f
#endif


/* ============================================================
 * 派生参数
 * ============================================================ */

/* 电机轴一圈对应 STM32 计数值 */
#define ENCODER_COUNTS_PER_MOTOR_REV \
    (ENCODER_LINES_PER_MOTOR_REV * ENCODER_QUADRATURE_MULTIPLIER)

/* 轮子一圈对应 STM32 计数值 */
#define ENCODER_COUNTS_PER_WHEEL_REV \
    (ENCODER_COUNTS_PER_MOTOR_REV * ENCODER_GEAR_RATIO)

/* 轮子周长，单位 mm */
#define ENCODER_WHEEL_CIRCUMFERENCE_MM \
    (ENCODER_PI * ENCODER_WHEEL_DIAMETER_MM)

/* 每一个编码器计数对应的实际距离，单位 mm/count */
#define ENCODER_MM_PER_COUNT \
    (ENCODER_WHEEL_CIRCUMFERENCE_MM / ENCODER_COUNTS_PER_WHEEL_REV)


/* ============================================================
 * 数据结构
 * ============================================================ */

typedef enum
{
    ENCODER_DIR_NORMAL  =  1,
    ENCODER_DIR_REVERSE = -1
} Encoder_Direction_t;

typedef struct
{
    TIM_HandleTypeDef *htim;

    Encoder_Direction_t direction;

    uint32_t last_raw_count;

    int32_t  delta_count;
    int64_t  total_count;

    float distance_mm;
    float speed_mm_s;
    float wheel_rpm;

    uint32_t last_update_tick;

} Encoder_Handle_t;


/* ============================================================
 * API
 * ============================================================ */

/*
 * 初始化并启动 STM32 TIM Encoder Mode。
 *
 * direction:
 *   ENCODER_DIR_NORMAL
 *   ENCODER_DIR_REVERSE
 */
HAL_StatusTypeDef Encoder_Init(
    Encoder_Handle_t *dev,
    TIM_HandleTypeDef *htim,
    Encoder_Direction_t direction
);

/*
 * 周期调用。
 *
 * 自动完成：
 * 1. 读取 CNT
 * 2. 处理定时器溢出/回绕
 * 3. 计算本周期增量
 * 4. 累计总脉冲
 * 5. 换算累计距离
 * 6. 计算线速度
 * 7. 计算轮子 RPM
 */
void Encoder_Update(
    Encoder_Handle_t *dev
);

/* 软件清零：累计脉冲、距离、速度全部归零 */
void Encoder_Reset(
    Encoder_Handle_t *dev
);

/* 获取本周期编码器增量 */
int32_t Encoder_GetDeltaCount(
    const Encoder_Handle_t *dev
);

/* 获取累计编码器计数 */
int64_t Encoder_GetTotalCount(
    const Encoder_Handle_t *dev
);

/* 获取累计实际距离，mm */
float Encoder_GetDistanceMM(
    const Encoder_Handle_t *dev
);

/* 获取累计实际距离，cm */
float Encoder_GetDistanceCM(
    const Encoder_Handle_t *dev
);

/* 获取线速度，mm/s */
float Encoder_GetSpeedMMps(
    const Encoder_Handle_t *dev
);

/* 获取轮子转速，RPM */
float Encoder_GetWheelRPM(
    const Encoder_Handle_t *dev
);

/* 单纯把任意编码器计数转换成距离 */
float Encoder_CountToDistanceMM(
    int64_t count
);

/* 单纯把距离转换成理论编码器计数 */
int64_t Encoder_DistanceMMToCount(
    float distance_mm
);

#ifdef __cplusplus
}
#endif

#endif /* ENCODER_H */