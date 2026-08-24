#ifndef ZDT_X42S_H
#define ZDT_X42S_H

#include "stm32h7xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ZDT X42S V1.0.5 free-CAN protocol:
 * Classic CAN, Extended ID, ID=(Addr<<8)|Packet, fixed tail 0x6B.
 */

/* 电机状态位 (motor_status) */
#define ZDT_X42S_STATUS_ENABLED       0x01U   /* 已使能 */
#define ZDT_X42S_STATUS_REACHED       0x02U   /* 目标到达 */
#define ZDT_X42S_STATUS_STALL         0x04U   /* 堵转 */
#define ZDT_X42S_STATUS_STALL_PROTECT 0x08U   /* 堵转保护触发 */

/* 回零状态位 (home_status) */
#define ZDT_X42S_HOME_RUNNING         0x04U   /* 回零中 */
#define ZDT_X42S_HOME_FAILED          0x08U   /* 回零失败 */

/* 固件版本（决定命令格式和参数精度） */
typedef enum {
    ZDT_X42S_FW_EMM = 0,   /* EMM 固件：速度整数RPM，位置用脉冲 */
    ZDT_X42S_FW_X          /* X 固件：速度0.1RPM，位置0.1度 */
} ZDT_X42S_Firmware_t;

/* 位置运动模式 */
typedef enum {
    ZDT_X42S_POS_REL_LAST = 0,  /* 相对上次目标位置 */
    ZDT_X42S_POS_ABS_ZERO = 1,  /* 相对绝对零点 */
    ZDT_X42S_POS_REL_NOW  = 2   /* 相对当前位置 */
} ZDT_X42S_PositionMode_t;

/* 回零模式 */
typedef enum {
    ZDT_X42S_HOME_NEAREST = 0,  /* 最近单圈回零 */
    ZDT_X42S_HOME_DIR     = 1,  /* 单方向回零 */
    ZDT_X42S_HOME_COLLIDE = 2,  /* 碰撞回零 */
    ZDT_X42S_HOME_LIMIT   = 3,  /* 限位开关回零 */
    ZDT_X42S_HOME_ZERO    = 4,  /* 绝对零点回零 */
    ZDT_X42S_HOME_POWER   = 5   /* 上次断电位置回零 */
} ZDT_X42S_HomeMode_t;

/**
 * @brief ZDT X42S 电机句柄结构体
 *
 * 使用前必须调用 ZDT_X42S_Init() 初始化。
 * RX 相关字段由 ProcessRx/DrainRxFifo 自动更新，声明为 volatile。
 */
typedef struct {
    FDCAN_HandleTypeDef *hfdcan;    /* FDCAN 外设句柄 */
    uint8_t address;                /* 电机 CAN 地址 */
    ZDT_X42S_Firmware_t firmware;   /* 固件类型 */

    uint32_t pulses_per_rev;        /* EMM 回零/位置: 每转脉冲数，默认 3200 */

    /* 以下字段由 ProcessRx()/DrainRxFifo() 自动更新 */
    volatile float speed_rpm;           /* 实时转速 (RPM) */
    volatile float position_deg;        /* 实时位置 (度) */
    volatile uint8_t motor_status;      /* 电机状态位掩码 */
    volatile uint8_t home_status;       /* 回零状态位掩码 */
    volatile uint8_t last_response;     /* 最近一次控制命令响应码 */
    volatile uint32_t last_reply_ms;    /* 最近一次收到回复的时间戳 (ms) */
    volatile uint8_t has_reply;         /* 是否收到过回复 (0/1) */
} ZDT_X42S_t;


/* ---------------------------- Init ---------------------------- */

HAL_StatusTypeDef ZDT_X42S_Init(
    ZDT_X42S_t *motor,
    FDCAN_HandleTypeDef *hfdcan,
    uint8_t address,
    ZDT_X42S_Firmware_t firmware);

void ZDT_X42S_SetPulsesPerRev(
    ZDT_X42S_t *motor,
    uint32_t pulses_per_rev);


/* ----------------------- Low-level escape ---------------------- */

/* payload does NOT contain function code or final 0x6B.
 * Long commands are split automatically; every CAN packet repeats code.
 */
HAL_StatusTypeDef ZDT_X42S_SendRaw(
    ZDT_X42S_t *motor,
    uint8_t code,
    const uint8_t *payload,
    uint16_t payload_len);


/* -------------------------- Control --------------------------- */

/* One motor: immediate execution. */
HAL_StatusTypeDef ZDT_X42S_Enable(
    ZDT_X42S_t *motor,
    bool enable);

/* Many motors: buffered commands + one broadcast sync. */
HAL_StatusTypeDef ZDT_X42S_EnableAll(
    ZDT_X42S_t motors[],
    uint8_t count,
    bool enable);

/* Signed RPM. Direction is selected automatically.
 * accel:
 *   X   -> RPM/s
 *   Emm -> 0..255 acceleration level
 */
HAL_StatusTypeDef ZDT_X42S_SetSpeed(
    ZDT_X42S_t *motor,
    float rpm,
    uint16_t accel);

/* One synchronized speed update for N contiguous motor handles. */
HAL_StatusTypeDef ZDT_X42S_SetSpeeds(
    ZDT_X42S_t motors[],
    uint8_t count,
    const float rpm[],
    uint16_t accel);

/* Signed degrees.
 * X: uses trapezoid position command; accel is RPM/s.
 * Emm: degrees are converted with pulses_per_rev; accel must be 0..255.
 */
HAL_StatusTypeDef ZDT_X42S_MoveDeg(
    ZDT_X42S_t *motor,
    float degrees,
    float max_rpm,
    uint16_t accel,
    ZDT_X42S_PositionMode_t mode);

HAL_StatusTypeDef ZDT_X42S_Stop(
    ZDT_X42S_t *motor);

HAL_StatusTypeDef ZDT_X42S_StopAll(
    ZDT_X42S_t motors[],
    uint8_t count);

HAL_StatusTypeDef ZDT_X42S_Home(
    ZDT_X42S_t *motor,
    ZDT_X42S_HomeMode_t mode);

HAL_StatusTypeDef ZDT_X42S_AbortHome(
    ZDT_X42S_t *motor);

/* Broadcast FF 66 6B. Useful for advanced/manual buffered commands. */
HAL_StatusTypeDef ZDT_X42S_TriggerSync(
    ZDT_X42S_t *motor);


/* --------------------------- Read ----------------------------- */

HAL_StatusTypeDef ZDT_X42S_ReadSpeed(ZDT_X42S_t *motor);
HAL_StatusTypeDef ZDT_X42S_ReadPosition(ZDT_X42S_t *motor);
HAL_StatusTypeDef ZDT_X42S_ReadStatus(ZDT_X42S_t *motor);
HAL_StatusTypeDef ZDT_X42S_ReadHomeStatus(ZDT_X42S_t *motor);
HAL_StatusTypeDef ZDT_X42S_ReadAllStatus(ZDT_X42S_t *motor);


/* ------------------------ RX processing ------------------------ */

/* Pass the same contiguous motor array used by SetSpeeds().
 * The function extracts address from Extended ID and updates that motor.
 */
bool ZDT_X42S_ProcessRx(
    ZDT_X42S_t motors[],
    uint8_t count,
    const FDCAN_RxHeaderTypeDef *header,
    const uint8_t data[8]);

/* Convenience for HAL_FDCAN_RxFifo0Callback().
 * Returns number of frames removed from the FIFO.
 */
uint32_t ZDT_X42S_DrainRxFifo(
    ZDT_X42S_t motors[],
    uint8_t count,
    uint32_t fifo);

/* True if this motor has produced a legal reply within timeout_ms. */
bool ZDT_X42S_IsOnline(
    const ZDT_X42S_t *motor,
    uint32_t timeout_ms);


#ifdef __cplusplus
}
#endif
#endif
