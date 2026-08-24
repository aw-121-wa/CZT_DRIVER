#ifndef ZDT_X42S_CAN_H
#define ZDT_X42S_CAN_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ZDT X42S CAN free-protocol driver.
 * Basis: ZDT_X42S user manual V1.0.5.
 * Scope: STM32 HAL FDCAN, Classic CAN, Extended ID, fixed 0x6B checksum.
 */

#define ZDT_X42S_CHECK_BYTE                 0x6BU
#define ZDT_X42S_EMM_SPEED_MAX_RPM          3000U
#define ZDT_X42S_X_SPEED_MAX_X10_RPM        30000U
#define ZDT_X42S_CAN_DATA_MAX               8U
#define ZDT_X42S_CAN_PAYLOAD_PER_PACKET     7U

/* Function codes */
#define ZDT_X42S_CMD_ENABLE                 0xF3U
#define ZDT_X42S_CMD_SPEED                  0xF6U
#define ZDT_X42S_CMD_POSITION               0xFDU
#define ZDT_X42S_CMD_STOP                   0xFEU
#define ZDT_X42S_CMD_SYNC                   0xFFU
#define ZDT_X42S_CMD_HOME                   0x9AU
#define ZDT_X42S_CMD_ABORT_HOME             0x9CU
#define ZDT_X42S_CMD_READ_SPEED             0x35U
#define ZDT_X42S_CMD_READ_POSITION          0x36U
#define ZDT_X42S_CMD_READ_MOTOR_STATUS      0x3AU
#define ZDT_X42S_CMD_READ_HOME_STATUS       0x3BU
#define ZDT_X42S_CMD_READ_ALL_STATUS        0x3CU

/* Auxiliary codes */
#define ZDT_X42S_AUX_ENABLE                 0xABU
#define ZDT_X42S_AUX_STOP                   0x98U
#define ZDT_X42S_AUX_SYNC                   0x66U
#define ZDT_X42S_AUX_ABORT_HOME             0x48U

typedef enum
{
    ZDT_X42S_FW_EMM = 0,
    ZDT_X42S_FW_X
} ZDT_X42S_Firmware_t;

typedef enum
{
    ZDT_X42S_DIR_CW  = 0,
    ZDT_X42S_DIR_CCW = 1
} ZDT_X42S_Direction_t;

typedef enum
{
    ZDT_X42S_SYNC_IMMEDIATE = 0,
    ZDT_X42S_SYNC_BUFFERED  = 1
} ZDT_X42S_Sync_t;

typedef enum
{
    ZDT_X42S_POS_REL_LAST_TARGET = 0,
    ZDT_X42S_POS_ABSOLUTE_ZERO   = 1,
    ZDT_X42S_POS_REL_CURRENT     = 2
} ZDT_X42S_PositionMode_t;

typedef enum
{
    ZDT_X42S_HOME_NEAREST_SINGLE_TURN = 0,
    ZDT_X42S_HOME_DIRECTION_SINGLE    = 1,
    ZDT_X42S_HOME_COLLISION           = 2,
    ZDT_X42S_HOME_LIMIT_SWITCH        = 3,
    ZDT_X42S_HOME_ABSOLUTE_ZERO       = 4,
    ZDT_X42S_HOME_LAST_POWER_POSITION = 5
} ZDT_X42S_HomeMode_t;

typedef enum
{
    ZDT_X42S_RESP_NONE       = 0x00U,
    ZDT_X42S_RESP_OK         = 0x02U,
    ZDT_X42S_RESP_HOME_12    = 0x12U,
    ZDT_X42S_RESP_HOME_22    = 0x22U,
    ZDT_X42S_RESP_PARAM_ERR  = 0xE2U,
    ZDT_X42S_RESP_FORMAT_ERR = 0xEEU,
    ZDT_X42S_RESP_COMPLETE   = 0x9FU
} ZDT_X42S_Response_t;

/* Motor status bits returned by function 0x3A */
#define ZDT_X42S_MOTOR_STATUS_ENABLED       0x01U
#define ZDT_X42S_MOTOR_STATUS_REACHED       0x02U
#define ZDT_X42S_MOTOR_STATUS_STALL         0x04U
#define ZDT_X42S_MOTOR_STATUS_STALL_PROTECT 0x08U
#define ZDT_X42S_MOTOR_STATUS_LEFT_LIMIT    0x10U
#define ZDT_X42S_MOTOR_STATUS_RIGHT_LIMIT   0x20U
#define ZDT_X42S_MOTOR_STATUS_POWER_OFF     0x80U

/* Home status bits returned by function 0x3B */
#define ZDT_X42S_HOME_STATUS_ENCODER_READY  0x01U
#define ZDT_X42S_HOME_STATUS_CAL_READY      0x02U
#define ZDT_X42S_HOME_STATUS_RUNNING        0x04U
#define ZDT_X42S_HOME_STATUS_FAILED         0x08U
#define ZDT_X42S_HOME_STATUS_OVER_TEMP      0x10U
#define ZDT_X42S_HOME_STATUS_OVER_CURRENT   0x20U

typedef struct
{
    FDCAN_HandleTypeDef *hfdcan;
    uint8_t address;
    ZDT_X42S_Firmware_t firmware;

    uint32_t last_rx_identifier;
    uint8_t  last_rx_packet;
    uint8_t  last_rx_length;
    uint8_t  last_rx_data[8];
    uint8_t  last_function;
    ZDT_X42S_Response_t last_response;
    uint32_t rx_count;

    float speed_rpm;
    float position_deg;
    uint8_t speed_valid;
    uint8_t position_valid;

    uint8_t motor_status;
    uint8_t home_status;
    uint8_t motor_status_valid;
    uint8_t home_status_valid;
} ZDT_X42S_Handle_t;

HAL_StatusTypeDef ZDT_X42S_Init(
    ZDT_X42S_Handle_t *dev,
    FDCAN_HandleTypeDef *hfdcan,
    uint8_t address,
    ZDT_X42S_Firmware_t firmware
);

/* payload excludes function code and checksum. Driver automatically
 * repeats function_code at the head of every CAN packet and appends 0x6B.
 */
HAL_StatusTypeDef ZDT_X42S_SendCommand(
    ZDT_X42S_Handle_t *dev,
    uint8_t function_code,
    const uint8_t *payload,
    uint16_t payload_length
);

HAL_StatusTypeDef ZDT_X42S_Enable(
    ZDT_X42S_Handle_t *dev,
    uint8_t enable,
    ZDT_X42S_Sync_t sync
);

HAL_StatusTypeDef ZDT_X42S_Stop(
    ZDT_X42S_Handle_t *dev,
    ZDT_X42S_Sync_t sync
);

/* Uses dev only to obtain the bus handle. The transmitted address is 0. */
HAL_StatusTypeDef ZDT_X42S_TriggerSync(
    ZDT_X42S_Handle_t *dev
);

/* Emm: speed 0..3000 RPM, accel 0..255. accel=0 means direct start. */
HAL_StatusTypeDef ZDT_X42S_SetSpeedEmm(
    ZDT_X42S_Handle_t *dev,
    ZDT_X42S_Direction_t direction,
    uint16_t speed_rpm,
    uint8_t accel,
    ZDT_X42S_Sync_t sync
);

/* X: acceleration in RPM/s; speed_x10_rpm uses 0.1 RPM units. */
HAL_StatusTypeDef ZDT_X42S_SetSpeedX(
    ZDT_X42S_Handle_t *dev,
    ZDT_X42S_Direction_t direction,
    uint16_t accel_rpm_s,
    uint16_t speed_x10_rpm,
    ZDT_X42S_Sync_t sync
);

/* Emm position mode. pulse_count is protocol pulse count. */
HAL_StatusTypeDef ZDT_X42S_MovePositionEmm(
    ZDT_X42S_Handle_t *dev,
    ZDT_X42S_Direction_t direction,
    uint16_t speed_rpm,
    uint8_t accel,
    uint32_t pulse_count,
    ZDT_X42S_PositionMode_t mode,
    ZDT_X42S_Sync_t sync
);

/* X standard trapezoidal position mode. angle_x10_deg uses 0.1 degree units. */
HAL_StatusTypeDef ZDT_X42S_MovePositionX(
    ZDT_X42S_Handle_t *dev,
    ZDT_X42S_Direction_t direction,
    uint16_t accel_rpm_s,
    uint16_t decel_rpm_s,
    uint16_t max_speed_x10_rpm,
    uint32_t angle_x10_deg,
    ZDT_X42S_PositionMode_t mode,
    ZDT_X42S_Sync_t sync
);

HAL_StatusTypeDef ZDT_X42S_Home(
    ZDT_X42S_Handle_t *dev,
    ZDT_X42S_HomeMode_t mode,
    ZDT_X42S_Sync_t sync
);

HAL_StatusTypeDef ZDT_X42S_AbortHome(
    ZDT_X42S_Handle_t *dev
);

HAL_StatusTypeDef ZDT_X42S_ReadSpeed(ZDT_X42S_Handle_t *dev);
HAL_StatusTypeDef ZDT_X42S_ReadPosition(ZDT_X42S_Handle_t *dev);
HAL_StatusTypeDef ZDT_X42S_ReadMotorStatus(ZDT_X42S_Handle_t *dev);
HAL_StatusTypeDef ZDT_X42S_ReadHomeStatus(ZDT_X42S_Handle_t *dev);
HAL_StatusTypeDef ZDT_X42S_ReadAllStatus(ZDT_X42S_Handle_t *dev);

/* Feed a received FDCAN frame into the device parser.
 * The caller remains responsible for HAL_FDCAN_GetRxMessage().
 */
void ZDT_X42S_ProcessRxFrame(
    ZDT_X42S_Handle_t *dev,
    const FDCAN_RxHeaderTypeDef *rx_header,
    const uint8_t *data
);

float ZDT_X42S_GetSpeedRPM(const ZDT_X42S_Handle_t *dev);
float ZDT_X42S_GetPositionDeg(const ZDT_X42S_Handle_t *dev);
uint8_t ZDT_X42S_IsEnabled(const ZDT_X42S_Handle_t *dev);
uint8_t ZDT_X42S_IsReached(const ZDT_X42S_Handle_t *dev);
uint8_t ZDT_X42S_IsHoming(const ZDT_X42S_Handle_t *dev);
uint8_t ZDT_X42S_IsHomeFailed(const ZDT_X42S_Handle_t *dev);
ZDT_X42S_Response_t ZDT_X42S_GetLastResponse(const ZDT_X42S_Handle_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* ZDT_X42S_CAN_H */
