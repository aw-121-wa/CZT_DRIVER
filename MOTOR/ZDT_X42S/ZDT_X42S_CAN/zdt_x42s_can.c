#include "zdt_x42s_can.h"
#include <string.h>

static uint8_t ZDT_X42S_IsDirectionValid(ZDT_X42S_Direction_t direction)
{
    return (direction == ZDT_X42S_DIR_CW || direction == ZDT_X42S_DIR_CCW) ? 1U : 0U;
}

static uint8_t ZDT_X42S_IsSyncValid(ZDT_X42S_Sync_t sync)
{
    return (sync == ZDT_X42S_SYNC_IMMEDIATE || sync == ZDT_X42S_SYNC_BUFFERED) ? 1U : 0U;
}

static uint8_t ZDT_X42S_IsPositionModeValid(ZDT_X42S_PositionMode_t mode)
{
    return (mode == ZDT_X42S_POS_REL_LAST_TARGET ||
            mode == ZDT_X42S_POS_ABSOLUTE_ZERO ||
            mode == ZDT_X42S_POS_REL_CURRENT) ? 1U : 0U;
}

static uint8_t ZDT_X42S_IsHomeModeValid(ZDT_X42S_HomeMode_t mode)
{
    return ((uint8_t)mode <= 5U) ? 1U : 0U;
}

static void ZDT_X42S_PutU16BE(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)(value >> 8);
    dst[1] = (uint8_t)value;
}

static void ZDT_X42S_PutU32BE(uint8_t *dst, uint32_t value)
{
    dst[0] = (uint8_t)(value >> 24);
    dst[1] = (uint8_t)(value >> 16);
    dst[2] = (uint8_t)(value >> 8);
    dst[3] = (uint8_t)value;
}

static uint16_t ZDT_X42S_ReadU16BE(const uint8_t *src)
{
    return ((uint16_t)src[0] << 8) | (uint16_t)src[1];
}

static uint32_t ZDT_X42S_ReadU32BE(const uint8_t *src)
{
    return ((uint32_t)src[0] << 24) |
           ((uint32_t)src[1] << 16) |
           ((uint32_t)src[2] << 8) |
           (uint32_t)src[3];
}

/* Manual rule: Extended CAN ID = (Addr << 8) | Packet. */
static uint32_t ZDT_X42S_MakeCANID(uint8_t address, uint8_t packet)
{
    return ((uint32_t)address << 8) | (uint32_t)packet;
}

static uint32_t ZDT_X42S_LengthToDLC(uint8_t length)
{
    switch (length)
    {
        case 0U: return FDCAN_DLC_BYTES_0;
        case 1U: return FDCAN_DLC_BYTES_1;
        case 2U: return FDCAN_DLC_BYTES_2;
        case 3U: return FDCAN_DLC_BYTES_3;
        case 4U: return FDCAN_DLC_BYTES_4;
        case 5U: return FDCAN_DLC_BYTES_5;
        case 6U: return FDCAN_DLC_BYTES_6;
        case 7U: return FDCAN_DLC_BYTES_7;
        case 8U: return FDCAN_DLC_BYTES_8;
        default: return FDCAN_DLC_BYTES_0;
    }
}

static uint8_t ZDT_X42S_DLCToLength(uint32_t dlc)
{
    switch (dlc)
    {
        case FDCAN_DLC_BYTES_0: return 0U;
        case FDCAN_DLC_BYTES_1: return 1U;
        case FDCAN_DLC_BYTES_2: return 2U;
        case FDCAN_DLC_BYTES_3: return 3U;
        case FDCAN_DLC_BYTES_4: return 4U;
        case FDCAN_DLC_BYTES_5: return 5U;
        case FDCAN_DLC_BYTES_6: return 6U;
        case FDCAN_DLC_BYTES_7: return 7U;
        case FDCAN_DLC_BYTES_8: return 8U;
        default: return 0U;
    }
}

static HAL_StatusTypeDef ZDT_X42S_SendPacketToAddress(
    FDCAN_HandleTypeDef *hfdcan,
    uint8_t address,
    uint8_t packet,
    const uint8_t *data,
    uint8_t length)
{
    if (hfdcan == NULL || data == NULL || length == 0U || length > ZDT_X42S_CAN_DATA_MAX)
    {
        return HAL_ERROR;
    }

    if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) == 0U)
    {
        return HAL_BUSY;
    }

    FDCAN_TxHeaderTypeDef tx_header;
    memset(&tx_header, 0, sizeof(tx_header));

    tx_header.Identifier = ZDT_X42S_MakeCANID(address, packet);
    tx_header.IdType = FDCAN_EXTENDED_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = ZDT_X42S_LengthToDLC(length);
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0U;

    return HAL_FDCAN_AddMessageToTxFifoQ(
        hfdcan,
        &tx_header,
        (uint8_t *)data);
}

/*
 * X42S CAN packetization:
 *   CAN data[0] is the function code in EVERY packet.
 *   Therefore each Classic CAN packet carries at most seven new bytes.
 *   The fixed checksum 0x6B is appended after the payload.
 */
static HAL_StatusTypeDef ZDT_X42S_SendCommandToAddress(
    FDCAN_HandleTypeDef *hfdcan,
    uint8_t address,
    uint8_t function_code,
    const uint8_t *payload,
    uint16_t payload_length)
{
    if (hfdcan == NULL)
    {
        return HAL_ERROR;
    }

    if (payload_length > 0U && payload == NULL)
    {
        return HAL_ERROR;
    }

    uint32_t total_data_length = (uint32_t)payload_length + 1UL;
    uint32_t offset = 0UL;
    uint8_t packet = 0U;
    uint8_t frame[8];

    while (offset < total_data_length)
    {
        uint8_t frame_length = 1U;
        frame[0] = function_code;

        while (frame_length < 8U && offset < total_data_length)
        {
            if (offset < payload_length)
            {
                frame[frame_length] = payload[offset];
            }
            else
            {
                frame[frame_length] = ZDT_X42S_CHECK_BYTE;
            }

            frame_length++;
            offset++;
        }

        HAL_StatusTypeDef status = ZDT_X42S_SendPacketToAddress(
            hfdcan,
            address,
            packet,
            frame,
            frame_length);

        if (status != HAL_OK)
        {
            return status;
        }

        if (packet == 0xFFU && offset < total_data_length)
        {
            return HAL_ERROR;
        }

        packet++;
    }

    return HAL_OK;
}

static uint8_t ZDT_X42S_IsResponseCode(uint8_t value)
{
    switch (value)
    {
        case ZDT_X42S_RESP_OK:
        case ZDT_X42S_RESP_HOME_12:
        case ZDT_X42S_RESP_HOME_22:
        case ZDT_X42S_RESP_PARAM_ERR:
        case ZDT_X42S_RESP_FORMAT_ERR:
        case ZDT_X42S_RESP_COMPLETE:
            return 1U;
        default:
            return 0U;
    }
}

static uint8_t ZDT_X42S_IsControlFunction(uint8_t function_code)
{
    switch (function_code)
    {
        case ZDT_X42S_CMD_ENABLE:
        case ZDT_X42S_CMD_SPEED:
        case ZDT_X42S_CMD_POSITION:
        case ZDT_X42S_CMD_STOP:
        case ZDT_X42S_CMD_SYNC:
        case ZDT_X42S_CMD_HOME:
        case ZDT_X42S_CMD_ABORT_HOME:
            return 1U;
        default:
            return 0U;
    }
}

HAL_StatusTypeDef ZDT_X42S_Init(
    ZDT_X42S_Handle_t *dev,
    FDCAN_HandleTypeDef *hfdcan,
    uint8_t address,
    ZDT_X42S_Firmware_t firmware)
{
    if (dev == NULL || hfdcan == NULL)
    {
        return HAL_ERROR;
    }

    if (firmware != ZDT_X42S_FW_EMM && firmware != ZDT_X42S_FW_X)
    {
        return HAL_ERROR;
    }

    memset(dev, 0, sizeof(*dev));

    dev->hfdcan = hfdcan;
    dev->address = address;
    dev->firmware = firmware;
    dev->last_response = ZDT_X42S_RESP_NONE;

    return HAL_OK;
}

HAL_StatusTypeDef ZDT_X42S_SendCommand(
    ZDT_X42S_Handle_t *dev,
    uint8_t function_code,
    const uint8_t *payload,
    uint16_t payload_length)
{
    if (dev == NULL || dev->hfdcan == NULL)
    {
        return HAL_ERROR;
    }

    return ZDT_X42S_SendCommandToAddress(
        dev->hfdcan,
        dev->address,
        function_code,
        payload,
        payload_length);
}

/* Addr F3 AB 00/01 00/01 6B */
HAL_StatusTypeDef ZDT_X42S_Enable(
    ZDT_X42S_Handle_t *dev,
    uint8_t enable,
    ZDT_X42S_Sync_t sync)
{
    if (dev == NULL || dev->hfdcan == NULL)
    {
        return HAL_ERROR;
    }

    if (enable > 1U || ZDT_X42S_IsSyncValid(sync) == 0U)
    {
        return HAL_ERROR;
    }

    uint8_t payload[3];
    payload[0] = ZDT_X42S_AUX_ENABLE;
    payload[1] = enable;
    payload[2] = (uint8_t)sync;

    return ZDT_X42S_SendCommand(dev, ZDT_X42S_CMD_ENABLE, payload, sizeof(payload));
}

/* Addr FE 98 00/01 6B */
HAL_StatusTypeDef ZDT_X42S_Stop(
    ZDT_X42S_Handle_t *dev,
    ZDT_X42S_Sync_t sync)
{
    if (dev == NULL || dev->hfdcan == NULL || ZDT_X42S_IsSyncValid(sync) == 0U)
    {
        return HAL_ERROR;
    }

    uint8_t payload[2];
    payload[0] = ZDT_X42S_AUX_STOP;
    payload[1] = (uint8_t)sync;

    return ZDT_X42S_SendCommand(dev, ZDT_X42S_CMD_STOP, payload, sizeof(payload));
}

/* Broadcast: 00 FF 66 6B */
HAL_StatusTypeDef ZDT_X42S_TriggerSync(ZDT_X42S_Handle_t *dev)
{
    if (dev == NULL || dev->hfdcan == NULL)
    {
        return HAL_ERROR;
    }

    uint8_t payload[1] = { ZDT_X42S_AUX_SYNC };

    return ZDT_X42S_SendCommandToAddress(
        dev->hfdcan,
        0U,
        ZDT_X42S_CMD_SYNC,
        payload,
        sizeof(payload));
}

/* Emm: Addr F6 Dir Speed(2) Acc Sync 6B */
HAL_StatusTypeDef ZDT_X42S_SetSpeedEmm(
    ZDT_X42S_Handle_t *dev,
    ZDT_X42S_Direction_t direction,
    uint16_t speed_rpm,
    uint8_t accel,
    ZDT_X42S_Sync_t sync)
{
    if (dev == NULL || dev->hfdcan == NULL)
    {
        return HAL_ERROR;
    }

    if (dev->firmware != ZDT_X42S_FW_EMM ||
        ZDT_X42S_IsDirectionValid(direction) == 0U ||
        ZDT_X42S_IsSyncValid(sync) == 0U ||
        speed_rpm > ZDT_X42S_EMM_SPEED_MAX_RPM)
    {
        return HAL_ERROR;
    }

    uint8_t payload[5];
    payload[0] = (uint8_t)direction;
    ZDT_X42S_PutU16BE(&payload[1], speed_rpm);
    payload[3] = accel;
    payload[4] = (uint8_t)sync;

    return ZDT_X42S_SendCommand(dev, ZDT_X42S_CMD_SPEED, payload, sizeof(payload));
}

/* X: Addr F6 Dir Acc(2) Speed(2, 0.1RPM) Sync 6B */
HAL_StatusTypeDef ZDT_X42S_SetSpeedX(
    ZDT_X42S_Handle_t *dev,
    ZDT_X42S_Direction_t direction,
    uint16_t accel_rpm_s,
    uint16_t speed_x10_rpm,
    ZDT_X42S_Sync_t sync)
{
    if (dev == NULL || dev->hfdcan == NULL)
    {
        return HAL_ERROR;
    }

    if (dev->firmware != ZDT_X42S_FW_X ||
        ZDT_X42S_IsDirectionValid(direction) == 0U ||
        ZDT_X42S_IsSyncValid(sync) == 0U ||
        speed_x10_rpm > ZDT_X42S_X_SPEED_MAX_X10_RPM)
    {
        return HAL_ERROR;
    }

    uint8_t payload[6];
    payload[0] = (uint8_t)direction;
    ZDT_X42S_PutU16BE(&payload[1], accel_rpm_s);
    ZDT_X42S_PutU16BE(&payload[3], speed_x10_rpm);
    payload[5] = (uint8_t)sync;

    return ZDT_X42S_SendCommand(dev, ZDT_X42S_CMD_SPEED, payload, sizeof(payload));
}

/* Emm: Addr FD Dir Speed(2) Acc Pulse(4) Mode Sync 6B */
HAL_StatusTypeDef ZDT_X42S_MovePositionEmm(
    ZDT_X42S_Handle_t *dev,
    ZDT_X42S_Direction_t direction,
    uint16_t speed_rpm,
    uint8_t accel,
    uint32_t pulse_count,
    ZDT_X42S_PositionMode_t mode,
    ZDT_X42S_Sync_t sync)
{
    if (dev == NULL || dev->hfdcan == NULL)
    {
        return HAL_ERROR;
    }

    if (dev->firmware != ZDT_X42S_FW_EMM ||
        ZDT_X42S_IsDirectionValid(direction) == 0U ||
        ZDT_X42S_IsPositionModeValid(mode) == 0U ||
        ZDT_X42S_IsSyncValid(sync) == 0U ||
        speed_rpm > ZDT_X42S_EMM_SPEED_MAX_RPM)
    {
        return HAL_ERROR;
    }

    uint8_t payload[10];
    payload[0] = (uint8_t)direction;
    ZDT_X42S_PutU16BE(&payload[1], speed_rpm);
    payload[3] = accel;
    ZDT_X42S_PutU32BE(&payload[4], pulse_count);
    payload[8] = (uint8_t)mode;
    payload[9] = (uint8_t)sync;

    dev->motor_status &= (uint8_t)~ZDT_X42S_MOTOR_STATUS_REACHED;

    return ZDT_X42S_SendCommand(dev, ZDT_X42S_CMD_POSITION, payload, sizeof(payload));
}

/* X standard trapezoidal position mode:
 * Addr FD Dir Accel(2) Decel(2) MaxSpeed(2) Angle(4) Mode Sync 6B
 */
HAL_StatusTypeDef ZDT_X42S_MovePositionX(
    ZDT_X42S_Handle_t *dev,
    ZDT_X42S_Direction_t direction,
    uint16_t accel_rpm_s,
    uint16_t decel_rpm_s,
    uint16_t max_speed_x10_rpm,
    uint32_t angle_x10_deg,
    ZDT_X42S_PositionMode_t mode,
    ZDT_X42S_Sync_t sync)
{
    if (dev == NULL || dev->hfdcan == NULL)
    {
        return HAL_ERROR;
    }

    if (dev->firmware != ZDT_X42S_FW_X ||
        ZDT_X42S_IsDirectionValid(direction) == 0U ||
        ZDT_X42S_IsPositionModeValid(mode) == 0U ||
        ZDT_X42S_IsSyncValid(sync) == 0U ||
        max_speed_x10_rpm > ZDT_X42S_X_SPEED_MAX_X10_RPM)
    {
        return HAL_ERROR;
    }

    uint8_t payload[13];
    payload[0] = (uint8_t)direction;
    ZDT_X42S_PutU16BE(&payload[1], accel_rpm_s);
    ZDT_X42S_PutU16BE(&payload[3], decel_rpm_s);
    ZDT_X42S_PutU16BE(&payload[5], max_speed_x10_rpm);
    ZDT_X42S_PutU32BE(&payload[7], angle_x10_deg);
    payload[11] = (uint8_t)mode;
    payload[12] = (uint8_t)sync;

    dev->motor_status &= (uint8_t)~ZDT_X42S_MOTOR_STATUS_REACHED;

    return ZDT_X42S_SendCommand(dev, ZDT_X42S_CMD_POSITION, payload, sizeof(payload));
}

/* Addr 9A Mode Sync 6B */
HAL_StatusTypeDef ZDT_X42S_Home(
    ZDT_X42S_Handle_t *dev,
    ZDT_X42S_HomeMode_t mode,
    ZDT_X42S_Sync_t sync)
{
    if (dev == NULL || dev->hfdcan == NULL)
    {
        return HAL_ERROR;
    }

    if (ZDT_X42S_IsHomeModeValid(mode) == 0U || ZDT_X42S_IsSyncValid(sync) == 0U)
    {
        return HAL_ERROR;
    }

    uint8_t payload[2];
    payload[0] = (uint8_t)mode;
    payload[1] = (uint8_t)sync;

    return ZDT_X42S_SendCommand(dev, ZDT_X42S_CMD_HOME, payload, sizeof(payload));
}

/* Addr 9C 48 6B */
HAL_StatusTypeDef ZDT_X42S_AbortHome(ZDT_X42S_Handle_t *dev)
{
    if (dev == NULL || dev->hfdcan == NULL)
    {
        return HAL_ERROR;
    }

    uint8_t payload[1] = { ZDT_X42S_AUX_ABORT_HOME };
    return ZDT_X42S_SendCommand(dev, ZDT_X42S_CMD_ABORT_HOME, payload, sizeof(payload));
}

HAL_StatusTypeDef ZDT_X42S_ReadSpeed(ZDT_X42S_Handle_t *dev)
{
    if (dev == NULL) return HAL_ERROR;
    return ZDT_X42S_SendCommand(dev, ZDT_X42S_CMD_READ_SPEED, NULL, 0U);
}

HAL_StatusTypeDef ZDT_X42S_ReadPosition(ZDT_X42S_Handle_t *dev)
{
    if (dev == NULL) return HAL_ERROR;
    return ZDT_X42S_SendCommand(dev, ZDT_X42S_CMD_READ_POSITION, NULL, 0U);
}

HAL_StatusTypeDef ZDT_X42S_ReadMotorStatus(ZDT_X42S_Handle_t *dev)
{
    if (dev == NULL) return HAL_ERROR;
    return ZDT_X42S_SendCommand(dev, ZDT_X42S_CMD_READ_MOTOR_STATUS, NULL, 0U);
}

HAL_StatusTypeDef ZDT_X42S_ReadHomeStatus(ZDT_X42S_Handle_t *dev)
{
    if (dev == NULL) return HAL_ERROR;
    return ZDT_X42S_SendCommand(dev, ZDT_X42S_CMD_READ_HOME_STATUS, NULL, 0U);
}

HAL_StatusTypeDef ZDT_X42S_ReadAllStatus(ZDT_X42S_Handle_t *dev)
{
    if (dev == NULL) return HAL_ERROR;
    return ZDT_X42S_SendCommand(dev, ZDT_X42S_CMD_READ_ALL_STATUS, NULL, 0U);
}

void ZDT_X42S_ProcessRxFrame(
    ZDT_X42S_Handle_t *dev,
    const FDCAN_RxHeaderTypeDef *rx_header,
    const uint8_t *data)
{
    if (dev == NULL || rx_header == NULL || data == NULL)
    {
        return;
    }

    if (rx_header->IdType != FDCAN_EXTENDED_ID)
    {
        return;
    }

    uint8_t rx_address = (uint8_t)((rx_header->Identifier >> 8) & 0xFFU);
    uint8_t packet = (uint8_t)(rx_header->Identifier & 0xFFU);

    if (rx_address != dev->address)
    {
        return;
    }

    uint8_t length = ZDT_X42S_DLCToLength(rx_header->DataLength);
    if (length == 0U || length > 8U)
    {
        return;
    }

    dev->last_rx_identifier = rx_header->Identifier;
    dev->last_rx_packet = packet;
    dev->last_rx_length = length;
    memset(dev->last_rx_data, 0, sizeof(dev->last_rx_data));
    memcpy(dev->last_rx_data, data, length);
    dev->rx_count++;

    /* All parsed responses in this version fit into packet 0. */
    if (packet != 0U)
    {
        return;
    }

    if (data[length - 1U] != ZDT_X42S_CHECK_BYTE)
    {
        return;
    }

    uint8_t function_code = data[0];
    dev->last_function = function_code;

    /* Common command response: [Function][Response][6B]. */
    if (ZDT_X42S_IsControlFunction(function_code) != 0U &&
        length >= 3U &&
        ZDT_X42S_IsResponseCode(data[1]) != 0U)
    {
        dev->last_response = (ZDT_X42S_Response_t)data[1];

        if (function_code == ZDT_X42S_CMD_POSITION &&
            data[1] == ZDT_X42S_RESP_COMPLETE)
        {
            dev->motor_status |= ZDT_X42S_MOTOR_STATUS_REACHED;
        }
        return;
    }

    /* 0x35: [35][Sign][Speed_H][Speed_L][6B] */
    if (function_code == ZDT_X42S_CMD_READ_SPEED && length >= 5U)
    {
        uint8_t sign = data[1];
        uint16_t raw_speed = ZDT_X42S_ReadU16BE(&data[2]);
        float speed;

        if (dev->firmware == ZDT_X42S_FW_EMM)
        {
            speed = (float)raw_speed;
        }
        else
        {
            speed = (float)raw_speed / 10.0f;
        }

        if (sign == 1U)
        {
            speed = -speed;
        }

        dev->speed_rpm = speed;
        dev->speed_valid = 1U;
        return;
    }

    /* 0x36: [36][Sign][P3][P2][P1][P0][6B] */
    if (function_code == ZDT_X42S_CMD_READ_POSITION && length >= 7U)
    {
        uint8_t sign = data[1];
        uint32_t raw_position = ZDT_X42S_ReadU32BE(&data[2]);
        float position;

        if (dev->firmware == ZDT_X42S_FW_EMM)
        {
            position = ((float)raw_position * 360.0f) / 65536.0f;
        }
        else
        {
            position = (float)raw_position / 10.0f;
        }

        if (sign == 1U)
        {
            position = -position;
        }

        dev->position_deg = position;
        dev->position_valid = 1U;
        return;
    }

    /* 0x3A: [3A][MotorStatus][6B] */
    if (function_code == ZDT_X42S_CMD_READ_MOTOR_STATUS && length >= 3U)
    {
        dev->motor_status = data[1];
        dev->motor_status_valid = 1U;
        return;
    }

    /* 0x3B: [3B][HomeStatus][6B] */
    if (function_code == ZDT_X42S_CMD_READ_HOME_STATUS && length >= 3U)
    {
        dev->home_status = data[1];
        dev->home_status_valid = 1U;
        return;
    }

    /* 0x3C: [3C][HomeStatus][MotorStatus][6B] */
    if (function_code == ZDT_X42S_CMD_READ_ALL_STATUS && length >= 4U)
    {
        dev->home_status = data[1];
        dev->motor_status = data[2];
        dev->home_status_valid = 1U;
        dev->motor_status_valid = 1U;
        return;
    }
}

float ZDT_X42S_GetSpeedRPM(const ZDT_X42S_Handle_t *dev)
{
    return (dev != NULL) ? dev->speed_rpm : 0.0f;
}

float ZDT_X42S_GetPositionDeg(const ZDT_X42S_Handle_t *dev)
{
    return (dev != NULL) ? dev->position_deg : 0.0f;
}

uint8_t ZDT_X42S_IsEnabled(const ZDT_X42S_Handle_t *dev)
{
    if (dev == NULL || dev->motor_status_valid == 0U) return 0U;
    return ((dev->motor_status & ZDT_X42S_MOTOR_STATUS_ENABLED) != 0U) ? 1U : 0U;
}

uint8_t ZDT_X42S_IsReached(const ZDT_X42S_Handle_t *dev)
{
    if (dev == NULL) return 0U;
    return ((dev->motor_status & ZDT_X42S_MOTOR_STATUS_REACHED) != 0U) ? 1U : 0U;
}

uint8_t ZDT_X42S_IsHoming(const ZDT_X42S_Handle_t *dev)
{
    if (dev == NULL || dev->home_status_valid == 0U) return 0U;
    return ((dev->home_status & ZDT_X42S_HOME_STATUS_RUNNING) != 0U) ? 1U : 0U;
}

uint8_t ZDT_X42S_IsHomeFailed(const ZDT_X42S_Handle_t *dev)
{
    if (dev == NULL || dev->home_status_valid == 0U) return 0U;
    return ((dev->home_status & ZDT_X42S_HOME_STATUS_FAILED) != 0U) ? 1U : 0U;
}

ZDT_X42S_Response_t ZDT_X42S_GetLastResponse(const ZDT_X42S_Handle_t *dev)
{
    return (dev != NULL) ? dev->last_response : ZDT_X42S_RESP_NONE;
}
