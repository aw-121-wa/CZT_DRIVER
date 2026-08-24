#include "zdt_x42s.h"
#include <string.h>

/* 协议固定尾部校验字节 */
#define ZDT_TAIL 0x6BU

/* 功能码定义 */
#define CMD_ENABLE      0xF3U   /* 使能/失能 */
#define CMD_SPEED       0xF6U   /* 速度模式 */
#define CMD_POSITION    0xFDU   /* 位置模式 */
#define CMD_STOP        0xFEU   /* 停止 */
#define CMD_SYNC        0xFFU   /* 同步触发(广播) */
#define CMD_HOME        0x9AU   /* 回零 */
#define CMD_ABORT_HOME  0x9CU   /* 中止回零 */
#define CMD_READ_SPEED  0x35U   /* 读取转速 */
#define CMD_READ_POS    0x36U   /* 读取位置 */
#define CMD_READ_STATUS 0x3AU   /* 读取电机状态 */
#define CMD_READ_HOME   0x3BU   /* 读取回零状态 */
#define CMD_READ_ALL    0x3CU   /* 读取全部状态 */

/* 字节数量到 FDCAN DLC 编码的查找表 */
static const uint32_t s_dlc[9] = {
    FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2,
    FDCAN_DLC_BYTES_3, FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5,
    FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7, FDCAN_DLC_BYTES_8
};

/**
 * @brief 将 16 位值按大端序写入缓冲区
 */
static void put16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)v;
}

/**
 * @brief 将 32 位值按大端序写入缓冲区
 */
static void put32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v >> 24);
    p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);
    p[3] = (uint8_t)v;
}

/**
 * @brief 从缓冲区按大端序读取 16 位值
 */
static uint16_t get16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

/**
 * @brief 从缓冲区按大端序读取 32 位值
 */
static uint32_t get32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8)  |  (uint32_t)p[3];
}

/**
 * @brief 计算给定载荷长度需要的 CAN 分包数量
 * @param payload_len 载荷字节数（不含功能码和校验字节）
 * @return 所需 CAN 帧数（每帧最多 7 字节新数据，末尾自动追加 0x6B）
 */
static uint16_t packet_count(uint16_t payload_len)
{
    return (uint16_t)(((uint32_t)payload_len + 7U) / 7U);
}

/**
 * @brief 检查电机数组是否全部挂载在同一个 FDCAN 总线上
 */
static bool same_bus(const ZDT_X42S_t motors[], uint8_t count)
{
    if (motors == NULL || count == 0U || motors[0].hfdcan == NULL)
        return false;

    for (uint8_t i = 1U; i < count; ++i)
        if (motors[i].hfdcan != motors[0].hfdcan)
            return false;

    return true;
}

/**
 * @brief 发送单个 CAN 数据帧
 * @param hfdcan  FDCAN 句柄
 * @param address 电机地址（高字节）
 * @param packet  包序号（低字节）
 * @param data    帧数据（含功能码）
 * @param len     帧数据长度（1~8）
 * @note  CAN ID = (address << 8) | packet，使用 Extended ID
 */
static HAL_StatusTypeDef send_packet(
    FDCAN_HandleTypeDef *hfdcan,
    uint8_t address,
    uint8_t packet,
    const uint8_t *data,
    uint8_t len)
{
    if (hfdcan == NULL || data == NULL || len == 0U || len > 8U)
        return HAL_ERROR;

    FDCAN_TxHeaderTypeDef h = {0};
    h.Identifier = ((uint32_t)address << 8) | packet;
    h.IdType = FDCAN_EXTENDED_ID;
    h.TxFrameType = FDCAN_DATA_FRAME;
    h.DataLength = s_dlc[len];
    h.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    h.BitRateSwitch = FDCAN_BRS_OFF;
    h.FDFormat = FDCAN_CLASSIC_CAN;
    h.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

    return HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &h, (uint8_t *)data);
}

/**
 * @brief 发送一条完整协议命令（自动分包 + 追加 0x6B 校验）
 * @param m            电机句柄
 * @param address      目标地址
 * @param code         功能码
 * @param payload      载荷数据（不含功能码和校验）
 * @param payload_len  载荷长度
 * @param check_fifo   是否在发送前检查 TX FIFO 空间
 * @note  协议格式: 每帧 data[0]=功能码, 最后一帧末尾=0x6B
 *        超过 7 字节的载荷会自动拆分成多帧，包序号递增
 */
static HAL_StatusTypeDef send_cmd(
    ZDT_X42S_t *m,
    uint8_t address,
    uint8_t code,
    const uint8_t *payload,
    uint16_t payload_len,
    bool check_fifo)
{
    if (m == NULL || m->hfdcan == NULL ||
        (payload_len != 0U && payload == NULL))
        return HAL_ERROR;

    uint16_t packets = packet_count(payload_len);
    if (check_fifo &&
        HAL_FDCAN_GetTxFifoFreeLevel(m->hfdcan) < packets)
        return HAL_BUSY;

    uint32_t offset = 0U;
    uint32_t total = (uint32_t)payload_len + 1U;
    uint8_t packet = 0U;

    while (offset < total) {
        uint8_t frame[8];
        uint8_t len = 1U;
        frame[0] = code;

        while (len < 8U && offset < total) {
            frame[len++] = (offset < payload_len)
                         ? payload[offset]
                         : ZDT_TAIL;
            ++offset;
        }

        HAL_StatusTypeDef st =
            send_packet(m->hfdcan, address, packet++, frame, len);
        if (st != HAL_OK)
            return st;
    }

    return HAL_OK;
}

/**
 * @brief 发送广播同步触发命令（地址 0x00，功能码 0xFF，辅助码 0x66）
 * @note  用于触发所有缓冲模式下已缓存的命令同时执行
 */
static HAL_StatusTypeDef sync(ZDT_X42S_t *m, bool check_fifo)
{
    const uint8_t p = 0x66U;
    return send_cmd(m, 0U, CMD_SYNC, &p, 1U, check_fifo);
}

/**
 * @brief 向一组电机发送简单命令（使能/停止等），多电机时自动缓冲+同步
 * @param motors  电机数组（必须在同一总线）
 * @param count   电机数量
 * @param code    功能码
 * @param aux     辅助码（如 0xAB=使能, 0x98=停止）
 * @param value   附加参数值，< 0 表示无此字节
 */
static HAL_StatusTypeDef simple_group(
    ZDT_X42S_t motors[],
    uint8_t count,
    uint8_t code,
    uint8_t aux,
    int value)
{
    if (!same_bus(motors, count))
        return HAL_ERROR;

    bool buffered = count > 1U;
    uint32_t needed = (uint32_t)count + (buffered ? 1U : 0U);

    if (HAL_FDCAN_GetTxFifoFreeLevel(motors[0].hfdcan) < needed)
        return HAL_BUSY;

    for (uint8_t i = 0U; i < count; ++i) {
        uint8_t p[3];
        uint8_t len;

        p[0] = aux;
        if (value >= 0) {
            p[1] = (uint8_t)value;
            p[2] = buffered ? 1U : 0U;
            len = 3U;
        } else {
            p[1] = buffered ? 1U : 0U;
            len = 2U;
        }

        HAL_StatusTypeDef st =
            send_cmd(&motors[i], motors[i].address, code, p, len, false);
        if (st != HAL_OK)
            return st;
    }

    return buffered ? sync(&motors[0], false) : HAL_OK;
}

/**
 * @brief 根据固件类型构建速度命令的载荷数据
 * @param m        电机句柄（用于判断固件类型）
 * @param rpm      目标转速（负值=反转）
 * @param accel    加速度（X: RPM/s, EMM: 0~255）
 * @param buffered 是否为缓冲模式
 * @param p        输出载荷缓冲区（至少 6 字节）
 * @param len      输出载荷实际长度
 * @return true=成功, false=参数越界
 * @note  X 固件: [Dir][Accel_H][Accel_L][Speed_H][Speed_L][Sync]  (6字节)
 *        EMM固件: [Dir][Speed_H][Speed_L][Accel][Sync]            (5字节)
 */
static bool build_speed(
    const ZDT_X42S_t *m,
    float rpm,
    uint16_t accel,
    bool buffered,
    uint8_t p[6],
    uint8_t *len)
{
    if (m == NULL || p == NULL || len == NULL)
        return false;

    uint8_t dir = rpm < 0.0f ? 1U : 0U;
    float mag = rpm < 0.0f ? -rpm : rpm;

    p[0] = dir;

    if (m->firmware == ZDT_X42S_FW_X) {
        uint32_t speed = (uint32_t)(mag * 10.0f + 0.5f);
        if (speed > 30000U)
            return false;

        put16(&p[1], accel);
        put16(&p[3], (uint16_t)speed);
        p[5] = buffered ? 1U : 0U;
        *len = 6U;
    } else {
        uint32_t speed = (uint32_t)(mag + 0.5f);
        if (speed > 3000U || accel > 255U)
            return false;

        put16(&p[1], (uint16_t)speed);
        p[3] = (uint8_t)accel;
        p[4] = buffered ? 1U : 0U;
        *len = 5U;
    }

    return true;
}


/* ================================================================== */
/*                            公共 API                                 */
/* ================================================================== */

/**
 * @brief 初始化电机句柄
 * @param m        电机句柄指针
 * @param hfdcan   FDCAN 外设句柄
 * @param address  电机 CAN 地址（1~255，0 为广播）
 * @param firmware 固件类型：ZDT_X42S_FW_EMM 或 ZDT_X42S_FW_X
 * @return HAL_OK=成功
 * @note  默认 pulses_per_rev=3200，可通过 ZDT_X42S_SetPulsesPerRev() 修改
 */
HAL_StatusTypeDef ZDT_X42S_Init(
    ZDT_X42S_t *m,
    FDCAN_HandleTypeDef *hfdcan,
    uint8_t address,
    ZDT_X42S_Firmware_t firmware)
{
    if (m == NULL || hfdcan == NULL ||
        (firmware != ZDT_X42S_FW_EMM && firmware != ZDT_X42S_FW_X))
        return HAL_ERROR;

    memset(m, 0, sizeof(*m));
    m->hfdcan = hfdcan;
    m->address = address;
    m->firmware = firmware;
    m->pulses_per_rev = 3200U;
    return HAL_OK;
}

/**
 * @brief 设置 EMM 固件下每转脉冲数（用于 MoveDeg 角度→脉冲换算）
 * @param m    电机句柄
 * @param ppr  每转脉冲数（默认 3200，不可为 0）
 */
void ZDT_X42S_SetPulsesPerRev(ZDT_X42S_t *m, uint32_t ppr)
{
    if (m != NULL && ppr != 0U)
        m->pulses_per_rev = ppr;
}

/**
 * @brief 发送原始协议命令（低级接口，不追加功能码和校验）
 * @param m           电机句柄
 * @param code        功能码
 * @param payload     载荷数据（不含功能码和 0x6B）
 * @param payload_len 载荷长度
 * @note  用于驱动未覆盖的自定义命令
 */
HAL_StatusTypeDef ZDT_X42S_SendRaw(
    ZDT_X42S_t *m,
    uint8_t code,
    const uint8_t *payload,
    uint16_t payload_len)
{
    if (m == NULL)
        return HAL_ERROR;
    return send_cmd(m, m->address, code, payload, payload_len, true);
}

/**
 * @brief 使能/失能单个电机（即时执行）
 * @param m      电机句柄
 * @param enable true=使能, false=失能
 */
HAL_StatusTypeDef ZDT_X42S_Enable(ZDT_X42S_t *m, bool enable)
{
    return simple_group(m, 1U, CMD_ENABLE, 0xABU, enable ? 1 : 0);
}

/**
 * @brief 使能/失能多个电机（自动缓冲+广播同步）
 * @param motors  电机数组（必须在同一总线）
 * @param count   电机数量
 * @param enable  true=使能, false=失能
 */
HAL_StatusTypeDef ZDT_X42S_EnableAll(
    ZDT_X42S_t motors[],
    uint8_t count,
    bool enable)
{
    return simple_group(motors, count, CMD_ENABLE, 0xABU, enable ? 1 : 0);
}

/**
 * @brief 设置单个电机转速（即时执行）
 * @param m     电机句柄
 * @param rpm   目标转速（负值=反转，单位: RPM）
 * @param accel 加速度（X 固件: RPM/s, EMM 固件: 0~255）
 */
HAL_StatusTypeDef ZDT_X42S_SetSpeed(
    ZDT_X42S_t *m,
    float rpm,
    uint16_t accel)
{
    return ZDT_X42S_SetSpeeds(m, 1U, &rpm, accel);
}

/**
 * @brief 同步设置多个电机转速（自动缓冲+广播同步）
 * @param motors 电机数组（必须在同一总线）
 * @param count  电机数量
 * @param rpm    各电机目标转速数组（负值=反转）
 * @param accel  加速度（所有电机共用）
 */
HAL_StatusTypeDef ZDT_X42S_SetSpeeds(
    ZDT_X42S_t motors[],
    uint8_t count,
    const float rpm[],
    uint16_t accel)
{
    if (!same_bus(motors, count) || rpm == NULL)
        return HAL_ERROR;

    bool buffered = count > 1U;
    uint32_t needed = (uint32_t)count + (buffered ? 1U : 0U);

    if (HAL_FDCAN_GetTxFifoFreeLevel(motors[0].hfdcan) < needed)
        return HAL_BUSY;

    for (uint8_t i = 0U; i < count; ++i) {
        uint8_t p[6], len;
        if (!build_speed(&motors[i], rpm[i], accel, buffered, p, &len))
            return HAL_ERROR;

        HAL_StatusTypeDef st =
            send_cmd(&motors[i], motors[i].address, CMD_SPEED, p, len, false);
        if (st != HAL_OK)
            return st;
    }

    return buffered ? sync(&motors[0], false) : HAL_OK;
}

/**
 * @brief 按角度移动电机（即时执行，自动根据固件类型选择协议格式）
 * @param m        电机句柄
 * @param degrees  目标角度（负值=反转，单位: 度）
 * @param max_rpm  最大转速（RPM）
 * @param accel    加速度（X 固件: RPM/s, EMM 固件: 0~255）
 * @param mode     位置模式（相对上次目标/绝对零点/相对当前位置）
 * @note  X 固件: 使用梯形位置命令，角度精度 0.1°
 *        EMM 固件: 自动将角度转换为脉冲数（基于 pulses_per_rev）
 */
HAL_StatusTypeDef ZDT_X42S_MoveDeg(
    ZDT_X42S_t *m,
    float degrees,
    float max_rpm,
    uint16_t accel,
    ZDT_X42S_PositionMode_t mode)
{
    if (m == NULL || m->hfdcan == NULL || mode > ZDT_X42S_POS_REL_NOW)
        return HAL_ERROR;

    uint8_t dir = degrees < 0.0f ? 1U : 0U;
    float deg = degrees < 0.0f ? -degrees : degrees;
    float rpm = max_rpm < 0.0f ? -max_rpm : max_rpm;

    if (m->firmware == ZDT_X42S_FW_X) {
        uint32_t angle = (uint32_t)(deg * 10.0f + 0.5f);
        uint32_t speed = (uint32_t)(rpm * 10.0f + 0.5f);
        uint8_t p[13];

        if (speed > 30000U)
            return HAL_ERROR;

        p[0] = dir;
        put16(&p[1], accel);       /* acceleration */
        put16(&p[3], accel);       /* deceleration */
        put16(&p[5], (uint16_t)speed);
        put32(&p[7], angle);
        p[11] = (uint8_t)mode;
        p[12] = 0U;                /* immediate */

        return send_cmd(m, m->address, CMD_POSITION, p, sizeof(p), true);
    } else {
        uint32_t speed = (uint32_t)(rpm + 0.5f);
        uint32_t pulses;
        uint8_t p[10];

        if (speed > 3000U || accel > 255U || m->pulses_per_rev == 0U)
            return HAL_ERROR;

        pulses = (uint32_t)(deg * (float)m->pulses_per_rev / 360.0f + 0.5f);

        p[0] = dir;
        put16(&p[1], (uint16_t)speed);
        p[3] = (uint8_t)accel;
        put32(&p[4], pulses);
        p[8] = (uint8_t)mode;
        p[9] = 0U;                 /* immediate */

        return send_cmd(m, m->address, CMD_POSITION, p, sizeof(p), true);
    }
}

/**
 * @brief 停止单个电机
 */
HAL_StatusTypeDef ZDT_X42S_Stop(ZDT_X42S_t *m)
{
    return simple_group(m, 1U, CMD_STOP, 0x98U, -1);
}

/**
 * @brief 停止多个电机（自动缓冲+广播同步）
 */
HAL_StatusTypeDef ZDT_X42S_StopAll(ZDT_X42S_t motors[], uint8_t count)
{
    return simple_group(motors, count, CMD_STOP, 0x98U, -1);
}

/**
 * @brief 执行回零操作
 * @param m     电机句柄
 * @param mode  回零模式（最近点/方向/碰撞/限位/绝对零点/上次断电位置）
 */
HAL_StatusTypeDef ZDT_X42S_Home(
    ZDT_X42S_t *m,
    ZDT_X42S_HomeMode_t mode)
{
    if (m == NULL || mode > ZDT_X42S_HOME_POWER)
        return HAL_ERROR;

    uint8_t p[2] = {(uint8_t)mode, 0U};
    return send_cmd(m, m->address, CMD_HOME, p, sizeof(p), true);
}

/**
 * @brief 中止正在进行的回零操作
 */
HAL_StatusTypeDef ZDT_X42S_AbortHome(ZDT_X42S_t *m)
{
    const uint8_t p = 0x48U;
    if (m == NULL)
        return HAL_ERROR;
    return send_cmd(m, m->address, CMD_ABORT_HOME, &p, 1U, true);
}

/**
 * @brief 手动触发广播同步（发送 0xFF 0x66 0x6B 到地址 0x00）
 * @note  用于高级场景：手动发送缓冲命令后统一触发
 */
HAL_StatusTypeDef ZDT_X42S_TriggerSync(ZDT_X42S_t *m)
{
    if (m == NULL)
        return HAL_ERROR;
    return sync(m, true);
}

/**
 * @brief 发送无载荷的查询命令（内部辅助函数）
 */
static HAL_StatusTypeDef read_cmd(ZDT_X42S_t *m, uint8_t code)
{
    if (m == NULL)
        return HAL_ERROR;
    return send_cmd(m, m->address, code, NULL, 0U, true);
}

/**
 * @brief 请求电机返回实时转速（回复通过 ProcessRx 解析）
 */
HAL_StatusTypeDef ZDT_X42S_ReadSpeed(ZDT_X42S_t *m)
{
    return read_cmd(m, CMD_READ_SPEED);
}

/**
 * @brief 请求电机返回实时位置（回复通过 ProcessRx 解析）
 */
HAL_StatusTypeDef ZDT_X42S_ReadPosition(ZDT_X42S_t *m)
{
    return read_cmd(m, CMD_READ_POS);
}

/**
 * @brief 请求电机返回电机状态字节（回复通过 ProcessRx 解析）
 */
HAL_StatusTypeDef ZDT_X42S_ReadStatus(ZDT_X42S_t *m)
{
    return read_cmd(m, CMD_READ_STATUS);
}

/**
 * @brief 请求电机返回回零状态字节（回复通过 ProcessRx 解析）
 */
HAL_StatusTypeDef ZDT_X42S_ReadHomeStatus(ZDT_X42S_t *m)
{
    return read_cmd(m, CMD_READ_HOME);
}

/**
 * @brief 请求电机返回全部状态（回零状态+电机状态，回复通过 ProcessRx 解析）
 */
HAL_StatusTypeDef ZDT_X42S_ReadAllStatus(ZDT_X42S_t *m)
{
    return read_cmd(m, CMD_READ_ALL);
}


/* ================================================================== */
/*                         接收处理 (RX)                               */
/* ================================================================== */

/**
 * @brief 在电机数组中按地址查找对应的句柄
 */
static ZDT_X42S_t *find_motor(
    ZDT_X42S_t motors[],
    uint8_t count,
    uint8_t address)
{
    if (motors == NULL)
        return NULL;

    for (uint8_t i = 0U; i < count; ++i)
        if (motors[i].address == address)
            return &motors[i];

    return NULL;
}

/**
 * @brief 解析一帧 CAN 回复数据并更新对应电机的状态
 * @param motors 电机数组（按地址匹配）
 * @param count  电机数量
 * @param h      FDCAN 接收帧头
 * @param data   帧数据（8 字节）
 * @return true=成功解析并更新, false=帧无效或地址不匹配
 * @note  自动从 Extended ID 提取地址，校验尾部 0x6B
 *        解析内容: 转速(0x35)、位置(0x36)、电机状态(0x3A)、
 *        回零状态(0x3B)、全部状态(0x3C)、控制命令响应码
 */
bool ZDT_X42S_ProcessRx(
    ZDT_X42S_t motors[],
    uint8_t count,
    const FDCAN_RxHeaderTypeDef *h,
    const uint8_t data[8])
{
    if (motors == NULL || count == 0U || h == NULL || data == NULL ||
        h->IdType != FDCAN_EXTENDED_ID ||
        h->RxFrameType != FDCAN_DATA_FRAME)
        return false;

    uint8_t address = (uint8_t)((h->Identifier >> 8) & 0xFFU);
    uint8_t packet = (uint8_t)(h->Identifier & 0xFFU);
    uint8_t len = (uint8_t)((h->DataLength >> 16) & 0x0FU);

    if (packet != 0U || len == 0U || len > 8U || data[len - 1U] != ZDT_TAIL)
        return false;

    ZDT_X42S_t *m = find_motor(motors, count, address);
    if (m == NULL)
        return false;

    switch (data[0]) {
        case CMD_READ_SPEED:
            if (len != 5U) return false;
            m->speed_rpm = (float)get16(&data[2]) /
                           (m->firmware == ZDT_X42S_FW_X ? 10.0f : 1.0f);
            if (data[1] == 1U) m->speed_rpm = -m->speed_rpm;
            break;

        case CMD_READ_POS:
            if (len != 7U) return false;
            {
                uint32_t raw = get32(&data[2]);
                m->position_deg = (m->firmware == ZDT_X42S_FW_X)
                    ? (float)raw / 10.0f
                    : (float)raw * 360.0f / 65536.0f;
                if (data[1] == 1U) m->position_deg = -m->position_deg;
            }
            break;

        case CMD_READ_STATUS:
            if (len != 3U) return false;
            m->motor_status = data[1];
            break;

        case CMD_READ_HOME:
            if (len != 3U) return false;
            m->home_status = data[1];
            break;

        case CMD_READ_ALL:
            if (len != 4U) return false;
            m->home_status = data[1];
            m->motor_status = data[2];
            break;

        default:
            if (len < 3U) return false;
            m->last_response = data[1];
            break;
    }

    m->last_reply_ms = HAL_GetTick();
    m->has_reply = 1U;
    return true;
}

/**
 * @brief 从 FDCAN 接收 FIFO 中取出所有帧并逐帧解析
 * @param motors 电机数组
 * @param count  电机数量
 * @param fifo   FIFO 索引（FDCAN_RX_FIFO0 或 FDCAN_RX_FIFO1）
 * @return 实际取出的帧数
 * @note  适合作为 HAL_FDCAN_RxFifo0Callback() 的处理入口
 */
uint32_t ZDT_X42S_DrainRxFifo(
    ZDT_X42S_t motors[],
    uint8_t count,
    uint32_t fifo)
{
    if (!same_bus(motors, count))
        return 0U;

    uint32_t n = 0U;
    FDCAN_RxHeaderTypeDef h;
    uint8_t data[8];

    while (HAL_FDCAN_GetRxFifoFillLevel(motors[0].hfdcan, fifo) > 0U) {
        if (HAL_FDCAN_GetRxMessage(motors[0].hfdcan, fifo, &h, data) != HAL_OK)
            break;

        (void)ZDT_X42S_ProcessRx(motors, count, &h, data);
        ++n;
    }

    return n;
}

/**
 * @brief 判断电机是否在线（在超时时间内收到过合法回复）
 * @param m          电机句柄
 * @param timeout_ms 超时时间（毫秒）
 * @return true=在线, false=离线或超时
 */
bool ZDT_X42S_IsOnline(const ZDT_X42S_t *m, uint32_t timeout_ms)
{
    return m != NULL && m->has_reply != 0U &&
           (HAL_GetTick() - m->last_reply_ms) <= timeout_ms;
}
