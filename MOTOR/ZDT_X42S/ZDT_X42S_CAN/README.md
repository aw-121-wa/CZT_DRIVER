# ZDT X42S CAN 驱动

基于 STM32 HAL FDCAN 接口的 ZDT X42S 步进电机 CAN 自由协议驱动，支持 EMM 和 X 两种固件版本。

## 协议概述

### CAN 帧格式

- **帧类型**: Classic CAN（非 CAN FD）
- **ID 类型**: Extended ID（29 位）
- **CAN ID 编码**: `ID = (电机地址 << 8) | 包序号`
  - 高字节：电机地址（0x00 为广播地址）
  - 低字节：分包序号（从 0 递增）

### 数据帧结构

每帧 data 区域最大 8 字节，结构如下：

```
[功能码(1B)] [参数(N字节)] [0x6B校验]
```

- `data[0]` 固定为功能码
- 每帧最多携带 **7 字节有效载荷**（1 功能码 + 6 参数 + 1 校验）
- 超过 7 字节的命令自动拆分为多帧，包序号递增
- 末尾固定追加校验字节 `0x6B`

### 两种固件版本

| 特性 | EMM 固件 | X 固件 |
|------|---------|--------|
| 速度单位 | 整数 RPM（0~3000） | 0.1 RPM（0~3000.0） |
| 位置单位 | 脉冲数 | 0.1 度 |
| 加速度 | 0~255 级 | RPM/s |
| 位置命令 | 10 字节 | 13 字节（含独立加减速） |

## 状态位定义

### 电机状态 (`motor_status`)

| 位 | 宏定义 | 含义 |
|----|--------|------|
| 0 | `ZDT_X42S_STATUS_ENABLED` | 已使能 |
| 1 | `ZDT_X42S_STATUS_REACHED` | 目标到达 |
| 2 | `ZDT_X42S_STATUS_STALL` | 堵转 |
| 3 | `ZDT_X42S_STATUS_STALL_PROTECT` | 堵转保护触发 |

### 回零状态 (`home_status`)

| 位 | 宏定义 | 含义 |
|----|--------|------|
| 2 | `ZDT_X42S_HOME_RUNNING` | 回零进行中 |
| 3 | `ZDT_X42S_HOME_FAILED` | 回零失败 |

## API 参考

### 初始化

```c
// 初始化电机句柄
HAL_StatusTypeDef ZDT_X42S_Init(
    ZDT_X42S_t *motor,           // 电机句柄
    FDCAN_HandleTypeDef *hfdcan, // FDCAN 外设句柄
    uint8_t address,             // 电机地址 (1~255)
    ZDT_X42S_Firmware_t firmware // 固件类型: ZDT_X42S_FW_EMM / ZDT_X42S_FW_X
);

// 设置 EMM 固件每转脉冲数（默认 3200）
void ZDT_X42S_SetPulsesPerRev(ZDT_X42S_t *motor, uint32_t ppr);
```

### 控制命令

```c
// 使能/失能单个电机（即时执行）
HAL_StatusTypeDef ZDT_X42S_Enable(ZDT_X42S_t *m, bool enable);

// 使能/失能多个电机（自动缓冲 + 广播同步）
HAL_StatusTypeDef ZDT_X42S_EnableAll(ZDT_X42S_t motors[], uint8_t count, bool enable);

// 设置单个电机转速（即时执行）
//   rpm:   负值=反转
//   accel: X固件=RPM/s, EMM固件=0~255
HAL_StatusTypeDef ZDT_X42S_SetSpeed(ZDT_X42S_t *m, float rpm, uint16_t accel);

// 同步设置多个电机转速（自动缓冲 + 广播同步）
HAL_StatusTypeDef ZDT_X42S_SetSpeeds(ZDT_X42S_t motors[], uint8_t count,
                                      const float rpm[], uint16_t accel);

// 按角度移动（即时执行，自动适配固件格式）
//   degrees: 负值=反转
//   mode:    ZDT_X42S_POS_REL_LAST / ZDT_X42S_POS_ABS_ZERO / ZDT_X42S_POS_REL_NOW
HAL_StatusTypeDef ZDT_X42S_MoveDeg(ZDT_X42S_t *m, float degrees, float max_rpm,
                                    uint16_t accel, ZDT_X42S_PositionMode_t mode);

// 停止
HAL_StatusTypeDef ZDT_X42S_Stop(ZDT_X42S_t *m);
HAL_StatusTypeDef ZDT_X42S_StopAll(ZDT_X42S_t motors[], uint8_t count);

// 回零
//   mode: ZDT_X42S_HOME_NEAREST / DIR / COLLIDE / LIMIT / ZERO / POWER
HAL_StatusTypeDef ZDT_X42S_Home(ZDT_X42S_t *m, ZDT_X42S_HomeMode_t mode);
HAL_StatusTypeDef ZDT_X42S_AbortHome(ZDT_X42S_t *m);

// 手动触发广播同步（高级用法）
HAL_StatusTypeDef ZDT_X42S_TriggerSync(ZDT_X42S_t *m);
```

### 查询命令

```c
// 发送查询请求（回复通过 ProcessRx 异步解析）
HAL_StatusTypeDef ZDT_X42S_ReadSpeed(ZDT_X42S_t *m);
HAL_StatusTypeDef ZDT_X42S_ReadPosition(ZDT_X42S_t *m);
HAL_StatusTypeDef ZDT_X42S_ReadStatus(ZDT_X42S_t *m);
HAL_StatusTypeDef ZDT_X42S_ReadHomeStatus(ZDT_X42S_t *m);
HAL_StatusTypeDef ZDT_X42S_ReadAllStatus(ZDT_X42S_t *m);
```

### 接收处理

```c
// 解析单帧 CAN 回复，自动匹配地址并更新电机状态
bool ZDT_X42S_ProcessRx(ZDT_X42S_t motors[], uint8_t count,
                        const FDCAN_RxHeaderTypeDef *header,
                        const uint8_t data[8]);

// 从 FIFO 中取出所有帧并逐帧解析（适合在 RxFifo Callback 中调用）
uint32_t ZDT_X42S_DrainRxFifo(ZDT_X42S_t motors[], uint8_t count, uint32_t fifo);

// 判断电机是否在线（超时时间内收到过合法回复）
bool ZDT_X42S_IsOnline(const ZDT_X42S_t *motor, uint32_t timeout_ms);
```

### 低级接口

```c
// 发送原始协议命令（用于驱动未覆盖的自定义命令）
//   payload 不含功能码和 0x6B 校验，驱动自动追加
HAL_StatusTypeDef ZDT_X42S_SendRaw(ZDT_X42S_t *m, uint8_t code,
                                    const uint8_t *payload, uint16_t payload_len);
```

## 使用示例

### 基本用法

```c
#include "zdt_x42s.h"

ZDT_X42S_t motor;

void motor_init(void)
{
    // 初始化: FDCAN1, 地址=1, X固件
    ZDT_X42S_Init(&motor, &hfdcan1, 1, ZDT_X42S_FW_X);

    // 使能
    ZDT_X42S_Enable(&motor, true);

    // 正转 500.0 RPM, 加速度 1000 RPM/s
    ZDT_X42S_SetSpeed(&motor, 500.0f, 1000);

    // 移动 90 度, 最大 300 RPM, 加速度 500 RPM/s
    ZDT_X42S_MoveDeg(&motor, 90.0f, 300.0f, 500, ZDT_X42S_POS_REL_NOW);
}

// 在 FDCAN 接收中断回调中
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    ZDT_X42S_DrainRxFifo(&motor, 1, FDCAN_RX_FIFO0);
}

// 读取状态
void check_status(void)
{
    ZDT_X42S_ReadAllStatus(&motor);

    // 等待回复后读取（或在主循环中轮询）
    if (motor.has_reply) {
        float rpm  = motor.speed_rpm;
        float deg  = motor.position_deg;
        bool  on   = (motor.motor_status & ZDT_X42S_STATUS_ENABLED) != 0;
        bool done  = (motor.motor_status & ZDT_X42S_STATUS_REACHED) != 0;
    }
}
```

### 多电机同步

```c
#define MOTOR_COUNT 3
ZDT_X42S_t motors[MOTOR_COUNT];

void multi_motor_init(void)
{
    // 所有电机必须在同一 FDCAN 总线上
    for (int i = 0; i < MOTOR_COUNT; i++)
        ZDT_X42S_Init(&motors[i], &hfdcan1, i + 1, ZDT_X42S_FW_X);

    // 同步使能（自动缓冲 + 广播同步触发）
    ZDT_X42S_EnableAll(motors, MOTOR_COUNT, true);

    // 同步设置不同转速
    float rpms[] = {100.0f, 200.0f, -150.0f};
    ZDT_X42S_SetSpeeds(motors, MOTOR_COUNT, rpms, 500);

    // 同步停止
    ZDT_X42S_StopAll(motors, MOTOR_COUNT);
}
```

## 注意事项

1. **地址匹配**: 接收时自动从 Extended ID 高字节提取地址，匹配对应电机句柄
2. **多电机同步**: `EnableAll`/`SetSpeeds`/`StopAll` 要求所有电机在同一 FDCAN 总线上
3. **RX 处理**: 必须在 FDCAN 接收中断中调用 `DrainRxFifo` 或 `ProcessRx`，否则状态不会更新
4. **volatile 字段**: `speed_rpm`、`position_deg`、`motor_status` 等字段在中断中更新，主循环读取时无需关中断
5. **在线检测**: `IsOnline()` 基于 `HAL_GetTick()` 判断超时，需确保 SysTick 正常工作
