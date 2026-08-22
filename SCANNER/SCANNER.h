#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include "main.h"
#include <stdint.h>

#define LINE_SENSOR_NUM 16 /* 16路灯数量 */

typedef struct
{
    uint16_t raw;      /* 16路灯状态 */

    float error;       /* 加权后的循迹误差 */

    uint8_t led_num;   /* 亮灯数量 */

    uint8_t line_num;  /* 连续线段数量 */

} LineSensor_t;


extern LineSensor_t line_sensor;


/* 初始化 */
void LineSensor_Init(void);


/* 读取16路灯 */
uint16_t LineSensor_Read(void);


/* 根据灯状态计算误差 */
float LineSensor_CalcError(
    uint16_t raw,
    uint8_t edge_ignore
);


/* 一次完成：读取 + 统计 + 计算误差 */
float LineSensor_Update(uint8_t edge_ignore);


#endif
