#include "SCANNER.h"

LineSensor_t line_sensor;

/*
 * 从左到右的16路权重
 *
 * 左边为正
 * 右边为负
 *
 * 中间接近0
 */
static const float line_weight[LINE_SENSOR_NUM] =
{
     3.0f,  2.4f,  1.8f,  1.3f,
     0.9f,  0.6f,  0.4f,  0.2f,

    -0.2f, -0.4f, -0.6f, -0.9f,
    -1.3f, -1.8f, -2.4f, -3.0f
};


/**
 * @brief 初始化循迹传感器
 */
/*初始化循迹传感器，清零所有状态变量*/
void LineSensor_Init(void)
{
    line_sensor.raw      = 0;
    line_sensor.error    = 0.0f;
    line_sensor.led_num  = 0;
    line_sensor.line_num = 0;
}


/**
 * @brief 读取16路循迹GPIO
 */
/*读取16路循迹传感器的GPIO状态，返回16位原始数据*/
uint16_t LineSensor_Read(void)
{
    uint16_t data = 0;

    data |= HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_14) << 15;
    data |= HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8)  << 14;
    data |= HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7)  << 13;
    data |= HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6)  << 12;
    data |= HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5)  << 11;
    data |= HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4)  << 10;
    data |= HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_7)  << 9;
    data |= HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15) << 8;
    data |= HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_5)  << 7;
    data |= HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_4)  << 6;
    data |= HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3)  << 5;
    data |= HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_1)  << 4;
    data |= HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_6)  << 3;
    data |= HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3)  << 2;
    data |= HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2)  << 1;
    data |= HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14) << 0;

    return data;
}


/**
 * @brief 根据16路灯状态计算循迹误差
 */
/*根据16路灯状态计算循迹误差，支持忽略边缘灯*/
float LineSensor_CalcError(
    uint16_t raw,
    uint8_t edge_ignore
)
{
    float error = 0.0f;
    uint8_t led_num = 0;

    /*
     * i = 权重数组下标
     *
     * bit15 -> weight[0]
     * bit14 -> weight[1]
     * ...
     * bit0  -> weight[15]
     */
    for (uint8_t i = edge_ignore;
         i < LINE_SENSOR_NUM - edge_ignore;
         i++)
    {
        uint8_t bit =
            (raw >> (LINE_SENSOR_NUM - 1 - i)) & 0x01;

        if (bit)
        {
            led_num++;

            error += line_weight[i];
        }
    }


    /*
     * 没有检测到线
     */
    if (led_num == 0)
    {
        return 0.0f;
    }


    /*
     * 多个灯同时检测到线时，
     * 使用平均权重作为线的位置
     */
    error /= (float)led_num;

    return error;
}


/**
 * @brief 一次完成循迹灯读取和误差计算
 */
/*一次完成循迹灯读取和误差计算，更新传感器状态并返回误差值*/
float LineSensor_Update(uint8_t edge_ignore)
{
    uint16_t raw;

    uint8_t led_num = 0;
    uint8_t line_num = 0;

    raw = LineSensor_Read();


    /* =============================
     * 统计亮灯数量
     * ============================= */

    for (uint8_t i = 0;
         i < LINE_SENSOR_NUM;
         i++)
    {
        if ((raw >> i) & 0x01)
        {
            led_num++;
        }
    }


    /* =============================
     * 统计连续线段数量
     * ============================= */

    uint8_t last = 0;

    for (uint8_t i = 0;
         i < LINE_SENSOR_NUM;
         i++)
    {
        uint8_t now =
            (raw >> i) & 0x01;

        /*
         * 0 -> 1
         *
         * 表示出现了一条新的线
         */
        if (now && !last)
        {
            line_num++;
        }

        last = now;
    }


    /* =============================
     * 保存状态
     * ============================= */

    line_sensor.raw = raw;

    line_sensor.led_num = led_num;

    line_sensor.line_num = line_num;

    line_sensor.error =
        LineSensor_CalcError(
            raw,
            edge_ignore
        );


    return line_sensor.error;
}