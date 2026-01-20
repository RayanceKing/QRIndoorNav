/**
 * @file motor.h
 * @brief TB6612 四路电机驱动模块
 * 
 * 电机映射：
 * Motor A: PA6  (TIM3_CH1) - 右前
 * Motor B: PA7  (TIM3_CH2) - 左前
 * Motor C: PB0  (TIM3_CH3) - 左后
 * Motor D: PB1  (TIM3_CH4) - 右后
 */

#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"
#include <stdint.h>

/* 电机通道定义 */
#define MOTOR_A     0  // 右前
#define MOTOR_B     1  // 左前
#define MOTOR_C     2  // 左后
#define MOTOR_D     3  // 右后

/* PWM常量 */
#define PWM_MAX     999
#define PWM_MIN    -999

/* 方向控制引脚映射 */
typedef struct {
    GPIO_TypeDef *in1_port;
    uint16_t      in1_pin;
    GPIO_TypeDef *in2_port;
    uint16_t      in2_pin;
} MotorPins_t;

/**
 * @brief 电机初始化
 */
void Motor_Init(void);

/**
 * @brief 设置单个电机速度
 * @param motor_id: MOTOR_A/B/C/D
 * @param speed: -999~999，负数为反向
 */
void Motor_SetSpeed(uint8_t motor_id, int16_t speed);

/**
 * @brief 停止所有电机
 */
void Motor_Stop_All(void);

/**
 * @brief 麦克纳姆轮全向移动
 * @param vx: 前后速度 (-999~999)
 * @param vy: 左右速度 (-999~999)
 * @param omega: 旋转速度 (-999~999)
 * 
 * 麦轮速度映射公式：
 * vA = vx - vy - omega  (右前)
 * vB = vx + vy + omega  (左前)
 * vC = vx - vy + omega  (左后)
 * vD = vx + vy - omega  (右后)
 */
void Mecanum_Move(float vx, float vy, float omega);

#endif /* __MOTOR_H */
