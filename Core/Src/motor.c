/**
 * @file motor.c
 * @brief TB6612 四路电机驱动实现
 */

#include "motor.h"
#include "stm32f4xx_hal.h"
#include <math.h>
#include <stdio.h>

extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart3; /* 用于可选的串口调试输出 */

/* 电机引脚映射表 */
static const MotorPins_t motor_pins[4] = {
    /* Motor A - 左前 */
    {GPIOC, AIN1_Pin, GPIOC, AIN2_Pin},
    /* Motor B - 右前 */
    {GPIOD, BIN1_Pin, GPIOD, BIN2_Pin},
    /* Motor C - 左后 */
    {GPIOC, CIN1_Pin, GPIOG, CIN2_Pin},
    /* Motor D - 右后 */
    {GPIOD, DIN1_Pin, GPIOC, DIN2_Pin},
};

/* 电机PWM通道映射 */
static const uint32_t pwm_channels[4] = {
    TIM_CHANNEL_1,  /* Motor A */
    TIM_CHANNEL_2,  /* Motor B */
    TIM_CHANNEL_3,  /* Motor C */
    TIM_CHANNEL_4,  /* Motor D */
};

/**
 * @brief 电机初始化
 */
void Motor_Init(void)
{
    /* 启动PWM输出 */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    
    /* 使能驱动（STBY拉高） */
    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);
    
    /* 初始化所有电机为停止状态 */
    Motor_Stop_All();
}

/**
 * @brief 设置单个电机速度
 * @param motor_id: 0-3 对应 A/B/C/D
 * @param speed: -999~999
 */
void Motor_SetSpeed(uint8_t motor_id, int16_t speed)
{
    if (motor_id >= 4) return;
    
    /* 限制速度范围 */
    if (speed > PWM_MAX) speed = PWM_MAX;
    if (speed < PWM_MIN) speed = PWM_MIN;
    
    const MotorPins_t *pins = &motor_pins[motor_id];
    
    if (speed > 0) {
        /* 正向：IN1=1, IN2=0 */
        HAL_GPIO_WritePin(pins->in1_port, pins->in1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(pins->in2_port, pins->in2_pin, GPIO_PIN_RESET);
    } else if (speed < 0) {
        /* 反向：IN1=0, IN2=1 */
        HAL_GPIO_WritePin(pins->in1_port, pins->in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(pins->in2_port, pins->in2_pin, GPIO_PIN_SET);
        speed = -speed;
    } else {
        /* 停止：IN1=0, IN2=0 */
        HAL_GPIO_WritePin(pins->in1_port, pins->in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(pins->in2_port, pins->in2_pin, GPIO_PIN_RESET);
        speed = 0;
    }
    
    /* 设置PWM占空比 */
    __HAL_TIM_SET_COMPARE(&htim3, pwm_channels[motor_id], speed);

#ifdef MOTOR_DEBUG_UART
    /* 发送调试信息到串口，便于验证物理引脚输出 */
    char dbg[64];
    int in1 = (HAL_GPIO_ReadPin(pins->in1_port, pins->in1_pin) == GPIO_PIN_SET) ? 1 : 0;
    int in2 = (HAL_GPIO_ReadPin(pins->in2_port, pins->in2_pin) == GPIO_PIN_SET) ? 1 : 0;
    int len = snprintf(dbg, sizeof(dbg), "M%d IN1=%d IN2=%d PWM=%d\r\n", motor_id, in1, in2, speed);
    if (len > 0) {
        HAL_UART_Transmit(&huart3, (uint8_t *)dbg, (uint16_t)len, 5);
    }
#endif
}

/**
 * @brief 停止所有电机
 */
void Motor_Stop_All(void)
{
    for (uint8_t i = 0; i < 4; i++) {
        Motor_SetSpeed(i, 0);
    }
}

/**
 * @brief 麦克纳姆轮全向移动
 */
void Mecanum_Move(float vx, float vy, float omega)
{
    /* 麦克纳姆轮运动学模型 */
    float vA = vx + vy + omega;  /* 左前 */
    float vB = vx - vy - omega;  /* 右前 */
    float vC = vx - vy + omega;  /* 左后 */
    float vD = vx + vy - omega;  /* 右后 */
    
    /* 归一化到[-999, 999]范围 */
    float max_speed = 0;
    max_speed = fabs(vA) > max_speed ? fabs(vA) : max_speed;
    max_speed = fabs(vB) > max_speed ? fabs(vB) : max_speed;
    max_speed = fabs(vC) > max_speed ? fabs(vC) : max_speed;
    max_speed = fabs(vD) > max_speed ? fabs(vD) : max_speed;
    
    if (max_speed > PWM_MAX) {
        vA = vA * PWM_MAX / max_speed;
        vB = vB * PWM_MAX / max_speed;
        vC = vC * PWM_MAX / max_speed;
        vD = vD * PWM_MAX / max_speed;
    }
    
    Motor_SetSpeed(MOTOR_A, (int16_t)vA);
    Motor_SetSpeed(MOTOR_B, (int16_t)vB);
    Motor_SetSpeed(MOTOR_C, (int16_t)vC);
    Motor_SetSpeed(MOTOR_D, (int16_t)vD);
}
