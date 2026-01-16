/**
 * @file localization.c
 * @brief 定位与导航系统实现
 */

#include "localization.h"
#include "motor.h"
#include <math.h>
#include <stdio.h>

static RobotPose_t robot_pose = {0.0f, 0.0f, 0.0f, 0};
static NavigationTarget_t nav_target = {0};
static PIDParameters_t pid_params = {
    .kp_linear = 0.5f,
    .kd_linear = 0.1f,
    .kp_angular = 0.3f,
    .kd_angular = 0.05f,
};

static float last_linear_error = 0.0f;
static float last_angular_error = 0.0f;

/**
 * @brief 初始化定位系统
 */
void Localization_Init(void)
{
    robot_pose.x = 0.0f;
    robot_pose.y = 0.0f;
    robot_pose.theta = 0.0f;
    robot_pose.timestamp = HAL_GetTick();
    
    nav_target.active = false;
    nav_target.tolerance = 5.0f;
}

/**
 * @brief 角度归一化到 [-π, π]
 */
static float Normalize_Angle(float angle)
{
    while (angle > M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

/**
 * @brief 用二维码数据更新位置
 * 简化假设：二维码提供的是世界坐标，使用二维码中心作为位置
 */
void Update_Position_From_QR(const QR_Data_t *qr)
{
    if (qr == NULL) return;
    
    /* 使用二维码世界坐标作为机器人位置 */
    robot_pose.x = qr->world_x;
    robot_pose.y = qr->world_y;
    
    /* 根据四个角点计算朝向角
     * 简化方法：使用角点0和1构成的边计算朝向
     */
    float dx = qr->corner_x[1] - qr->corner_x[0];
    float dy = qr->corner_y[1] - qr->corner_y[0];
    robot_pose.theta = atan2f(dy, dx);
    
    robot_pose.timestamp = HAL_GetTick();
}

/**
 * @brief 获取当前机器人位姿
 */
RobotPose_t* Get_Robot_Pose(void)
{
    return &robot_pose;
}

/**
 * @brief 设置导航目标
 */
void Set_Navigation_Target(float x, float y, float tolerance)
{
    nav_target.target_x = x;
    nav_target.target_y = y;
    nav_target.tolerance = tolerance;
    nav_target.active = true;
    
    last_linear_error = 0.0f;
    last_angular_error = 0.0f;
}

/**
 * @brief 停止导航
 */
void Stop_Navigation(void)
{
    nav_target.active = false;
    Motor_Stop_All();
}

/**
 * @brief 检查是否到达目标
 */
bool Check_Target_Reached(void)
{
    if (!nav_target.active) return false;
    
    float dx = nav_target.target_x - robot_pose.x;
    float dy = nav_target.target_y - robot_pose.y;
    float distance = sqrtf(dx * dx + dy * dy);
    
    return distance < nav_target.tolerance;
}

/**
 * @brief 计算机器人到目标的距离和角度
 */
void Calculate_Navigation_Error(float *distance, float *angle_error)
{
    float dx = nav_target.target_x - robot_pose.x;
    float dy = nav_target.target_y - robot_pose.y;
    
    *distance = sqrtf(dx * dx + dy * dy);
    float target_theta = atan2f(dy, dx);
    *angle_error = Normalize_Angle(target_theta - robot_pose.theta);
}

/**
 * @brief 导航控制 - PD控制器
 */
bool Navigate_Update(void)
{
    if (!nav_target.active) {
        return false;
    }
    
    /* 计算导航误差 */
    float distance, angle_error;
    Calculate_Navigation_Error(&distance, &angle_error);
    
    /* 检查是否到达 */
    if (distance < nav_target.tolerance) {
        Motor_Stop_All();
        nav_target.active = false;
        return true;
    }
    
    /* PD控制器计算速度命令 */
    /* 线速度控制：距离越大，速度越大 */
    float linear_control = pid_params.kp_linear * distance + 
                          pid_params.kd_linear * (distance - last_linear_error);
    
    /* 角速度控制：角度误差越大，旋转越快 */
    float angular_control = pid_params.kp_angular * angle_error + 
                           pid_params.kd_angular * (angle_error - last_angular_error);
    
    /* 限制速度 */
    if (linear_control > 500.0f) linear_control = 500.0f;
    if (linear_control < 50.0f && distance > nav_target.tolerance) linear_control = 50.0f;
    
    if (angular_control > 300.0f) angular_control = 300.0f;
    if (angular_control < -300.0f) angular_control = -300.0f;
    
    /* 保存误差用于微分 */
    last_linear_error = distance;
    last_angular_error = angle_error;
    
    /* 发送控制命令到麦轮 */
    /* vx: 前进速度，omega: 旋转速度 */
    Mecanum_Move(linear_control, 0.0f, angular_control);
    
    return false;
}

/**
 * @brief 设置PID参数
 */
void Set_PID_Parameters(const PIDParameters_t *params)
{
    if (params != NULL) {
        pid_params = *params;
    }
}
