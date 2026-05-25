/**
 * @file localization.h
 * @brief 定位与导航系统
 * 
 * 使用二维码进行位置更新，结合编码器进行位姿估计
 */

#ifndef __LOCALIZATION_H
#define __LOCALIZATION_H

#include "main.h"
#include "qr_comm.h"
#include <stdint.h>
#include <stdbool.h>

/* 机器人状态 */
typedef struct {
    float x;           /* 世界坐标X (cm) */
    float y;           /* 世界坐标Y (cm) */
    float theta;       /* 朝向角 (弧度) */
    uint32_t timestamp;
} RobotPose_t;

/* 导航目标 */
typedef struct {
    float target_x;
    float target_y;
    float tolerance;   /* 到达距离阈值 (cm) */
    bool active;
} NavigationTarget_t;

/* 控制器参数 */
typedef struct {
    float kp_linear;   /* 线速度比例系数 */
    float kd_linear;   /* 线速度微分系数 */
    float kp_angular;  /* 角速度比例系数 */
    float kd_angular;  /* 角速度微分系数 */
} PIDParameters_t;

/**
 * @brief 初始化定位系统
 */
void Localization_Init(void);

/**
 * @brief 用二维码数据更新位置
 * @param qr: 二维码数据结构
 */
void Update_Position_From_QR(const QR_Data_t *qr);

/**
 * @brief 使用PnP从角点估计相机/机器人位姿（若可用）
 */
bool Update_Pose_From_QR_PnP(const QR_Data_t *qr);

/**
 * @brief 配置相机内参（像素域）
 */
void Set_Camera_Intrinsics(float fx, float fy, float cx, float cy);

/**
 * @brief 设置二维码实际边长（厘米）
 */
void Set_Marker_Size(float size_cm);

/**
 * @brief 获取当前机器人位姿
 */
RobotPose_t* Get_Robot_Pose(void);

/**
 * @brief 设置导航目标
 */
void Set_Navigation_Target(float x, float y, float tolerance);

/**
 * @brief 停止导航
 */
void Stop_Navigation(void);

/**
 * @brief 检查是否到达目标
 */
bool Check_Target_Reached(void);

/**
 * @brief 导航控制 - 使用PD控制器
 * @return true 表示已到达目标
 */
bool Navigate_Update(void);

/**
 * @brief 检查是否已启用定位
 */
bool Is_Positioning_Enabled(void);

/**
 * @brief 计算机器人到目标的距离和角度
 */
void Calculate_Navigation_Error(float *distance, float *angle_error);

/**
 * @brief 设置PID参数
 */
void Set_PID_Parameters(const PIDParameters_t *params);

/**
 * @brief 获取当前位置（用于Wi-Fi发送）
 */
typedef struct {
    float x;
    float y;
    float heading;
} Position_t;

Position_t Get_Current_Position(void);

#endif /* __LOCALIZATION_H */
