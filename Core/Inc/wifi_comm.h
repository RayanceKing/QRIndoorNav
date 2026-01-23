/**
 * @file wifi_comm.h
 * @brief DX-WF24-A Wi-Fi/蓝牙模块通信协议
 * 
 * 通信格式：
 * 手机->STM32:
 *   CMD:GOTO,x,y\r\n      - 前往目标坐标
 *   CMD:MOVE,dir,speed\r\n - 手动控制 (dir: F前/B后/L左/R右/TL左转/TR右转)
 *   CMD:STOP\r\n          - 停止
 *   CMD:QUERY\r\n         - 查询状态
 * 
 * STM32->手机:
 *   POS:x,y,heading\r\n   - 位置和航向
 *   STATUS:state\r\n      - 状态 (IDLE/NAV/MANUAL)
 */

#ifndef __WIFI_COMM_H
#define __WIFI_COMM_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* 命令类型 */
typedef enum {
    WIFI_CMD_NONE = 0,
    WIFI_CMD_GOTO,      /* 前往坐标 */
    WIFI_CMD_MOVE,      /* 手动移动 */
    WIFI_CMD_STOP,      /* 停止 */
    WIFI_CMD_QUERY      /* 查询状态 */
} WiFi_CmdType_t;

/* 移动方向 */
typedef enum {
    MOVE_FORWARD = 0,   /* 前进 */
    MOVE_BACKWARD,      /* 后退 */
    MOVE_LEFT,          /* 左移 */
    MOVE_RIGHT,         /* 右移 */
    MOVE_TURN_LEFT,     /* 左转 */
    MOVE_TURN_RIGHT     /* 右转 */
} Move_Direction_t;

/* 接收的命令数据 */
typedef struct {
    WiFi_CmdType_t type;
    union {
        struct {
            float target_x;
            float target_y;
        } goto_cmd;
        struct {
            Move_Direction_t direction;
            int16_t speed;  /* 0-999 */
        } move_cmd;
    };
} WiFi_Command_t;

/**
 * @brief Wi-Fi通信初始化
 */
void WiFi_Comm_Init(void);

/**
 * @brief 启动接收
 */
void WiFi_Comm_Start_Receive(void);

/**
 * @brief 处理接收的命令
 * @param cmd: 输出命令结构
 * @return true表示有新命令
 */
bool WiFi_Comm_Process(WiFi_Command_t *cmd);

/**
 * @brief 发送位置信息到手机
 * @param x, y: 当前坐标
 * @param heading: 航向角度
 */
void WiFi_Send_Position(float x, float y, float heading);

/**
 * @brief 发送状态信息
 * @param state: 状态字符串
 */
void WiFi_Send_Status(const char *state);

/**
 * @brief 检查Wi-Fi连接是否活跃
 * @param timeout_ms: 超时时间
 * @return true表示连接活跃
 */
bool WiFi_Link_Alive(uint32_t timeout_ms);

/**
 * @brief 检查透传模式是否就绪
 * @return true表示已进入透传模式，可以收发数据
 */
bool WiFi_Transparent_Mode_Ready(void);

/**
 * @brief 输出调试信息（在主循环中调用）
 */
void WiFi_Print_Debug(void);

#endif /* __WIFI_COMM_H */
