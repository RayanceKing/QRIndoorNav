/**
 * @file qr_comm.h
 * @brief K210 通信与二维码数据处理
 * 
 * 数据格式：$POSE,id,tx,ty,tz,rx,ry,rz\r\n
 * 例：$POSE,0,35,-22,-109,233,18,29\r\n
 */

#ifndef __QR_COMM_H
#define __QR_COMM_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* 二维码数据结构（与 qr_comm.c 实现匹配） */
typedef struct {
    char id[16];           /* 二维码标识字符串 */
    float world_x;         /* 世界坐标 X（cm 或 与 K210 约定的单位） */
    float world_y;         /* 世界坐标 Y */
    float heading_deg;     /* K210融合后的航向角（度），仅 pose_valid=true 时有效 */
    uint16_t corner_x[4];  /* 四个角点的像素 x 坐标 */
    uint16_t corner_y[4];  /* 四个角点的像素 y 坐标 */
    bool corners_valid;    /* 是否包含有效角点数据 */
    bool pose_valid;       /* 是否为 $POS 直接位姿包 */
    uint32_t timestamp;    /* 接收时间戳 */
} QR_Data_t;

/* 通信缓冲区 */
typedef struct {
    uint8_t buffer[256];
    uint16_t length;
    bool data_ready;
} UART_Buffer_t;

/**
 * @brief K210 连接状态
 */
bool QR_Comm_Is_K210_Ready(void);

/**
 * @brief 上次心跳的时间戳（ms）
 */
uint32_t QR_Comm_Last_Heartbeat_Tick(void);

/**
 * @brief 链路是否在指定超时时间内保持心跳
 */
bool QR_Comm_Link_Alive(uint32_t timeout_ms);

/**
 * @brief 初始化UART通信
 */
void QR_Comm_Init(void);

/**
 * @brief 启用DMA接收中断
 */
void QR_Comm_Start_Receive(void);

/**
 * @brief 处理接收到的数据包
 * @return true 如果成功解析了完整数据包
 */
bool QR_Comm_Process(QR_Data_t *qr_data);

/**
 * @brief 获取最新的二维码数据
 * @return 指向最新QR_Data_t的指针，若无新数据则返回NULL
 */
QR_Data_t* QR_Comm_Get_Latest(void);

/**
 * @brief 检查是否有新数据
 */
bool QR_Comm_Has_New_Data(void);

/**
 * @brief 清空接收缓冲区
 */
void QR_Comm_Clear_Buffer(void);

/**
 * @brief UART空闲中断回调（由HAL调用）
 */
void QR_UART_Idle_Callback(void);

#endif /* __QR_COMM_H */
