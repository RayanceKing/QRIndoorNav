/**
 * @file qr_comm.h
 * @brief K210 通信与二维码数据处理
 * 
 * 数据格式：$QR_ID,world_x,world_y|corner1_x,corner1_y|...|corner4_x,corner4_y\n
 * 例：$QR01,150.5,200.3|120,80|180,80|180,140|120,140\n
 */

#ifndef __QR_COMM_H
#define __QR_COMM_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* 二维码数据结构 */
typedef struct {
    char id[16];           /* 二维码ID */
    float world_x;         /* 世界坐标 X */
    float world_y;         /* 世界坐标 Y */
    uint16_t corner_x[4];  /* 四个角点的像素坐标 */
    uint16_t corner_y[4];
    uint32_t timestamp;    /* 接收时间戳 */
} QR_Data_t;

/* 通信缓冲区 */
typedef struct {
    uint8_t buffer[256];
    uint16_t length;
    bool data_ready;
} UART_Buffer_t;

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
