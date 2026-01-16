/**
 * @file qr_comm.c
 * @brief K210 通信与二维码数据处理实现
 */

#include "qr_comm.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

static UART_Buffer_t uart_buffer = {0};
static QR_Data_t latest_qr_data = {0};
static bool new_data_flag = false;

#define RX_BUFFER_SIZE 256
static uint8_t rx_dma_buffer[RX_BUFFER_SIZE];

/**
 * @brief 初始化UART通信
 */
void QR_Comm_Init(void)
{
    memset(&uart_buffer, 0, sizeof(UART_Buffer_t));
    memset(&latest_qr_data, 0, sizeof(QR_Data_t));
}

/**
 * @brief 启用DMA接收中断
 */
void QR_Comm_Start_Receive(void)
{
    /* 启动DMA接收，并启用空闲中断 */
    HAL_UART_Receive_DMA(&huart3, rx_dma_buffer, RX_BUFFER_SIZE);
    
    /* 启用UART空闲中断 */
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
}

/**
 * @brief 解析二维码数据包
 * 格式：$QR01,150.5,200.3|120,80|180,80|180,140|120,140\n
 */
static bool QR_Parse_Data(const uint8_t *data, uint16_t length, QR_Data_t *qr)
{
    if (length < 20 || data[0] != '$') {
        return false;
    }
    
    /* 跳过 $ */
    const char *p = (const char *)data + 1;
    const char *end = (const char *)data + length;
    
    /* 1. 解析ID */
    char id[16] = {0};
    int i = 0;
    while (p < end && *p != ',' && i < 15) {
        id[i++] = *p++;
    }
    id[i] = 0;
    
    if (p >= end || *p != ',') return false;
    p++;  /* 跳过 , */
    
    /* 2. 解析world_x, world_y */
    float world_x = strtof(p, (char **)&p);
    if (*p != ',') return false;
    p++;
    
    float world_y = strtof(p, (char **)&p);
    if (*p != '|') return false;
    p++;
    
    /* 3. 解析4个角点坐标 */
    uint16_t corners_x[4], corners_y[4];
    for (i = 0; i < 4; i++) {
        corners_x[i] = strtol(p, (char **)&p, 10);
        if (*p != ',') return false;
        p++;
        
        corners_y[i] = strtol(p, (char **)&p, 10);
        if (i < 3 && *p != '|') return false;
        if (i < 3) p++;
    }
    
    /* 保存解析结果 */
    strcpy(qr->id, id);
    qr->world_x = world_x;
    qr->world_y = world_y;
    for (i = 0; i < 4; i++) {
        qr->corner_x[i] = corners_x[i];
        qr->corner_y[i] = corners_y[i];
    }
    qr->timestamp = HAL_GetTick();
    
    return true;
}

/**
 * @brief 处理接收到的数据
 */
bool QR_Comm_Process(QR_Data_t *qr_data)
{
    if (!uart_buffer.data_ready) {
        return false;
    }
    
    /* 查找数据包边界：$ 开始，\n 结束 */
    uint8_t *start = NULL;
    uint8_t *end = NULL;
    
    for (uint16_t i = 0; i < uart_buffer.length; i++) {
        if (uart_buffer.buffer[i] == '$') {
            start = &uart_buffer.buffer[i];
        }
        if (uart_buffer.buffer[i] == '\n' && start != NULL) {
            end = &uart_buffer.buffer[i];
            break;
        }
    }
    
    if (start == NULL || end == NULL) {
        uart_buffer.data_ready = false;
        return false;
    }
    
    uint16_t packet_len = end - start + 1;
    
    if (QR_Parse_Data(start, packet_len, qr_data)) {
        /* 保存最新数据 */
        memcpy(&latest_qr_data, qr_data, sizeof(QR_Data_t));
        new_data_flag = true;
        uart_buffer.data_ready = false;
        return true;
    }
    
    uart_buffer.data_ready = false;
    return false;
}

/**
 * @brief 获取最新的二维码数据
 */
QR_Data_t* QR_Comm_Get_Latest(void)
{
    if (new_data_flag) {
        new_data_flag = false;
        return &latest_qr_data;
    }
    return NULL;
}

/**
 * @brief 检查是否有新数据
 */
bool QR_Comm_Has_New_Data(void)
{
    return new_data_flag;
}

/**
 * @brief 清空接收缓冲区
 */
void QR_Comm_Clear_Buffer(void)
{
    uart_buffer.data_ready = false;
    uart_buffer.length = 0;
    memset(uart_buffer.buffer, 0, sizeof(uart_buffer.buffer));
}

/**
 * @brief UART空闲中断回调
 * 在 stm32f4xx_it.c 的 USART3_IRQHandler 中调用
 */
void QR_UART_Idle_Callback(void)
{
    /* 停止DMA并获取接收长度 */
    HAL_UART_AbortReceive_IT(&huart3);
    
    uint32_t transferred = RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);
    
    if (transferred > 0) {
        memcpy(uart_buffer.buffer, rx_dma_buffer, transferred);
        uart_buffer.length = transferred;
        uart_buffer.data_ready = true;
    }
    
    /* 重新启动DMA接收 */
    HAL_UART_Receive_DMA(&huart3, rx_dma_buffer, RX_BUFFER_SIZE);
}
