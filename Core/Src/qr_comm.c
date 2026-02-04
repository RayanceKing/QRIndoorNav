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
static bool k210_ready = false;
static uint32_t last_heartbeat_tick = 0;

#define RX_BUFFER_SIZE 256
static uint8_t rx_dma_buffer[RX_BUFFER_SIZE];

static void QR_Send_ACK(const char *tag)
{
    char ack[32];
    int len = snprintf(ack, sizeof(ack), "$ACK,%s\r\n", tag);
    if (len > 0) {
        HAL_UART_Transmit(&huart3, (uint8_t *)ack, (uint16_t)len, 10);
    }
}

/**
 * @brief 初始化UART通信
 */
void QR_Comm_Init(void)
{
    memset(&uart_buffer, 0, sizeof(UART_Buffer_t));
    memset(&latest_qr_data, 0, sizeof(QR_Data_t));
    k210_ready = false;
    last_heartbeat_tick = 0;
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
 * @brief 解析AprilTag位置数据包
 * 格式：$POS,id,x,y,yaw\r\n
 * 例：$POS,0,50,80,90\r\n
 */
static bool QR_Parse_Data(const uint8_t *data, uint16_t length, QR_Data_t *qr)
{
    if (length < 10 || data[0] != '$') {
        return false;
    }

    /* 复制为可终止的字符串以便安全解析 */
    char tmp[RX_BUFFER_SIZE];
    if (length >= RX_BUFFER_SIZE) return false;
    memcpy(tmp, data, length);
    tmp[length] = '\0';

    /* 使用更兼容的格式：全部用int解析 */
    int pos_id = 0, x = 0, y = 0, yaw = 0;
    
    int parsed = sscanf(tmp + 1, "POS,%d,%d,%d,%d", 
                        &pos_id, &x, &y, &yaw);

    if (parsed == 4) {
        /* 全部解析成功 */
        qr->id = (int8_t)pos_id;  /* 支持负数ID（-1表示融合位置） */
        qr->x = (int16_t)x;
        qr->y = (int16_t)y;
        qr->yaw = (int16_t)yaw;
        qr->valid = true;
        qr->timestamp = HAL_GetTick();
        
        return true;
    }

    /* 解析失败，输出调试信息 */
    char debug[64];
    snprintf(debug, sizeof(debug), "PARSE_ERR: parsed=%d (id=%d,x=%d,y=%d,yaw=%d)\r\n", 
             parsed, pos_id, x, y, yaw);
    HAL_UART_Transmit(&huart3, (uint8_t *)debug, strlen(debug), 10);
    return false;
}

/**
 * @brief 处理接收到的数据
 */
bool QR_Comm_Process(QR_Data_t *qr_data)
{
    if (!uart_buffer.data_ready) {
        return false;
    }
    
    /* 立即清除标志位，防止重入 */
    uart_buffer.data_ready = false;
    
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
    char packet[RX_BUFFER_SIZE] = {0};
    uint16_t copy_len = packet_len < (RX_BUFFER_SIZE - 1) ? packet_len : (RX_BUFFER_SIZE - 1);
    memcpy(packet, start, copy_len);
    packet[copy_len] = '\0';

    /* USART3调试输出：原始K210报文 */
    HAL_UART_Transmit(&huart3, (uint8_t *)"RX:", 3, 10);
    HAL_UART_Transmit(&huart3, (uint8_t *)packet, (uint16_t)strlen(packet), 10);

    /* 处理心跳与初始化消息 */
    if (strncmp(packet, "$HEARTBEAT", 10) == 0) {
        last_heartbeat_tick = HAL_GetTick();
        return false;
    }

    if (strncmp(packet, "$INIT", 5) == 0) {
        k210_ready = true;
        last_heartbeat_tick = HAL_GetTick();
        const char *init_ok = "K210_INIT_OK\r\n";
        HAL_UART_Transmit(&huart3, (uint8_t *)init_ok, strlen(init_ok), 10);
        return false;
    }

    /* 正常二维码数据 */
    if (QR_Parse_Data((uint8_t *)packet, packet_len, qr_data)) {
        /* 解析成功，保存并回复 ACK */
        memcpy(&latest_qr_data, qr_data, sizeof(QR_Data_t));
        new_data_flag = true;
        last_heartbeat_tick = HAL_GetTick();
        
        /* 输出SUCCESS以便诊断 */
        const char *success = "PARSE_OK\r\n";
        HAL_UART_Transmit(&huart3, (uint8_t *)success, strlen(success), 10);
        
        return true;
    }
    
    /* 解析失败 */
    const char *fail = "PARSE_FAIL\r\n";
    HAL_UART_Transmit(&huart3, (uint8_t *)fail, strlen(fail), 10);
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

bool QR_Comm_Is_K210_Ready(void)
{
    return k210_ready;
}

uint32_t QR_Comm_Last_Heartbeat_Tick(void)
{
    return last_heartbeat_tick;
}

bool QR_Comm_Link_Alive(uint32_t timeout_ms)
{
    if (last_heartbeat_tick == 0) {
        return false;
    }
    return (HAL_GetTick() - last_heartbeat_tick) <= timeout_ms;
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
    /* 如果上一帧数据还未处理完，跳过本次接收 */
    if (uart_buffer.data_ready) {
        /* 清除DMA计数器但不覆盖缓冲区 */
        HAL_UART_AbortReceive_IT(&huart3);
        HAL_UART_Receive_DMA(&huart3, rx_dma_buffer, RX_BUFFER_SIZE);
        return;
    }
    
    /* 停止DMA并获取接收长度 */
    HAL_UART_AbortReceive_IT(&huart3);
    
    uint32_t transferred = RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);
    
    if (transferred > 0 && transferred < RX_BUFFER_SIZE) {
        memcpy(uart_buffer.buffer, rx_dma_buffer, transferred);
        uart_buffer.length = transferred;
        uart_buffer.data_ready = true;  /* 设置标志位，主循环将处理 */
    }
    
    /* 重新启动DMA接收 */
    HAL_UART_Receive_DMA(&huart3, rx_dma_buffer, RX_BUFFER_SIZE);
}
