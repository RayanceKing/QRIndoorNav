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
 * @brief 解析二维码数据包
 * 格式：$QR01,150.5,200.3|120,80|180,80|180,140|120,140\n
 */
static bool QR_Parse_Data(const uint8_t *data, uint16_t length, QR_Data_t *qr)
{
    if (length < 4 || data[0] != '$') {
        return false;
    }

    /* 复制为可终止的字符串以便安全解析 */
    char tmp[RX_BUFFER_SIZE];
    if (length >= RX_BUFFER_SIZE) return false;
    memcpy(tmp, data, length);
    tmp[length] = '\0';

    /* 先尝试使用 sscanf 快速解析完整格式（世界坐标以整数发送，例如：QR01,150,200） */
    char id[16] = {0};
    int world_x_i = 0, world_y_i = 0;
    int cx[4], cy[4];
    int parsed = sscanf(tmp + 1, "%15[^,],%d,%d|%d,%d|%d,%d|%d,%d|%d,%d", id, &world_x_i, &world_y_i,
                        &cx[0], &cy[0], &cx[1], &cy[1], &cx[2], &cy[2], &cx[3], &cy[3]);

    if (parsed == 11) {
        /* 全部解析成功（将整数坐标赋值为浮点字段以兼容定位模块） */
        strcpy(qr->id, id);
        qr->world_x = (float)world_x_i;
        qr->world_y = (float)world_y_i;
        for (int i = 0; i < 4; i++) {
            qr->corner_x[i] = (uint16_t)cx[i];
            qr->corner_y[i] = (uint16_t)cy[i];
        }
        qr->timestamp = HAL_GetTick();
        return true;
    }

    /* 若 sscanf 失败，退回到逐字段解析以便更灵活地容错 */
    const char *p = tmp + 1; /* 跳过 $ */
    char *endptr = NULL;

    /* ID */
    int i = 0;
    while (*p && *p != ',' && i < 15) id[i++] = *p++;
    id[i] = '\0';
    if (*p != ',') return false; p++;

    /* world_x（作为整数发送） */
    long wx = strtol(p, &endptr, 10);
    if (endptr == p) return false; p = endptr;
    if (*p != ',') return false; p++;

    /* world_y（作为整数发送） */
    long wy = strtol(p, &endptr, 10);
    if (endptr == p) return false; p = endptr;
    if (*p != '|') return false; p++;

    /* corners */
    uint16_t corners_x[4] = {0}, corners_y[4] = {0};
    for (i = 0; i < 4; i++) {
        long tx = strtol(p, &endptr, 10);
        if (endptr == p) return false;
        corners_x[i] = (uint16_t)tx;
        p = endptr;
        if (*p != ',') return false; p++;

        long ty = strtol(p, &endptr, 10);
        if (endptr == p) return false;
        corners_y[i] = (uint16_t)ty;
        p = endptr;
        if (i < 3) {
            if (*p != '|') return false;
            p++;
        }
    }

    strcpy(qr->id, id);
    qr->world_x = (float)wx;
    qr->world_y = (float)wy;
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
        char packet[RX_BUFFER_SIZE] = {0};
        uint16_t copy_len = packet_len < (RX_BUFFER_SIZE - 1) ? packet_len : (RX_BUFFER_SIZE - 1);
        memcpy(packet, start, copy_len);
        packet[copy_len] = '\0';

        /* 去除调试输出：仅保留解析与回复行为 */

        /* 处理心跳与初始化消息 */
        if (strncmp(packet, "$HEARTBEAT", 10) == 0) {
            last_heartbeat_tick = HAL_GetTick();
            uart_buffer.data_ready = false;
            QR_Send_ACK("HB");
            return false;
        }

        if (strncmp(packet, "$INIT", 5) == 0) {
            k210_ready = true;
            last_heartbeat_tick = HAL_GetTick();
            uart_buffer.data_ready = false;
            QR_Send_ACK("INIT");
            return false;
        }

        /* 正常二维码数据 */
        if (QR_Parse_Data((uint8_t *)packet, packet_len, qr_data)) {
            /* 解析成功，保存并回复 ACK（不打印调试信息） */
            /* 保存最新数据 */
            memcpy(&latest_qr_data, qr_data, sizeof(QR_Data_t));
            new_data_flag = true;
            last_heartbeat_tick = HAL_GetTick();
            QR_Send_ACK("QR");
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
