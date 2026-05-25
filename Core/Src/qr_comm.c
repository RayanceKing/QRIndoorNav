/**
 * @file qr_comm.c
 * @brief K210 通信与二维码/位姿数据处理
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

void QR_Comm_Init(void)
{
    memset(&uart_buffer, 0, sizeof(UART_Buffer_t));
    memset(&latest_qr_data, 0, sizeof(QR_Data_t));
    k210_ready = false;
    last_heartbeat_tick = 0;
}

void QR_Comm_Start_Receive(void)
{
    HAL_UART_Receive_DMA(&huart3, rx_dma_buffer, RX_BUFFER_SIZE);
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
}

/* 解析格式：
 *   $POS,id,x,y,yaw
 *   $ID,world_x,world_y|x0,y0|x1,y1|x2,y2|x3,y3
 */
static bool QR_Parse_Data(const uint8_t *data, uint16_t length, QR_Data_t *qr)
{
    if (length < 4 || data[0] != '$') return false;

    char tmp[RX_BUFFER_SIZE];
    if (length >= RX_BUFFER_SIZE) return false;
    memcpy(tmp, data, length);
    tmp[length] = '\0';

    char *line_end = strpbrk(tmp, "\r\n");
    if (line_end != NULL) {
        *line_end = '\0';
    }

    memset(qr, 0, sizeof(QR_Data_t));

    if (strncmp(tmp, "$POS,", 5) == 0) {
        int tag_id = 0;
        float x = 0.0f, y = 0.0f, yaw = 0.0f;
        int parsed = sscanf(tmp + 5, "%d,%f,%f,%f", &tag_id, &x, &y, &yaw);
        if (parsed == 4) {
            snprintf(qr->id, sizeof(qr->id), "POS%d", tag_id);
            qr->world_x = x;
            qr->world_y = y;
            qr->heading_deg = yaw;
            qr->pose_valid = true;
            qr->corners_valid = false;
            qr->timestamp = HAL_GetTick();
            return true;
        }
    }

    /* 快速尝试解析整数世界坐标的常见情况 */
    char id[16] = {0};
    int wxi=0, wyi=0;
    int cx[4], cy[4];
    int parsed = sscanf(tmp + 1, "%15[^,],%d,%d|%d,%d|%d,%d|%d,%d|%d,%d",
                        id, &wxi, &wyi, &cx[0], &cy[0], &cx[1], &cy[1], &cx[2], &cy[2], &cx[3], &cy[3]);
    if (parsed == 11) {
        strncpy(qr->id, id, sizeof(qr->id)-1);
        qr->world_x = (float)wxi;
        qr->world_y = (float)wyi;
        for (int i=0;i<4;i++){ qr->corner_x[i]=(uint16_t)cx[i]; qr->corner_y[i]=(uint16_t)cy[i]; }
        qr->corners_valid = true;
        qr->pose_valid = false;
        qr->timestamp = HAL_GetTick();
        return true;
    }

    /* 宽容解析：逐字段处理，支持浮点世界坐标 */
    const char *p = tmp + 1; /* skip $ */
    char *endptr = NULL;

    /* ID */
    int i=0;
    while (*p && *p!=',' && i < (int)sizeof(id)-1) id[i++]=*p++;
    id[i]='\0';
    if (*p != ',') return false;
    p++;

    /* world_x (float) */
    float wx = strtof(p, &endptr);
    if (endptr == p) return false;
    p = endptr;
    if (*p != ',') return false;
    p++;

    /* world_y (float) */
    float wy = strtof(p, &endptr);
    if (endptr == p) return false;
    p = endptr;
    if (*p != '|') return false;
    p++;

    uint16_t corners_x[4]={0}, corners_y[4]={0};
    for (i=0;i<4;i++){
        long tx = strtol(p, &endptr, 10);
        if (endptr == p) return false;
        corners_x[i] = (uint16_t)tx; p = endptr;
        if (*p != ',') return false;
        p++;
        long ty = strtol(p, &endptr, 10);
        if (endptr == p) return false;
        corners_y[i] = (uint16_t)ty; p = endptr;
        if (i<3){
            if (*p!='|') return false;
            p++;
        }
    }

    strncpy(qr->id, id, sizeof(qr->id)-1);
    qr->world_x = wx; qr->world_y = wy;
    for (i=0;i<4;i++){ qr->corner_x[i]=corners_x[i]; qr->corner_y[i]=corners_y[i]; }
    qr->corners_valid = true;
    qr->pose_valid = false;
    qr->timestamp = HAL_GetTick();
    return true;
}

bool QR_Comm_Process(QR_Data_t *qr_data)
{
    if (!uart_buffer.data_ready) return false;

    uint8_t *start = NULL, *end = NULL;
    for (uint16_t i=0;i<uart_buffer.length;i++){
        if (uart_buffer.buffer[i] == '$') start = &uart_buffer.buffer[i];
        if (uart_buffer.buffer[i] == '\n' && start!=NULL){ end=&uart_buffer.buffer[i]; break; }
    }
    if (!start || !end) { uart_buffer.data_ready=false; return false; }

    uint16_t packet_len = end - start + 1;
    char packet[RX_BUFFER_SIZE];
    uint16_t copy_len = packet_len < (RX_BUFFER_SIZE-1) ? packet_len : (RX_BUFFER_SIZE-1);
    memcpy(packet, start, copy_len); packet[copy_len]='\0';

    /* heartbeats and init */
    if (strncmp(packet, "$HEARTBEAT", 10)==0){ last_heartbeat_tick = HAL_GetTick(); uart_buffer.data_ready=false; QR_Send_ACK("HB"); return false; }
    if (strncmp(packet, "$INIT",5)==0){ k210_ready=true; last_heartbeat_tick=HAL_GetTick(); uart_buffer.data_ready=false; QR_Send_ACK("INIT"); return false; }

    if (QR_Parse_Data((uint8_t*)packet, packet_len, qr_data)){
        memcpy(&latest_qr_data, qr_data, sizeof(QR_Data_t));
        new_data_flag = true;
        last_heartbeat_tick = HAL_GetTick();
        uart_buffer.data_ready = false;
        QR_Send_ACK(qr_data->pose_valid ? "POS" : "QR");
        return true;
    }
    uart_buffer.data_ready = false;
    return false;
}

QR_Data_t* QR_Comm_Get_Latest(void)
{
    if (new_data_flag){ new_data_flag=false; return &latest_qr_data; }
    return NULL;
}

bool QR_Comm_Has_New_Data(void){ return new_data_flag; }
bool QR_Comm_Is_K210_Ready(void){ return k210_ready; }
uint32_t QR_Comm_Last_Heartbeat_Tick(void){ return last_heartbeat_tick; }
bool QR_Comm_Link_Alive(uint32_t timeout_ms){ if (last_heartbeat_tick==0) return false; return (HAL_GetTick()-last_heartbeat_tick)<=timeout_ms; }

void QR_Comm_Clear_Buffer(void){ uart_buffer.data_ready=false; uart_buffer.length=0; memset(uart_buffer.buffer,0,sizeof(uart_buffer.buffer)); }

void QR_UART_Idle_Callback(void)
{
    if (uart_buffer.data_ready) {
        HAL_UART_AbortReceive_IT(&huart3);
        HAL_UART_Receive_DMA(&huart3, rx_dma_buffer, RX_BUFFER_SIZE);
        return;
    }
    HAL_UART_AbortReceive_IT(&huart3);
    uint32_t transferred = RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);
    if (transferred > 0 && transferred < RX_BUFFER_SIZE) {
        memcpy(uart_buffer.buffer, rx_dma_buffer, transferred);
        uart_buffer.length = transferred;
        uart_buffer.data_ready = true;
    }
    HAL_UART_Receive_DMA(&huart3, rx_dma_buffer, RX_BUFFER_SIZE);
}
