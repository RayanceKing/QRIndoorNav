/**
 * @file wifi_comm.c
 * @brief DX-WF24-A Wi-Fi/蓝牙模块通信实现
 */

#include "wifi_comm.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

extern UART_HandleTypeDef huart1;

/* 接收缓冲区 */
#define WIFI_RX_BUF_SIZE 256
static uint8_t wifi_rx_buffer[WIFI_RX_BUF_SIZE];
static uint16_t wifi_rx_index = 0;

/* 命令解析状态 */
static WiFi_Command_t pending_command;
static bool command_ready = false;
static uint32_t last_recv_time = 0;

/**
 * @brief 初始化Wi-Fi通信
 */
void WiFi_Comm_Init(void)
{
    wifi_rx_index = 0;
    command_ready = false;
    memset(wifi_rx_buffer, 0, WIFI_RX_BUF_SIZE);
}

/**
 * @brief 启动UART接收
 */
void WiFi_Comm_Start_Receive(void)
{
    /* 启动中断接收 */
    HAL_UART_Receive_IT(&huart1, &wifi_rx_buffer[wifi_rx_index], 1);
}

/**
 * @brief 解析命令字符串
 */
static bool Parse_Command(const char *line, WiFi_Command_t *cmd)
{
    if (strncmp(line, "CMD:GOTO,", 9) == 0) {
        /* 格式: CMD:GOTO,x,y */
        const char *p = line + 9;
        char *endptr;
        
        cmd->goto_cmd.target_x = strtof(p, &endptr);
        if (endptr == p || *endptr != ',') return false;
        p = endptr + 1;
        
        cmd->goto_cmd.target_y = strtof(p, &endptr);
        if (endptr == p) return false;
        
        cmd->type = WIFI_CMD_GOTO;
        return true;
        
    } else if (strncmp(line, "CMD:MOVE,", 9) == 0) {
        /* 格式: CMD:MOVE,dir,speed */
        const char *p = line + 9;
        char dir[8] = {0};
        int i = 0;
        
        while (*p && *p != ',' && i < 7) {
            dir[i++] = *p++;
        }
        if (*p != ',') return false;
        p++;
        
        /* 解析方向 */
        if (strcmp(dir, "F") == 0) cmd->move_cmd.direction = MOVE_FORWARD;
        else if (strcmp(dir, "B") == 0) cmd->move_cmd.direction = MOVE_BACKWARD;
        else if (strcmp(dir, "L") == 0) cmd->move_cmd.direction = MOVE_LEFT;
        else if (strcmp(dir, "R") == 0) cmd->move_cmd.direction = MOVE_RIGHT;
        else if (strcmp(dir, "TL") == 0) cmd->move_cmd.direction = MOVE_TURN_LEFT;
        else if (strcmp(dir, "TR") == 0) cmd->move_cmd.direction = MOVE_TURN_RIGHT;
        else return false;
        
        /* 解析速度 */
        char *endptr;
        long speed = strtol(p, &endptr, 10);
        if (endptr == p || speed < 0 || speed > 999) return false;
        cmd->move_cmd.speed = (int16_t)speed;
        
        cmd->type = WIFI_CMD_MOVE;
        return true;
        
    } else if (strcmp(line, "CMD:STOP") == 0) {
        cmd->type = WIFI_CMD_STOP;
        return true;
        
    } else if (strcmp(line, "CMD:QUERY") == 0) {
        cmd->type = WIFI_CMD_QUERY;
        return true;
    }
    
    return false;
}

/**
 * @brief UART接收完成回调（由中断处理）
 */
void WiFi_UART_RxCallback(uint8_t byte)
{
    last_recv_time = HAL_GetTick();
    
    if (byte == '\n' && wifi_rx_index > 0 && wifi_rx_buffer[wifi_rx_index - 1] == '\r') {
        /* 收到完整行 */
        wifi_rx_buffer[wifi_rx_index - 1] = '\0';  /* 去掉\r */
        
        if (Parse_Command((char *)wifi_rx_buffer, &pending_command)) {
            command_ready = true;
        }
        
        wifi_rx_index = 0;
    } else if (wifi_rx_index < WIFI_RX_BUF_SIZE - 1) {
        wifi_rx_buffer[wifi_rx_index++] = byte;
    } else {
        /* 缓冲区溢出，重置 */
        wifi_rx_index = 0;
    }
    
    /* 继续接收下一个字节 */
    HAL_UART_Receive_IT(&huart1, &wifi_rx_buffer[wifi_rx_index], 1);
}

/**
 * @brief 处理命令
 */
bool WiFi_Comm_Process(WiFi_Command_t *cmd)
{
    if (command_ready) {
        *cmd = pending_command;
        command_ready = false;
        return true;
    }
    return false;
}

/**
 * @brief 发送位置到手机
 */
void WiFi_Send_Position(float x, float y, float heading)
{
    char msg[64];
    int len = snprintf(msg, sizeof(msg), "POS:%.1f,%.1f,%.1f\r\n", x, y, heading);
    if (len > 0) {
        HAL_UART_Transmit(&huart1, (uint8_t *)msg, len, 100);
    }
}

/**
 * @brief 发送状态到手机
 */
void WiFi_Send_Status(const char *state)
{
    char msg[64];
    int len = snprintf(msg, sizeof(msg), "STATUS:%s\r\n", state);
    if (len > 0) {
        HAL_UART_Transmit(&huart1, (uint8_t *)msg, len, 100);
    }
}

/**
 * @brief 检查连接活跃
 */
bool WiFi_Link_Alive(uint32_t timeout_ms)
{
    return (HAL_GetTick() - last_recv_time) < timeout_ms;
}
