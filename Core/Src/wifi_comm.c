/**
 * @file wifi_comm.c
 * @brief DX-WF24-A Wi-Fi/蓝牙模块通信实现
 */

#include "wifi_comm.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;  /* 用于调试输出 */

/* 接收缓冲区 */
#define WIFI_RX_BUF_SIZE 256
static uint8_t wifi_rx_buffer[WIFI_RX_BUF_SIZE];
static volatile uint16_t wifi_rx_index = 0;
static uint8_t wifi_rx_byte;  /* 单字节接收缓冲 */
static volatile uint32_t last_byte_time = 0;  /* 上次接收时间戳 */

/* 命令解析状态 */
static WiFi_Command_t pending_command;
static bool command_ready = false;
static uint32_t last_recv_time = 0;
static bool ble_connected = false;  /* BLE连接状态 */
static bool transparent_mode_ready = false;  /* 透传模式就绪标志 */

/* 调试信息缓冲 */
/* 前向声明 */
static bool Parse_Command(const char *line, WiFi_Command_t *cmd);

/* 内部函数：处理一行命令/消息，wifi_rx_buffer 需以'\0'结尾 */
static void WiFi_Process_Line(void)
{
    /* 检查是否是BLE连接成功消息 */
    if (strcmp((char *)wifi_rx_buffer, "BLE_CONNECT_SUCCESS") == 0) {
        if (!ble_connected) {
            ble_connected = true;
            transparent_mode_ready = true;  /* 设置透传模式就绪 */
            /* 进入透传模式 */
            const char *at_cmd = "AT+BLUFISEND=1\r\n";
            HAL_UART_Transmit(&huart2, (uint8_t *)at_cmd, strlen(at_cmd), 100);
        }
    } else if (Parse_Command((char *)wifi_rx_buffer, &pending_command)) {
        command_ready = true;
    } else {
        /* 解析失败忽略 */
    }
}

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
    /* 启动中断接收到固定缓冲区 */
    HAL_UART_Receive_IT(&huart2, &wifi_rx_byte, 1);
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
 * @brief UART接收完成回调（由HAL调用）
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        uint8_t byte = wifi_rx_byte;  /* 读取接收到的字节 */
        last_recv_time = HAL_GetTick();
        last_byte_time = last_recv_time;

        /* 行结束符：支持 \r 或 \n 任一作为结束 */
        if (byte == '\r' || byte == '\n') {
            if (wifi_rx_index > 0U) {
                wifi_rx_buffer[wifi_rx_index] = '\0';
                WiFi_Process_Line();
                wifi_rx_index = 0;
            }
        } else if (wifi_rx_index < WIFI_RX_BUF_SIZE - 1U) {
            wifi_rx_buffer[wifi_rx_index++] = byte;  /* 保存字节 */
        } else {
            /* 缓冲区溢出，丢弃并重置 */
            wifi_rx_index = 0;
        }

        /* 继续接收下一个字节到固定位置 */
        HAL_UART_Receive_IT(&huart2, &wifi_rx_byte, 1);
    }
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
    int len = snprintf(msg, sizeof(msg), "POS:%d,%d,%d\r\n", (int)x, (int)y, (int)heading);
    if (len > 0) {
        HAL_UART_Transmit(&huart2, (uint8_t *)msg, len, 100);
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
        HAL_UART_Transmit(&huart2, (uint8_t *)msg, len, 100);
    }
}

/**
 * @brief 检查连接活跃
 */
bool WiFi_Link_Alive(uint32_t timeout_ms)
{
    return (HAL_GetTick() - last_recv_time) < timeout_ms;
}

/**
 * @brief 检查透传模式是否就绪
 * @return true表示已进入透传模式，可以收发数据
 */
bool WiFi_Transparent_Mode_Ready(void)
{
    return transparent_mode_ready;
}


/**
 * @brief 输出调试信息（在主循环中调用）
 */
void WiFi_Print_Debug(void)
{
    static uint32_t last_stats_time = 0;
    uint32_t current_time = HAL_GetTick();

    /* 若缓冲有残留且200ms未继续接收，按一行处理（兼容无换行的发送） */
    if (wifi_rx_index > 0U && (current_time - last_byte_time) >= 200U) {
        wifi_rx_buffer[wifi_rx_index] = '\0';
        WiFi_Process_Line();
        wifi_rx_index = 0;
        last_byte_time = current_time;
    }
}