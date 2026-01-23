/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "qr_comm.h"
#include "localization.h"
#include "wifi_comm.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  
  /* 初始化各个模块 */
  Motor_Init();
  QR_Comm_Init();
  QR_Comm_Start_Receive();
  WiFi_Comm_Init();
  WiFi_Comm_Start_Receive();
  Localization_Init();
  
  /* DEBUG: 通过UART3发送初始化消息 */
  const char *init_msg = "System initialized\r\n";
  HAL_UART_Transmit(&huart3, (uint8_t *)init_msg, strlen(init_msg), 100);
  
  /* 电机自检：短暂转动以验证接线和驱动器 */
  const char *test_msg = "Motor self-test...\r\n";
  HAL_UART_Transmit(&huart3, (uint8_t *)test_msg, strlen(test_msg), 100);
  
  Motor_SetSpeed(MOTOR_A, 300);
  Motor_SetSpeed(MOTOR_B, 300);
  Motor_SetSpeed(MOTOR_C, 300);
  Motor_SetSpeed(MOTOR_D, 300);
  HAL_Delay(500);
  Motor_Stop_All();
  
  const char *test_ok = "Motor test complete\r\n";
  HAL_UART_Transmit(&huart3, (uint8_t *)test_ok, strlen(test_ok), 100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  /* 主控制循环参数 */
  #define CONTROL_PERIOD_MS  20  /* 50Hz控制频率 */
  #define WIFI_SEND_PERIOD_MS 200  /* Wi-Fi发送周期5Hz */
  uint32_t last_control_time = 0;
  uint32_t last_wifi_send = 0;
  
  bool navigation_active = false;
  bool manual_control_mode = false;
    bool k210_ready_logged = false;
    bool link_lost = true;
    uint32_t last_link_warn = 0;
  
  while (1)
  {
    uint32_t current_time = HAL_GetTick();
    
    /* 执行50Hz控制循环 */
    if (current_time - last_control_time >= CONTROL_PERIOD_MS) {
        last_control_time = current_time;
        
      /* 输出WiFi调试/服务（非阻塞） */
      WiFi_Print_Debug();

      /* 链路健康监测和事件回应 */
      if (QR_Comm_Is_K210_Ready() && !k210_ready_logged) {
        const char *ready_msg = "K210 ready\r\n";
        HAL_UART_Transmit(&huart3, (uint8_t *)ready_msg, strlen(ready_msg), 10);
        k210_ready_logged = true;
      }

      if (QR_Comm_Link_Alive(3000U)) {
        if (link_lost) {
          const char *restore_msg = "Link restored\r\n";
          HAL_UART_Transmit(&huart3, (uint8_t *)restore_msg, strlen(restore_msg), 10);
          link_lost = false;
        }
      } else {
        if (!link_lost && (current_time - last_link_warn >= 1000U)) {
          const char *lost_msg = "Link lost, stop\r\n";
          HAL_UART_Transmit(&huart3, (uint8_t *)lost_msg, strlen(lost_msg), 10);
          Motor_Stop_All();
          navigation_active = false;
          link_lost = true;
          last_link_warn = current_time;
        }
      }

        /* ========== 第一步：处理Wi-Fi命令 ========== */
        WiFi_Command_t wifi_cmd;
        if (WiFi_Comm_Process(&wifi_cmd)) {
            /* 调试输出：命令已接收 */
            char cmd_debug[64];
            snprintf(cmd_debug, sizeof(cmd_debug), "[Main] WiFi CMD type=%d\r\n", wifi_cmd.type);
            HAL_UART_Transmit(&huart3, (uint8_t *)cmd_debug, strlen(cmd_debug), 10);
            
            switch (wifi_cmd.type) {
                case WIFI_CMD_GOTO:
                    /* 前往目标坐标 */
                    Set_Navigation_Target(wifi_cmd.goto_cmd.target_x, 
                                         wifi_cmd.goto_cmd.target_y, 10.0f);
                    navigation_active = true;
                    manual_control_mode = false;
                    WiFi_Send_Status("NAVIGATING");
                    break;
                    
                case WIFI_CMD_MOVE:
                    /* 手动控制 */
                    manual_control_mode = true;
                    navigation_active = false;
                    {
                        float vx = 0, vy = 0, omega = 0;
                        int16_t spd = wifi_cmd.move_cmd.speed;
                        
                        /* 调试输出：MOVE命令详情 */
                        char move_debug[64];
                        snprintf(move_debug, sizeof(move_debug), "[Main] MOVE dir=%d spd=%d\r\n", 
                                wifi_cmd.move_cmd.direction, spd);
                        HAL_UART_Transmit(&huart3, (uint8_t *)move_debug, strlen(move_debug), 10);
                        
                        switch (wifi_cmd.move_cmd.direction) {
                            case MOVE_FORWARD:    vx = spd;  break;
                            case MOVE_BACKWARD:   vx = -spd; break;
                            case MOVE_LEFT:       vy = spd;  break;
                            case MOVE_RIGHT:      vy = -spd; break;
                            case MOVE_TURN_LEFT:  omega = spd; break;
                            case MOVE_TURN_RIGHT: omega = -spd; break;
                        }
                        Mecanum_Move(vx, vy, omega);
                        
                        /* 调试输出：Mecanum参数 */
                        char mec_debug[64];
                        snprintf(mec_debug, sizeof(mec_debug), "[Main] Mecanum vx=%.0f vy=%.0f w=%.0f\r\n", 
                                vx, vy, omega);
                        HAL_UART_Transmit(&huart3, (uint8_t *)mec_debug, strlen(mec_debug), 10);
                    }
                    WiFi_Send_Status("MANUAL");
                    break;
                    
                case WIFI_CMD_STOP:
                    Motor_Stop_All();
                    navigation_active = false;
                    manual_control_mode = false;
                    WiFi_Send_Status("IDLE");
                    break;
                    
                case WIFI_CMD_QUERY:
                    /* 发送当前状态 */
                    if (navigation_active) {
                        WiFi_Send_Status("NAVIGATING");
                    } else if (manual_control_mode) {
                        WiFi_Send_Status("MANUAL");
                    } else {
                        WiFi_Send_Status("IDLE");
                    }
                    break;
                    
                default:
                    break;
            }
        }
        
        /* ========== 第二步：接收并处理二维码数据 ========== */
        QR_Data_t qr_data;
        if (QR_Comm_Process(&qr_data)) {
            /* 成功接收二维码数据，更新位置 */
            Update_Position_From_QR(&qr_data);
            
            /* DEBUG: 打印接收到的数据 */
            char debug_msg[128];
            snprintf(debug_msg, sizeof(debug_msg), 
                    "QR: %s @ (%.1f, %.1f)\r\n", 
                    qr_data.id, qr_data.world_x, qr_data.world_y);
            HAL_UART_Transmit(&huart3, (uint8_t *)debug_msg, strlen(debug_msg), 10);
            
            /* 自动激活导航：首次检测到QR后，仅在无Wi-Fi控制时激活 */
            if (!navigation_active && !manual_control_mode) {
                float target_x = qr_data.world_x + 50.0f;
                float target_y = qr_data.world_y;
                Set_Navigation_Target(target_x, target_y, 10.0f);
                navigation_active = true;
                
                char nav_msg[128];
                snprintf(nav_msg, sizeof(nav_msg), 
                        "Navigation activated: target (%.1f, %.1f)\r\n", 
                        target_x, target_y);
                HAL_UART_Transmit(&huart3, (uint8_t *)nav_msg, strlen(nav_msg), 10);
            }
        }
        
        /* ========== 第三步：导航控制 ========== */
        if (navigation_active && !manual_control_mode) {
            if (Navigate_Update()) {
                /* 已到达目标 */
                navigation_active = false;
                Motor_Stop_All();
                const char *arrived_msg = "Target reached\r\n";
                HAL_UART_Transmit(&huart3, (uint8_t *)arrived_msg, strlen(arrived_msg), 10);
            }
        }
        
        /* ========== 第四步：定期发送位置到Wi-Fi ========== */
        if (WiFi_Transparent_Mode_Ready()) {  /* 只有进入透传模式才发送POS */
            if (current_time - last_wifi_send >= WIFI_SEND_PERIOD_MS) {
                last_wifi_send = current_time;
                Position_t pos = Get_Current_Position();
                WiFi_Send_Position(pos.x, pos.y, pos.heading);
            }
        }
        
        /* 示例：自动导航到目标点(300, 300)
         * 取消下面的注释来启用自动导航
         */
        // if (!navigation_active && current_time % 5000 == 0) {
        //     Set_Navigation_Target(300.0f, 300.0f, 10.0f);
        //     navigation_active = true;
        // }
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 168-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, AIN2_Pin|AIN1_Pin|CIN1_Pin|DIN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, DIN1_Pin|BIN2_Pin|BIN1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CIN2_GPIO_Port, CIN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : STBY_Pin */
  GPIO_InitStruct.Pin = STBY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STBY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : AIN2_Pin AIN1_Pin CIN1_Pin DIN2_Pin */
  GPIO_InitStruct.Pin = AIN2_Pin|AIN1_Pin|CIN1_Pin|DIN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : DIN1_Pin BIN2_Pin BIN1_Pin */
  GPIO_InitStruct.Pin = DIN1_Pin|BIN2_Pin|BIN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : CIN2_Pin */
  GPIO_InitStruct.Pin = CIN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(CIN2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
