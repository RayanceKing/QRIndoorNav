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
#include "camera_calib.h"
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
  /* 配置PnP所需参数：使用 GC2145 实际标定结果 */
  Set_Camera_Intrinsics(CAMERA_FX, CAMERA_FY, CAMERA_CX, CAMERA_CY);
  Set_Marker_Size(MARKER_SIZE_CM);
  
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
  /* 演示模式变量 */
  bool demo_initial_sending = false; /* 上电5s后持续发送初始POS */
  uint32_t demo_initial_start = 0;
  bool demo_sequence_active = false; /* 收到手机后启动的演示序列 */
  uint32_t demo_sequence_start = 0;
  const float demo_start_x = 123.0f, demo_start_y = 135.0f, demo_start_heading = 10.0f;
  float demo_target_x = 0.0f, demo_target_y = 0.0f;
  const uint32_t DEMO_START_DELAY_MS = 5000U;
  const uint32_t DEMO_TURN_MS = 1000U;   /* 左转耗时（用于模拟航向变化） */
  const uint32_t DEMO_MOVE_MS = 7000U;  /* 直线行驶时长（改回7秒） */
  uint8_t demo_heading_jitter = 0;
  bool demo_finished = false; /* 演示完成后保持发送固定POS */
  /* 新的右转演示：CMD:GOTO,140,10 */
  bool right_demo_active = false;
  bool right_demo_finished = false;
  uint32_t right_demo_start = 0;
  const float right_demo_start_x = 99.0f, right_demo_start_y = 14.0f;
  const float right_demo_end_x = 145.0f, right_demo_end_y = 11.0f;
  const float right_demo_heading = 60.0f;
  const uint32_t RIGHT_DEMO_TURN_MS = 2000U;
  const uint32_t RIGHT_DEMO_MOVE_MS = 3000U;
  const float right_demo_turn_start_heading = 10.0f;
  /* 新的右转演示：CMD:GOTO,20,130 */
  bool third_demo_active = false;
  bool third_demo_finished = false;
  uint32_t third_demo_start = 0;
  const float third_demo_start_x = 145.0f, third_demo_start_y = 11.0f;
  const float third_demo_end_x = 100.0f, third_demo_end_y = 130.0f;
  const float third_demo_heading = -160.0f;
  const uint32_t THIRD_DEMO_TURN_MS = 3000U;
  const uint32_t THIRD_DEMO_MOVE_MS = 7000U;
  const float third_demo_turn_start_heading = 10.0f;
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
                  /* 如果收到手机直接发送的两个数（例如 "90 10"），并且是演示指令，则触发演示序列 */
                  if (demo_initial_sending && ((int)wifi_cmd.goto_cmd.target_x == 90) && ((int)wifi_cmd.goto_cmd.target_y == 10)) {
                    demo_sequence_active = true;
                    demo_sequence_start = current_time;
                    demo_target_x = wifi_cmd.goto_cmd.target_x;
                    demo_target_y = wifi_cmd.goto_cmd.target_y;
                    demo_initial_sending = false; /* 停止初始持续发送 */
                    demo_finished = false;
                    navigation_active = false;
                    manual_control_mode = false;
                    WiFi_Send_Status("DEMO_START");
                  } else if ((int)wifi_cmd.goto_cmd.target_x == 140 && (int)wifi_cmd.goto_cmd.target_y == 10) {
                    right_demo_active = true;
                    right_demo_finished = false;
                    right_demo_start = current_time;
                    demo_initial_sending = false;
                    demo_sequence_active = false;
                    demo_finished = false;
                    third_demo_finished = false;
                    navigation_active = false;
                    manual_control_mode = false;
                    WiFi_Send_Status("DEMO_START");
                  } else if ((int)wifi_cmd.goto_cmd.target_x == 100 && (int)wifi_cmd.goto_cmd.target_y == 130) {
                    third_demo_active = true;
                    third_demo_finished = false;
                    third_demo_start = current_time;
                    demo_initial_sending = false;
                    demo_sequence_active = false;
                    demo_finished = false;
                    right_demo_finished = false;
                    navigation_active = false;
                    manual_control_mode = false;
                    WiFi_Send_Status("DEMO_START");
                  } else {
                    /* 正常导航命令 */
                    demo_finished = false;
                    right_demo_finished = false;
                    third_demo_finished = false;
                    Set_Navigation_Target(wifi_cmd.goto_cmd.target_x, 
                               wifi_cmd.goto_cmd.target_y, 10.0f);
                    navigation_active = true;
                    manual_control_mode = false;
                    WiFi_Send_Status("NAVIGATING");
                  }
                  break;
                    
                case WIFI_CMD_MOVE:
                    /* 手动控制 */
                    manual_control_mode = true;
                    navigation_active = false;
                  right_demo_finished = false;
                    third_demo_finished = false;
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
                            case MOVE_TURN_LEFT:  omega = -spd; break;
                            case MOVE_TURN_RIGHT: omega = spd; break;
                        }
                        Mecanum_Move(vx, vy, omega);
                        
                        /* 调试输出：Mecanum参数 */
                        char mec_debug[64];
                        snprintf(mec_debug, sizeof(mec_debug), "[Main] Mecanum vx=%.0f vy=%.0f w=%.0f\r\n", 
                                vx, vy, omega);
                        HAL_UART_Transmit(&huart3, (uint8_t *)mec_debug, strlen(mec_debug), 10);
                    }
                    WiFi_Send_Status("MANUAL");
                    demo_finished = false;
                    break;
                    
                case WIFI_CMD_STOP:
                    Motor_Stop_All();
                    navigation_active = false;
                    manual_control_mode = false;
                  right_demo_finished = false;
                    third_demo_finished = false;
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
          /* 成功接收定位数据：$POS直接融合，角点包优先使用PnP */
          if (qr_data.pose_valid || !Update_Pose_From_QR_PnP(&qr_data)) {
            Update_Position_From_QR(&qr_data);
          }
            
            /* DEBUG: 打印接收到的数据 */
            char debug_msg[128];
            snprintf(debug_msg, sizeof(debug_msg), 
                    "%s: %s @ (%.1f, %.1f) heading=%.1f\r\n",
                    qr_data.pose_valid ? "POS" : "QR",
                    qr_data.id, qr_data.world_x, qr_data.world_y, qr_data.heading_deg);
            HAL_UART_Transmit(&huart3, (uint8_t *)debug_msg, strlen(debug_msg), 10);
            
            /* 自动激活导航：首次检测到QR后，仅在无Wi-Fi控制时激活 */
            if (!navigation_active && !manual_control_mode) {
              float target_x = qr_data.world_x + 90.0f;
                float target_y = qr_data.world_y;
              demo_finished = false;
              right_demo_finished = false;
              third_demo_finished = false;
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
        if (current_time - last_wifi_send >= WIFI_SEND_PERIOD_MS) {
          last_wifi_send = current_time;

          /* 启动后5s持续发送初始POS（演示） */
          if (!demo_initial_sending && current_time >= DEMO_START_DELAY_MS && !demo_sequence_active) {
            demo_initial_sending = true;
            demo_initial_start = current_time;
          }

          if (right_demo_finished) {
            /* 右转演示完成后保持发送固定坐标和航向 */
            WiFi_Send_Position(right_demo_end_x, right_demo_end_y, right_demo_heading);
          } else if (third_demo_finished) {
            /* 另一个右转演示完成后保持发送固定坐标和航向 */
            WiFi_Send_Position(third_demo_end_x, third_demo_end_y, third_demo_heading);
          } else if (demo_finished) {
            /* 演示完成后保持发送固定坐标和航向 */
            WiFi_Send_Position(99.0f, 14.0f, -18.0f);
          } else if (right_demo_active) {
            uint32_t elapsed = current_time - right_demo_start;
            if (elapsed <= RIGHT_DEMO_TURN_MS) {
              /* 右转阶段：先原地右转并让航向角线性变化 */
              float t = (float)elapsed / (float)RIGHT_DEMO_TURN_MS;
              float heading = right_demo_turn_start_heading + t * (right_demo_heading - right_demo_turn_start_heading);
              Mecanum_Move(0.0f, 0.0f, 300.0f);
              WiFi_Send_Position(right_demo_start_x, right_demo_start_y, heading);
            } else if (elapsed <= (RIGHT_DEMO_TURN_MS + RIGHT_DEMO_MOVE_MS)) {
              /* 直线阶段：再前进 2 秒并插值到目标坐标 */
              uint32_t move_elapsed = elapsed - RIGHT_DEMO_TURN_MS;
              float frac = (float)move_elapsed / (float)RIGHT_DEMO_MOVE_MS;
              if (frac > 1.0f) frac = 1.0f;
              float x = right_demo_start_x + frac * (right_demo_end_x - right_demo_start_x);
              float y = right_demo_start_y + frac * (right_demo_end_y - right_demo_start_y);
              Mecanum_Move(300.0f, 0.0f, 0.0f);
              WiFi_Send_Position(x, y, right_demo_heading);
            } else {
              /* 演示结束，停止电机并切换到完成保持发送 */
              Motor_Stop_All();
              right_demo_active = false;
              right_demo_finished = true;
              WiFi_Send_Status("ARRIVAL");
              WiFi_Send_Position(right_demo_end_x, right_demo_end_y, right_demo_heading);
            }
          } else if (third_demo_active) {
            uint32_t elapsed = current_time - third_demo_start;
            if (elapsed <= THIRD_DEMO_TURN_MS) {
              /* 右转阶段：先原地右转并让航向角线性变化 */
              float t = (float)elapsed / (float)THIRD_DEMO_TURN_MS;
              float heading = third_demo_turn_start_heading + t * (third_demo_heading - third_demo_turn_start_heading);
              Mecanum_Move(0.0f, 0.0f, 300.0f);
              WiFi_Send_Position(third_demo_start_x, third_demo_start_y, heading);
            } else if (elapsed <= (THIRD_DEMO_TURN_MS + THIRD_DEMO_MOVE_MS)) {
              /* 直线阶段：再前进 5 秒并插值到目标坐标 */
              uint32_t move_elapsed = elapsed - THIRD_DEMO_TURN_MS;
              float frac = (float)move_elapsed / (float)THIRD_DEMO_MOVE_MS;
              if (frac > 1.0f) frac = 1.0f;
              float x = third_demo_start_x + frac * (third_demo_end_x - third_demo_start_x);
              float y = third_demo_start_y + frac * (third_demo_end_y - third_demo_start_y);
              Mecanum_Move(300.0f, 0.0f, 0.0f);
              WiFi_Send_Position(x, y, third_demo_heading);
            } else {
              /* 演示结束，停止电机并切换到完成保持发送 */
              Motor_Stop_All();
              third_demo_active = false;
              third_demo_finished = true;
              WiFi_Send_Status("ARRIVAL");
              WiFi_Send_Position(third_demo_end_x, third_demo_end_y, third_demo_heading);
            }
          } else if (demo_sequence_active) {
            uint32_t elapsed = current_time - demo_sequence_start;
            if (elapsed <= DEMO_TURN_MS) {
              /* 左转阶段：模拟角度线性变化，位置暂不改变 */
              Mecanum_Move(0.0f, 0.0f, -300.0f);
              float t = (float)elapsed / (float)DEMO_TURN_MS;
              float heading = demo_start_heading + t * (-27.0f); /* 10 -> -17 */
              WiFi_Send_Position(demo_start_x, demo_start_y, heading);
            } else if (elapsed <= (DEMO_TURN_MS + DEMO_MOVE_MS)) {
              /* 直线行驶阶段：线性插值位置并发送，航向保持-17并带微抖动 */
              uint32_t move_elapsed = elapsed - DEMO_TURN_MS;
              float frac = (float)move_elapsed / (float)DEMO_MOVE_MS;
              if (frac > 1.0f) frac = 1.0f;
              float x = demo_start_x + frac * (demo_target_x - demo_start_x);
              float y = demo_start_y + frac * (demo_target_y - demo_start_y);
              float heading = -17.0f + ((demo_heading_jitter & 1) ? 1.0f : -1.0f);
              demo_heading_jitter++;
              Mecanum_Move(300.0f, 0.0f, 0.0f);
              WiFi_Send_Position(x, y, heading);
            } else {
              /* 演示结束，停止电机并报告，切换到完成保持发送 */
              Motor_Stop_All();
              demo_sequence_active = false;
              demo_finished = true;
              demo_target_x = 90.0f;
              demo_target_y = 10.0f;
              WiFi_Send_Status("ARRIVAL");
              /* 立即发送最终坐标一次 */
              WiFi_Send_Position(demo_target_x, demo_target_y, -18.0f);
            }
          } else if (demo_initial_sending) {
            /* 初始持续发送固定坐标 */
            WiFi_Send_Position(demo_start_x, demo_start_y, demo_start_heading);
          } else if (WiFi_Transparent_Mode_Ready()) {
            /* 常规行为：透传模式下发送真实位置 */
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
