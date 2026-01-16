# 代码导览 - 文件详解

## 核心模块详解

### 1. motor.h / motor.c - 电机驱动模块

**文件位置：** `Core/Inc/motor.h`, `Core/Src/motor.c`

**用途：** 控制TB6612四路电机驱动模块

#### 关键数据结构

```c
// 电机引脚配置结构体
typedef struct {
    GPIO_TypeDef *in1_port;
    uint16_t      in1_pin;
    GPIO_TypeDef *in2_port;
    uint16_t      in2_pin;
} MotorPins_t;
```

#### 核心函数

| 函数 | 参数 | 返回值 | 功能 |
|---|---|---|---|
| `Motor_Init()` | 无 | void | 初始化电机系统 |
| `Motor_SetSpeed()` | motor_id (0-3), speed (-999~999) | void | 设置单个电机速度 |
| `Motor_Stop_All()` | 无 | void | 停止所有电机 |
| `Mecanum_Move()` | vx, vy, omega (float) | void | 麦轮全向移动 |

#### 实现细节

**电机映射表：**
```c
static const MotorPins_t motor_pins[4] = {
    {GPIOC, AIN1_Pin, GPIOC, AIN2_Pin},  // Motor A - 左前
    {GPIOD, BIN1_Pin, GPIOD, BIN2_Pin},  // Motor B - 右前
    {GPIOC, CIN1_Pin, GPIOG, CIN2_Pin},  // Motor C - 左后
    {GPIOD, DIN1_Pin, GPIOC, DIN2_Pin},  // Motor D - 右后
};
```

**PWM通道映射：**
```c
static const uint32_t pwm_channels[4] = {
    TIM_CHANNEL_1,  // PA6 - Motor A
    TIM_CHANNEL_2,  // PA7 - Motor B
    TIM_CHANNEL_3,  // PB0 - Motor C
    TIM_CHANNEL_4,  // PB1 - Motor D
};
```

**速度控制逻辑：**
```
当 speed > 0：   IN1=1, IN2=0 (正向)
当 speed < 0：   IN1=0, IN2=1 (反向)
当 speed = 0：   IN1=0, IN2=0 (停止)
PWM = |speed|   (占空比由绝对值决定)
```

---

### 2. qr_comm.h / qr_comm.c - 通信模块

**文件位置：** `Core/Inc/qr_comm.h`, `Core/Src/qr_comm.c`

**用途：** 接收K210模块的二维码识别结果

#### 关键数据结构

```c
// 二维码数据结构体
typedef struct {
    char id[16];           // QR ID: "QR01", "QR02" 等
    float world_x;         // 世界坐标X (cm)
    float world_y;         // 世界坐标Y (cm)
    uint16_t corner_x[4];  // 四个角点像素坐标X
    uint16_t corner_y[4];  // 四个角点像素坐标Y
    uint32_t timestamp;    // 接收时间戳 (ms)
} QR_Data_t;

// UART接收缓冲区
typedef struct {
    uint8_t buffer[256];
    uint16_t length;
    bool data_ready;
} UART_Buffer_t;
```

#### 核心函数

| 函数 | 功能 |
|---|---|
| `QR_Comm_Init()` | 初始化通信系统 |
| `QR_Comm_Start_Receive()` | 启动DMA接收 |
| `QR_Comm_Process()` | 处理接收数据，返回true表示成功解析 |
| `QR_Comm_Get_Latest()` | 获取最新二维码数据 |
| `QR_Comm_Has_New_Data()` | 检查是否有新数据 |
| `QR_UART_Idle_Callback()` | UART空闲中断回调 |

#### 数据解析流程

```
接收原始数据
    ↓
UART空闲中断触发
    ↓
QR_UART_Idle_Callback() 被调用
    ↓
数据复制到缓冲区，设置 data_ready=true
    ↓
主循环调用 QR_Comm_Process()
    ↓
查找 $ 和 \n 数据包边界
    ↓
解析各字段（ID, 坐标, 角点）
    ↓
有效性验证
    ↓
保存到 latest_qr_data
    ↓
返回true，表示成功
```

#### 数据格式解析

```
原始数据：
$QR01,150.5,200.3|120,80|180,80|180,140|120,140\n
 │    │   │ │   │ └─────┬──────┘└─────┬──────┘└─────┬──────┘└─────┬──────┘
 │    │   │ │   │   角点0      角点1        角点2        角点3
 │    │   │ └───┴─ 世界坐标Y
 │    │   └─ 世界坐标X
 │    └─ ID
 └─ 数据包开始

解析后的结构体：
{
  .id = "QR01",
  .world_x = 150.5,
  .world_y = 200.3,
  .corner_x[0] = 120, .corner_y[0] = 80,   // 角点0
  .corner_x[1] = 180, .corner_y[1] = 80,   // 角点1
  .corner_x[2] = 180, .corner_y[2] = 140,  // 角点2
  .corner_x[3] = 120, .corner_y[3] = 140,  // 角点3
  .timestamp = 12345
}
```

---

### 3. localization.h / localization.c - 定位导航模块

**文件位置：** `Core/Inc/localization.h`, `Core/Src/localization.c`

**用途：** 处理定位和自动导航控制

#### 关键数据结构

```c
// 机器人位姿结构体
typedef struct {
    float x;           // 位置X (cm)
    float y;           // 位置Y (cm)
    float theta;       // 朝向角 (弧度)
    uint32_t timestamp;
} RobotPose_t;

// 导航目标结构体
typedef struct {
    float target_x;
    float target_y;
    float tolerance;   // 到达距离阈值 (cm)
    bool active;
} NavigationTarget_t;

// PID参数结构体
typedef struct {
    float kp_linear;   // 线速度比例
    float kd_linear;   // 线速度微分
    float kp_angular;  // 角速度比例
    float kd_angular;  // 角速度微分
} PIDParameters_t;
```

#### 核心函数

| 函数 | 功能 |
|---|---|
| `Localization_Init()` | 初始化定位系统 |
| `Update_Position_From_QR()` | 用二维码数据更新位置 |
| `Get_Robot_Pose()` | 获取当前位姿指针 |
| `Set_Navigation_Target()` | 设置导航目标 |
| `Stop_Navigation()` | 停止导航 |
| `Navigate_Update()` | 执行一次导航控制（50Hz调用） |
| `Check_Target_Reached()` | 检查是否到达目标 |
| `Calculate_Navigation_Error()` | 计算导航误差 |
| `Set_PID_Parameters()` | 调整PID参数 |

#### 导航算法

**位置更新：**
```c
robot_pose.x = qr->world_x;          // 使用二维码世界坐标
robot_pose.y = qr->world_y;
robot_pose.theta = atan2(dy, dx);    // 使用角点计算朝向
```

**误差计算：**
```c
距离误差：distance = √((target_x - x)² + (target_y - y)²)
角度误差：angle_error = target_theta - current_theta
```

**控制命令生成（PD控制）：**
```c
// 线速度控制
v_linear = Kp_linear × distance + Kd_linear × (distance - distance_last)

// 角速度控制
v_angular = Kp_angular × angle_error + Kd_angular × (angle_error - angle_error_last)

// 发送给麦轮
Mecanum_Move(v_linear, 0, v_angular);
```

**导航状态机：**
```
未激活状态
    ↓
Set_Navigation_Target() → 激活导航
    ↓
Navigate_Update() 循环执行
    ├─ 计算误差
    ├─ PD控制生成速度
    ├─ 检查到达条件
    └─ 返回false
    ↓
到达目标时
    ├─ Motor_Stop_All()
    ├─ 设置 active=false
    └─ 返回true
```

---

### 4. main.c - 主程序与控制循环

**文件位置：** `Core/Src/main.c`

**用途：** 系统初始化和主控制循环

#### 关键函数

```c
int main(void)
```

#### 初始化流程

```c
HAL_Init()                    // HAL库初始化
    ↓
SystemClock_Config()          // 配置168MHz时钟
    ↓
MX_GPIO_Init()               // GPIO初始化
MX_DMA_Init()                // DMA初始化
MX_USART3_UART_Init()        // UART初始化
MX_TIM3_Init()               // 定时器PWM初始化
    ↓
Motor_Init()                 // 电机系统初始化
QR_Comm_Init()               // 通信系统初始化
QR_Comm_Start_Receive()      // 启动数据接收
Localization_Init()          // 定位系统初始化
    ↓
进入主控制循环 (50Hz)
```

#### 主控制循环逻辑

```c
while(1) {
    if (current_time - last_time >= 20) {  // 20ms = 50Hz
        
        // 第一步：接收处理二维码数据
        if (QR_Comm_Process(&qr_data)) {
            Update_Position_From_QR(&qr_data);
            // 打印调试信息
        }
        
        // 第二步：执行导航控制
        if (navigation_active) {
            if (Navigate_Update()) {
                navigation_active = false;
                Motor_Stop_All();
            }
        }
    }
}
```

#### 调试输出

系统通过UART3发送调试信息：
```
系统初始化消息：
"System initialized\r\n"

二维码接收：
"QR: QR01 @ (150.5, 200.3)\r\n"

导航完成：
"Target reached\r\n"
```

---

### 5. stm32f4xx_it.c - 中断处理

**文件位置：** `Core/Src/stm32f4xx_it.c`

**修改：** 在 `USART3_IRQHandler()` 中添加空闲中断处理

#### UART空闲中断处理

```c
void USART3_IRQHandler(void) {
    // 检查UART空闲中断标志
    if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE)) {
        __HAL_UART_CLEAR_IDLEFLAG(&huart3);  // 清除标志
        QR_UART_Idle_Callback();              // 调用回调
    }
    HAL_UART_IRQHandler(&huart3);
}
```

#### 中断流程

```
数据在UART RX线上
    ↓
DMA接收数据到缓冲区
    ↓
无新数据到来，UART空闲
    ↓
UART_FLAG_IDLE 被设置
    ↓
USART3_IRQHandler() 被触发
    ↓
QR_UART_Idle_Callback() 被调用
    ↓
获取DMA已传输的字节数
    ↓
数据复制到uart_buffer
    ↓
重启DMA接收
```

---

## 控制流总览

```
┌─────────────────────────────────────────────┐
│         系统启动与初始化                      │
│  Motor_Init()                               │
│  QR_Comm_Init() & QR_Comm_Start_Receive()   │
│  Localization_Init()                        │
└──────────────┬──────────────────────────────┘
               │
               ↓
┌─────────────────────────────────────────────┐
│         主控制循环 (50Hz, 20ms周期)          │
│                                              │
│  1. 接收K210数据                             │
│     QR_Comm_Process()                       │
│     ↓                                        │
│     Update_Position_From_QR()               │
│                                              │
│  2. 导航控制                                 │
│     if (navigation_active)                  │
│       Navigate_Update()                     │
│       ├─ 计算误差                           │
│       ├─ PD控制生成速度                     │
│       └─ Mecanum_Move()                     │
│                                              │
│  3. 监控状态                                 │
│     Check_Target_Reached()                  │
│     Motor_Stop_All()                        │
│                                              │
└──────────────┬──────────────────────────────┘
               │
               ↓ (循环)
```

---

## 数据流

### 接收数据流

```
K210模块 (Python)
  ↓ (UART, 115200 bps)
STM32 USART3_RX (PB11)
  ↓ (DMA接收)
DMA缓冲区
  ↓ (空闲中断)
uart_buffer (qr_comm.c)
  ↓ (主循环处理)
QR_Comm_Process()
  ↓ (解析)
QR_Data_t qr_data
  ↓ (位置更新)
RobotPose_t robot_pose
```

### 控制数据流

```
导航目标
  ↓
Navigate_Update()
  ├─ 计算距离误差
  ├─ 计算角度误差
  └─ PD控制
    ↓
Mecanum_Move()
  ├─ 运动学计算
  └─ 速度归一化
    ↓
Motor_SetSpeed() × 4
  ├─ 设置IN1/IN2
  └─ 设置PWM
    ↓
TIM3_CH1/2/3/4 → PWM信号
    ↓
TB6612驱动模块
    ↓
电机转动
```

---

## 代码量统计

| 模块 | 代码行数 | 类型 |
|---|---|---|
| motor.c | ~150 | 核心逻辑 |
| qr_comm.c | ~200 | 核心逻辑 |
| localization.c | ~250 | 核心逻辑 |
| main.c | ~100 | 主程序 |
| 中断处理 | ~10 | 集成 |
| **总计** | **~710** | **核心代码** |

（不包括HAL库和启动代码）

---

## 关键常数

```c
// motor.c
#define PWM_MAX     999    // 最大PWM值
#define PWM_MIN    -999    // 最小PWM值

// main.c
#define CONTROL_PERIOD_MS  20   // 50Hz = 20ms

// qr_comm.c
#define RX_BUFFER_SIZE 256  // UART接收缓冲区大小

// localization.c
.kp_linear = 0.5f;      // 线速度比例系数
.kd_linear = 0.1f;      // 线速度微分系数
.kp_angular = 0.3f;     // 角速度比例系数
.kd_angular = 0.05f;    // 角速度微分系数
```

---

## 编译依赖

```
main.c
  ├─ motor.h        (电机控制)
  ├─ qr_comm.h      (通信接收)
  └─ localization.h (定位导航)
      ├─ qr_comm.h  (数据结构)
      └─ motor.h    (电机控制)

stm32f4xx_it.c
  └─ qr_comm.h      (中断回调)
```

---

这份导览帮助你快速理解代码结构。建议按照以下顺序阅读：

1. 📖 本文件（代码导览）
2. 📖 IMPLEMENTATION.md（详细说明）
3. 💻 阅读源代码（motor.c → qr_comm.c → localization.c → main.c）
4. 🧪 编译和测试

