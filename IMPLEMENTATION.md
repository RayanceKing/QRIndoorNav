# 二维码室内定位系统 - 使用说明

## 系统架构

本系统由以下主要模块组成：

### 1. **电机控制模块** (`motor.h/c`)
负责TB6612四路电机驱动的控制。

**主要函数：**
- `Motor_Init()` - 初始化电机系统
- `Motor_SetSpeed(motor_id, speed)` - 设置单个电机速度 (-999~999)
- `Motor_Stop_All()` - 停止所有电机
- `Mecanum_Move(vx, vy, omega)` - 麦轮全向移动

**麦轮配置：**
```
   左前(A)  右前(B)
      ↙  ↘
    ←  机器人  →
      ↖  ↗
   左后(C)  右后(D)
```

**运动学模型：**
```
vA = vx + vy + omega  (左前)
vB = vx - vy - omega  (右前)
vC = vx - vy + omega  (左后)
vD = vx + vy - omega  (右后)
```

### 2. **通信模块** (`qr_comm.h/c`)
负责与K210模块通过UART进行通信，接收并解析二维码数据。

**数据格式：**
```
$QR_ID,world_x,world_y|corner1_x,corner1_y|corner2_x,corner2_y|corner3_x,corner3_y|corner4_x,corner4_y\n
```

示例：`$QR01,150.5,200.3|120,80|180,80|180,140|120,140\n`

**主要函数：**
- `QR_Comm_Init()` - 初始化通信
- `QR_Comm_Start_Receive()` - 启动DMA接收
- `QR_Comm_Process(qr_data)` - 处理接收的数据
- `QR_Comm_Get_Latest()` - 获取最新二维码数据

### 3. **定位导航模块** (`localization.h/c`)
负责根据二维码数据更新机器人位置，并执行导航控制。

**主要函数：**
- `Localization_Init()` - 初始化定位系统
- `Update_Position_From_QR(qr)` - 用二维码数据更新位置
- `Get_Robot_Pose()` - 获取当前机器人位姿 (x, y, theta)
- `Set_Navigation_Target(x, y, tolerance)` - 设置导航目标
- `Navigate_Update()` - 执行一次导航控制循环
- `Check_Target_Reached()` - 检查是否到达目标

**控制器参数：**
使用PD控制器进行导航控制，可通过 `Set_PID_Parameters()` 调整。

## 硬件连接

### 电机与驱动
| TB6612通道 | 方向控制 | PWM输出 | 麦轮位置 |
|---|---|---|---|
| A | PC7(AIN1), PC6(AIN2) | PA6 (TIM3_CH1) | 左前 |
| B | PD7(BIN1), PD6(BIN2) | PA7 (TIM3_CH2) | 右前 |
| C | PC10(CIN1), PG15(CIN2) | PB0 (TIM3_CH3) | 左后 |
| D | PD2(DIN1), PC11(DIN2) | PB1 (TIM3_CH4) | 右后 |
| STBY | PB15 | - | 驱动使能 |

### 通信
- **K210串口**：USART3 (PB10=TX, PB11=RX)
- **波特率**：115200 bps

## 主控制循环

主控制循环在 `main()` 函数中实现，运行频率为 **50Hz**（20ms周期）。

**循环流程：**
1. 接收K210数据
2. 如果收到新的二维码数据，更新机器人位置
3. 如果导航使能，执行导航控制
4. 计算与发送电机速度命令

**示例代码：**
```c
// 设置导航目标，机器人将自动导航到 (300, 300)
Set_Navigation_Target(300.0f, 300.0f, 10.0f);  // 距离阈值 10cm

// 在主循环中，导航会自动执行
while (1) {
    if (Navigate_Update()) {
        // 已到达目标
        break;
    }
}
```

## 调试与使用

### 编译与烧录

```bash
# 清空旧的构建
make clean

# 编译
make

# 烧录（使用 st-flash）
st-flash write build/debug/QRIndoorNav.bin 0x08000000
```

### 使用OpenOCD烧录

```bash
# 使用 ST-Link
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
  -c "program build/debug/QRIndoorNav.elf verify reset exit"
```

### 串口调试

使用串口工具连接到STM32的USART3 (115200 bps)，可以看到：
- 系统初始化消息
- 接收到的二维码数据
- 导航状态

**推荐工具：**
- minicom
- CoolTerm
- VS Code Serial Monitor

```bash
# macOS 使用 minicom
minicom -D /dev/tty.usbserial-* -b 115200
```

## 自定义配置

### 调整导航PID参数

在 `localization.c` 中修改：
```c
static PIDParameters_t pid_params = {
    .kp_linear = 0.5f,    // 线速度比例
    .kd_linear = 0.1f,    // 线速度微分
    .kp_angular = 0.3f,   // 角速度比例
    .kd_angular = 0.05f,  // 角速度微分
};
```

### 调整电机速度限制

在 `motor.h` 中修改：
```c
#define PWM_MAX     999  /* 最大PWM值 */
```

### 修改二维码数据库

在 `K210/main.py` 中修改：
```python
qr_database = {
    "QR01": (150.0, 200.0),  # 二维码ID: (世界坐标X, 世界坐标Y)
    "QR02": (300.0, 200.0),
    ...
}
```

## 常见问题

### Q: 电机不转或转向错误
**A:** 
1. 检查电源连接和电机参数
2. 调整 `motor.c` 中的方向控制逻辑
3. 确认PWM信号到达电机驱动

### Q: 无法接收K210数据
**A:**
1. 检查UART连接 (TX/RX 是否正确)
2. 确认波特率为115200
3. 查看串口调试信息

### Q: 导航路径不稳定
**A:**
1. 调整PID参数，使用较小的Kp值
2. 确保K210二维码识别率
3. 检查编码器精度

## 技术参数

- **系统主频**：168MHz (STM32F407)
- **控制频率**：50Hz
- **UART波特率**：115200 bps
- **电机PWM频率**：1kHz (TIM3配置)
- **电机数量**：4 (Mecanum轮)
- **定位精度**：依赖二维码识别精度

## 许可证

本项目在MIT协议下开源。

---

**最后更新**：2026年1月16日

