# 基于STM32的二维码室内定位系统

## 项目简介
本毕业设计实现一套基于二维码的室内定位与导航系统。系统由 STM32F407ZGT6 最小系统板负责运动控制与里程计融合，K210 视觉模块负责二维码识别，驱动采用 TB6612 四路电机驱动模块，轮组使用 520 编码器麦轮套装。

适用场景：移动机器人（小车）室内定位与路径导航实验平台。

## 主要特性
- 实时二维码检测与识别（K210）。
- 基于编码器的里程计与运动控制（STM32 HAL）。
- 通过 TB6612 驱动电机并支持 PWM 速度控制与方向控制。
- 支持使用 VSCode + EIDE + OpenOCD 在 macOS 上编译与烧录（gcc-arm-embedded）。
- 蓝牙远程控制支持（DX-WF24-A 模块，透传模式）。

## 硬件清单
- 主控：STM32F407ZGT6 最小系统板
- 视觉模块：K210（带摄像头，运行识别脚本）
- 轮组：520 编码器麦轮套装（含 A/B 编码器信号）
- 驱动：TB6612 四路电机驱动模块
- 无线通信：DX-WF24-A（蓝牙 + Wi-Fi 模块）
- 供电：电池或稳定电源（按电机/驱动额定电压配置）
- 调试/烧录：ST-Link / J-Link

## 引脚映射（与 `Core/Inc/main.h` 对齐）
> 下面列出的引脚来自 `Core/Inc/main.h` 中的 `*_Pin` 定义，已与工程代码保持一致。PWM 输出、编码器定时器/通道与 K210 串口需要在代码中确认并映射到合适的引脚/通道。

| 功能 | STM32 引脚（代码定义） | 说明 |
|---|---:|---|
| TB6612 STBY | PB15 (`STBY_Pin`) | 驱动使能/待机控制 |
| AIN1 | PC7 (`AIN1_Pin`) | TB6612 A 通道 1 方向控制 |
| AIN2 | PC6 (`AIN2_Pin`) | TB6612 A 通道 2 方向控制 |
| BIN1 | PD7 (`BIN1_Pin`) | TB6612 B 通道 1 方向控制 |
| BIN2 | PD6 (`BIN2_Pin`) | TB6612 B 通道 2 方向控制 |
| CIN1 | PC10 (`CIN1_Pin`) | TB6612 C 通道 1 方向控制 |
| CIN2 | PG15 (`CIN2_Pin`) | TB6612 C 通道 2 方向控制 |
| DIN1 | PD2 (`DIN1_Pin`) | TB6612 D 通道 1 方向控制 |
| DIN2 | PC11 (`DIN2_Pin`) | TB6612 D 通道 2 方向控制 |

注意：
- `main.h` 中目前定义的是各通道的方向控制与 STBY 引脚（见上表）。实际的 PWM 输出与 UART 在 `Core/Src` 中初始化：

- PWM 输出（来自 `Core/Src/stm32f4xx_hal_msp.c` 与 `Core/Src/main.c`，使用 `TIM3`）：

  - `TIM3_CH1` -> PA6 （对应工程中第 1 路 PWM）
  - `TIM3_CH2` -> PA7 （对应工程中第 2 路 PWM）
  - `TIM3_CH3` -> PB0 （对应工程中第 3 路 PWM）
  - `TIM3_CH4` -> PB1 （对应工程中第 4 路 PWM）

- 串口（K210 通信）在工程中使用 `USART3`，映射为：

  - `USART3_TX` -> PB10
  - `USART3_RX` -> PB11

- 串口（DX-WF24-A 蓝牙模块通信）在工程中使用 `USART2`，映射为：

  - `USART2_TX` -> PA2
  - `USART2_RX` -> PA3
  - 波特率：115200 baud
  - 数据位：8
  - 停止位：1
  - 校验位：无
  - 流控：无
  - 工作模式：透传模式（通过AT+BLUFISEND=1命令进入）

- 编码器（增量 A/B 信号）在当前代码中尚未看到定时器输入捕获或 `HAL_TIM_Encoder` 的初始化，因此编码器引脚尚未在工程中定义。建议将编码器 A/B 信号接到支持外部中断或定时器输入捕获的引脚，然后在代码中配置对应的 `TIMx` 为编码器接口（`HAL_TIM_Encoder_Init`）。

## 电源与接线注意
- 电机驱动（TB6612）的电源应单独供电，确保电机电源与 STM32 共地。
- K210 与摄像头供电应按模块规格供电（通常 3.3V 或 5V，视模块而定）。
- 编码器的 VCC/GND/信号线需稳固，若存在抖动建议加入 RC 滤波或硬件消抖电路。

## 软件架构与文件位置
- STM32 固件：`Core/`（主程序与 HAL 配置），主要入口 [Core/Src/main.c](Core/Src/main.c)。
- K210 脚本：位于 [K210/QRRecognize.py](K210/QRRecognize.py)。
- 驱动与第三方库：`Drivers/STM32F4xx_HAL_Driver`。
- 构建输出：`build/`。
- **使用说明文档**：[QR_SYSTEM_USAGE.md](QR_SYSTEM_USAGE.md)（详细的功能说明和API文档）

## 系统功能

### 已实现功能
✅ **完整的电机控制系统**
- TB6612 四路电机驱动初始化与控制
- 单个电机速度与方向控制（-999 到 999）
- 麦克纳姆轮全向移动控制（vx, vy, omega）

✅ **二维码识别与数据处理**
- K210 串口通信（USART3，115200 波特率，DMA接收）
- 二维码数据包解析（ID、世界坐标、四角点像素坐标）
- 数据有效性验证

✅ **定位与导航系统**
- 基于二维码的位置更新（简化PnP算法）
- 机器人位姿估计（x, y, theta）
- 自动导航到目标点（PD控制器）
- 到达检测与自动停止

✅ **实时控制循环**
- 50Hz 主控制频率
- 异步串口数据接收（UART空闲中断 + DMA）
- 状态管理与导航使能控制

✅ **蓝牙遥控功能**
- DX-WF24-A 蓝牙模块集成（UART2，115200 波特率）
- 手机应用通过蓝牙发送控制命令
- 实时位置反馈（5Hz 发送频率）
- 支持自动导航与手动控制


### DX-WF24-A 蓝牙通信协议
**连接流程：**
1. DX-WF24-A 模块上电后进行蓝牙初始化
2. 手机端通过蓝牙搜索并连接模块
3. 连接成功后模块向 STM32 发送 `BLE_CONNECT_SUCCESS` 消息
4. STM32 自动发送 `AT+BLUFISEND=1\r\n` 进入透传模式
5. 之后所有收发数据为透传模式（无 AT 命令识别）

**手机 → STM32 命令格式：**
```
CMD:GOTO,x,y        // 自动导航到坐标 (x,y)
CMD:MOVE,dir,speed  // 手动控制移动 (dir: F/B/L/R/TL/TR, speed: 0-999)
CMD:STOP            // 停止运动
CMD:QUERY           // 查询当前状态
```

**STM32 → 手机 反馈格式：**
```
POS:x,y,heading     // 当前位置坐标与航向角（整数精度，5Hz 发送频率）
STATUS:state        // 状态反馈 (IDLE/NAV/MANUAL)
```
### K210 数据格式
```
$QR_ID,world_x,world_y|corner1_x,corner1_y|corner2_x,corner2_y|corner3_x,corner3_y|corner4_x,corner4_y
```

示例：`$QR01,150.5,200.3|120,80|180,80|180,140|120,140`

### 主要 API 函数
```c
// 电机控制
void Motor_Init(void);                              // 初始化电机
void Motor_SetSpeed(uint8_t motor_id, int16_t speed);  // 设置单个电机速度
void Motor_Stop_All(void);                          // 停止所有电机
void Mecanum_Move(float vx, float vy, float omega); // 麦轮全向移动

// 定位与导航
void Process_QR_Data(uint8_t *data);                // 解析二维码数据
void Update_Position_From_QR(QR_Data_t *qr);        // 更新位置
void Set_Navigation_Target(float x, float y);       // 设置导航目标
void Navigate_To_Target(float tx, float ty);        // 导航控制
void Stop_Navigation(void);                         // 停止导航
```

详细使用说明请参考 **[QR_SYSTEM_USAGE.md](QR_SYSTEM_USAGE.md)**。

## macOS 开发环境（VSCode + EIDE + OpenOCD + gcc-arm）
1. 安装 Homebrew（若未安装）：

```bash
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
```

2. 安装交叉编译工具链与 OpenOCD、stlink 等：

```bash
brew tap ArmMbed/homebrew-formulae
brew install arm-none-eabi-gcc
brew install open-ocd
brew install stlink
```

3. 在 VSCode 中安装并配置 EIDE（或你使用的 STM32 插件），工作区已经包含 `QRIndoorNav.code-workspace` 和 VSCode 任务（`build` / `flash` / `build and flash`）。

4. 在 VSCode 中使用任务：

  - 运行 `Tasks: Run Task` -> 选择 `build` 编译固件。
  - 运行 `flash` 或 `build and flash` 烧录固件到 MCU（需配置上传工具）。

## 使用 OpenOCD 在 macOS 上刷写（示例）
> 以下命令为示例，请根据你系统上 OpenOCD 的接口/目标配置文件修改。

```bash
# 使用 ST-Link 接口示例
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
  -c "program build/your_firmware.elf verify reset exit"

# 若使用 bin 文件：
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
  -c "init; reset init; flash write_image erase build/your_firmware.bin 0x08000000; reset run; exit"
```

如果使用 `st-flash`（来自 `stlink` 工具包）：

```bash
st-flash write build/your_firmware.bin 0x08000000
```

## 调试
- 使用 SWD 接入 ST-Link/J-Link，并在 VSCode/EIDE 或 STM32CubeIDE 中进行单步调试。
- 通过串口查看运行日志，串口参数请参照代码中配置（波特率通常为 115200）。

## 运行流程（高层）
1. K210 捕获图像并进行二维码识别，提取二维码 ID/相对位置信息并通过 UART 发送给 STM32。
2. STM32 接收识别结果，结合编码器里程计进行定位融合与路径规划。
3. MCU 输出 PWM 控制 TB6612 驱动电机执行导航动作，并根据反馈调整速度与方向。

## 常见问题与排查
- 无法识别二维码：检查摄像头视野、二维码尺寸与光照；在 `K210/QRRecognize.py` 中启用调试输出。
- 编码器读数异常：检查信号连线、接地与定时器配置；考虑软件去抖或滤波。
- 电机无响应或异常发热：确认 TB6612 电源、使能管脚与 PWM 占空比；确保电机电流不超额定值。

## 致谢
- 感谢 STM32 HAL 库与 K210 社区的开源支持。

---
