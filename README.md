# 基于STM32的二维码室内定位系统

## 项目简介
本毕业设计实现一套基于二维码的室内定位与导航系统。系统由 STM32F407ZGT6 最小系统板负责运动控制与里程计融合，K210 视觉模块负责二维码识别，驱动采用 TB6612 四路电机驱动模块，轮组使用 520 编码器麦轮套装。

适用场景：移动机器人（小车）室内定位与路径导航实验平台。

## 主要特性
- 实时二维码检测与识别（K210）。
- 基于编码器的里程计与运动控制（STM32 HAL）。
- 通过 TB6612 驱动电机并支持 PWM 速度控制与方向控制。
- 支持使用 VSCode + EIDE + OpenOCD 在 macOS 上编译与烧录（gcc-arm-embedded）。

## 硬件清单
- 主控：STM32F407ZGT6 最小系统板
- 视觉模块：K210（带摄像头，运行识别脚本）
- 轮组：520 编码器麦轮套装（含 A/B 编码器信号）
- 驱动：TB6612 四路电机驱动模块
- 供电：电池或稳定电源（按电机/驱动额定电压配置）
- 调试/烧录：ST-Link / J-Link

## 建议引脚表（示例）
> 注：以下引脚为建议映射，请根据实际最小系统板与 PCB 引脚资源、以及工程中 `Core/Inc/main.h` 的配置做对应修改。

| 功能 | 建议 STM32 引脚 | 说明 |
|---|---:|---|
| 左轮编码器 A | TIM2_CH1 — PA0 | 定时器输入捕获（增量编码器） |
| 左轮编码器 B | TIM2_CH2 — PA1 | 定时器输入捕获 |
| 右轮编码器 A | TIM3_CH1 — PA6 | 定时器输入捕获 |
| 右轮编码器 B | TIM3_CH2 — PA7 | 定时器输入捕获 |
| 电机1 PWM | TIM4_CH1 — PB6 | PWM 输出给 TB6612 PWMA|
| 电机1 DIR | PB0 | AIN1/AIN2 方向控制 |
| 电机2 PWM | TIM4_CH2 — PB7 | PWM 输出 |
| 电机2 DIR | PB1 | 方向控制 |
| 电机3 PWM | TIM4_CH3 — PB8 | PWM 输出 |
| 电机3 DIR | PB2 | 方向控制 |
| 电机4 PWM | TIM4_CH4 — PB9 | PWM 输出 |
| 电机4 DIR | PB3 | 方向控制 |
| K210 串口（UART）TX | USART2_TX — PA2 | K210 接收主控数据/命令 |
| K210 串口（UART）RX | USART2_RX — PA3 | K210 发送识别结果 |
| SWDIO | PA13 | SWD 调试线 |
| SWCLK | PA14 | SWD 时钟 |
| NRST | NRST 引脚 | 外部复位（可选） |

如果你的编码器或电机数量较少，可只使用前两对 PWM/方向。实际在代码中使用的定时器/通道需与 `Core/Src` 中的配置一致。

## 电源与接线注意
- 电机驱动（TB6612）的电源应单独供电，确保电机电源与 STM32 共地。
- K210 与摄像头供电应按模块规格供电（通常 3.3V 或 5V，视模块而定）。
- 编码器的 VCC/GND/信号线需稳固，若存在抖动建议加入 RC 滤波或硬件消抖电路。

## 软件架构与文件位置
- STM32 固件：`Core/`（主程序与 HAL 配置），主要入口 [Core/Src/main.c](Core/Src/main.c)。
- K210 脚本：位于 [K210/QRRecognize.py](K210/QRRecognize.py)。
- 驱动与第三方库：`Drivers/STM32F4xx_HAL_Driver`。
- 构建输出：`build/`。

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

如果你希望我把上表中的引脚与 `Core/Inc/main.h` 中的实际定义对齐，我可以读取并同步两者（需要我检查该文件并做改动的话请确认）。
