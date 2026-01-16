# 快速入门指南

## 编译状态

✅ **编译成功！**

```
Text (程序代码)：38,080 bytes
Data (初始数据)：  484 bytes
BSS  (未初始化)：2,748 bytes
总计：41,312 bytes (~40 KB)
```

固件文件位置：`build/debug/QRIndoorNav.elf` 或 `build/debug/QRIndoorNav.hex`

## 快速烧录

### 方法1：使用 st-flash（推荐macOS）

```bash
cd /Users/rayanceking/Develop/QRIndoorNav
st-flash write build/debug/QRIndoorNav.bin 0x08000000
```

### 方法2：使用 OpenOCD

```bash
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
  -c "program build/debug/QRIndoorNav.elf verify reset exit"
```

### 方法3：VSCode集成

在VSCode中运行任务：
- `Ctrl+Shift+P` → `Tasks: Run Task` → 选择 `build and flash`

## 功能测试

### 1. 验证系统启动
- 连接STM32到电脑
- 打开串口监视器（波特率 115200）
- 应该看到：`System initialized\r\n`

### 2. 测试电机
在串口发送命令（需要在 `main()` 中实现命令解析）：
```
M A 500    // Motor A 速度 500
M B -300   // Motor B 速度 -300 (反向)
S          // Stop all motors
```

### 3. 测试二维码识别
- 启动K210模块
- 在摄像头前放置二维码
- 串口应显示接收到的数据：`QR: QR01 @ (150.5, 200.3)`

### 4. 测试导航
取消 `main()` 中的导航示例注释，机器人应自动导航到目标点。

## 模块结构

```
Core/Inc/
  ├── motor.h          # 电机驱动接口
  ├── qr_comm.h        # K210通信接口
  └── localization.h   # 定位导航接口

Core/Src/
  ├── main.c           # 主程序与控制循环
  ├── motor.c          # 电机驱动实现
  ├── qr_comm.c        # 通信实现
  ├── localization.c   # 定位导航实现
  └── stm32f4xx_it.c   # 中断处理（UART空闲中断）
```

## 下一步开发

### 1. 实现命令行接口
在 `main.c` 中添加UART命令解析，支持：
- 电机单独控制
- 导航目标设置
- 参数调整

### 2. 添加编码器支持
- 配置定时器为编码器接口
- 实现速度反馈控制
- 集成里程计与二维码融合

### 3. 数据记录
- 记录轨迹数据到SD卡或内存
- 实现实时性能监测

### 4. 高级导航
- 路径规划算法
- 障碍物避免
- 多目标导航

## 调试技巧

### 启用DEBUG输出
在 `main.c` 中添加更多串口打印：
```c
char debug_msg[128];
snprintf(debug_msg, sizeof(debug_msg), 
        "Position: (%.1f, %.1f, %.2f)\r\n", 
        pose.x, pose.y, pose.theta);
HAL_UART_Transmit(&huart3, (uint8_t *)debug_msg, strlen(debug_msg), 10);
```

### 使用逻辑分析仪
- PWM 输出在 PA6, PA7, PB0, PB1
- UART 信号在 PB10 (TX), PB11 (RX)

### 性能监测
使用 `HAL_GetTick()` 测量函数执行时间：
```c
uint32_t start = HAL_GetTick();
// ... 函数执行 ...
uint32_t elapsed = HAL_GetTick() - start;
```

## 常用命令

```bash
# 仅编译
make

# 清空并重新编译
make clean && make

# 查看编译优化
arm-none-eabi-objdump -S build/debug/QRIndoorNav.elf | head -100

# 生成反汇编
arm-none-eabi-objdump -d build/debug/QRIndoorNav.elf > firmware.asm
```

## 故障排查

| 症状 | 可能原因 | 解决方案 |
|---|---|---|
| 固件无法烧录 | ST-Link连接问题 | 重新连接，检查驱动 |
| 系统无启动消息 | UART配置错误 | 检查波特率和引脚 |
| 电机无反应 | PWM未启动 | 检查 `Motor_Init()` 执行 |
| 无法接收二维码 | DMA中断未配置 | 检查UART空闲中断 |

## 相关文档

- [详细实现说明](IMPLEMENTATION.md)
- [原始README](README.md)
- [K210 Python脚本](K210/main.py)

---

**祝贺你！你的毕业设计框架已完成。现在可以开始集成、测试和优化了！** 🎉

