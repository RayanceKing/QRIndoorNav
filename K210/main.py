import sensor, image, time, lcd
from fpioa_manager import fm
from machine import UART
from board import board_info

# ================= 1. 硬件初始化 =================
# 串口映射：IO10(TX) -> STM32 PB11 (USART3_RX)
fm.register(10, fm.fpioa.UART1_TX, force=True)
uart = UART(UART.UART1, 115200, 8, 1, 0, timeout=1000, read_buf_len=4096)

# LED 指示灯（可选，用于状态指示）
# fm.register(board_info.LED_R, fm.fpioa.GPIO0)
# led = GPIO(GPIO.GPIO0, GPIO.OUT)
# led.value(1)  # 初始熄灭（低电平点亮）

lcd.init()
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA) # 320x240
sensor.set_hmirror(0)              # 水平镜像（根据安装方向调整）
sensor.set_vflip(0)                # 垂直翻转（根据安装方向调整）
sensor.skip_frames(time = 2000)

# ================= 2. 二维码世界坐标数据库 =================
# 存储每个二维码在世界坐标系中的位置（单位：cm）
# 如果二维码本身包含坐标信息（格式：ID,X,Y），则此字典可选
qr_database = {
    "QR01": (150.0, 200.0),
    "QR02": (300.0, 200.0),
    "QR03": (150.0, 400.0),
    "QR04": (300.0, 400.0),
}

# ================= 3. 全局变量 =================
last_send_time = time.ticks_ms()
send_interval = 100  # 最小发送间隔（ms），避免数据洪泛

print("QR Indoor Positioning System - K210 Module")
print("Waiting for QR codes...")

# ================= 4. 主循环 =================
while(True):
    img = sensor.snapshot()
    res = img.find_qrcodes()

    current_time = time.ticks_ms()

    if res:
        for code in res:
            # A. 获取角点坐标 [(u0,v0), (u1,v1), (u2,v2), (u3,v3)]
            # PnP 算法需要这 4 个特征点来求解相机到二维码的位姿
            corners = code.corners()

            # B. 提取二维码内容
            # 支持两种格式：
            # 1. 直接包含坐标：QR01,150.5,200.3
            # 2. 只有ID：QR01（从数据库查询坐标）
            payload = code.payload()

            # 尝试解析payload中的坐标
            parts = payload.split(',')
            if len(parts) == 3:
                # 格式1：二维码内容直接包含坐标
                qr_id = parts[0]
                world_x = float(parts[1])
                world_y = float(parts[2])
            elif payload in qr_database:
                # 格式2：从数据库查询
                qr_id = payload
                world_x, world_y = qr_database[payload]
            else:
                # 未知二维码，跳过
                img.draw_string(10, 10, "Unknown QR: " + payload, color=(255, 0, 0), scale=2)
                continue

            # C. 绘制角点和边框（可视化）
            for i in range(4):
                img.draw_circle(corners[i][0], corners[i][1], 5, color=(255, 255, 0))
                img.draw_string(corners[i][0]-10, corners[i][1]-20, str(i), color=(255,255,0))
            img.draw_rectangle(code.rect(), color=(0, 255, 0), thickness=2)

            # 显示二维码信息
            info_str = "%s (%.1f,%.1f)" % (qr_id, world_x, world_y)
            img.draw_string(10, 10, info_str, color=(0, 255, 0), scale=2)

            # D. 发送数据到 STM32（控制发送频率）
            if time.ticks_diff(current_time, last_send_time) >= send_interval:
                # 构建数据包：$ID,world_x,world_y|u0,v0|u1,v1|u2,v2|u3,v3\n
                msg = "$%s,%.1f,%.1f|%d,%d|%d,%d|%d,%d|%d,%d\n" % (
                    qr_id, world_x, world_y,
                    corners[0][0], corners[0][1],
                    corners[1][0], corners[1][1],
                    corners[2][0], corners[2][1],
                    corners[3][0], corners[3][1]
                )
                uart.write(msg)
                last_send_time = current_time
                print("Sent:", msg.strip())

                # LED 闪烁指示（如果启用）
                # led.value(0)
                # time.sleep_ms(50)
                # led.value(1)

            break  # 只处理第一个识别到的二维码
    else:
        # 未检测到二维码
        img.draw_string(10, 10, "Searching QR...", color=(255, 255, 255), scale=2)

    lcd.display(img)
