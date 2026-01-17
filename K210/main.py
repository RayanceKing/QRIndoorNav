import sensor, image, time, lcd
from modules import ybserial

# ================= 1. 硬件初始化 =================
# 使用Yahboom外置串口模块
serial = ybserial()

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
sensor.set_auto_gain(False)        # 关闭自动增益，提高识别稳定性
sensor.set_auto_whitebal(False)    # 关闭自动白平衡
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
last_heartbeat_time = time.ticks_ms()
heartbeat_interval = 2000  # 心跳间隔（ms）

print("QR Indoor Positioning System - K210 Module")
print("Waiting for QR codes...")

# 发送启动消息到STM32
time.sleep_ms(500)
init_msg = "$INIT,K210_READY\r\n"
num = serial.send(init_msg)
print("Sent INIT message to STM32, bytes:", num)

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
                u = int(corners[i][0])  # 转换为整数
                v = int(corners[i][1])
                img.draw_circle(u, v, 5, color=(255, 255, 0), thickness=2)
                img.draw_cross(u, v, color=(0, 255, 255), size=8, thickness=2)
                img.draw_string(u-10, v-20, str(i), color=(255, 255, 0), scale=1)
            img.draw_rectangle(code.rect(), color=(0, 255, 0), thickness=3)

            # 显示二维码信息
            info_str = "%s (%.1f,%.1f)" % (qr_id, world_x, world_y)
            img.draw_string(10, 10, info_str, color=(0, 255, 0), scale=2)

            # D. 发送数据到 STM32（控制发送频率）
            if time.ticks_diff(current_time, last_send_time) >= send_interval:
                # 构建数据包：$ID,world_x,world_y|u0,v0|u1,v1|u2,v2|u3,v3\n
                msg = "$%s,%d,%d|%d,%d|%d,%d|%d,%d|%d,%d\n" % (
                    qr_id, int(world_x), int(world_y),
                    int(corners[0][0]), int(corners[0][1]),
                    int(corners[1][0]), int(corners[1][1]),
                    int(corners[2][0]), int(corners[2][1]),
                    int(corners[3][0]), int(corners[3][1])
                )
                num = serial.send(msg)
                last_send_time = current_time
                print("Sent:", msg.strip())
                print("UART TX: %d bytes" % num)

                # LED 闪烁指示（如果启用）
                # led.value(0)
                # time.sleep_ms(50)
                # led.value(1)

            break  # 只处理第一个识别到的二维码
    else:
        # 未检测到二维码
        img.draw_string(10, 10, "Searching QR...", color=(255, 255, 255), scale=2)
        
        # 定期发送心跳消息
        if time.ticks_diff(current_time, last_heartbeat_time) >= heartbeat_interval:
            heartbeat_msg = "$HEARTBEAT\r\n"
            num = serial.send(heartbeat_msg)
            last_heartbeat_time = current_time
            print("Heartbeat sent, bytes:", num)

    lcd.display(img)
