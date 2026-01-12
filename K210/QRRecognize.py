import sensor, image, time, lcd
from fpioa_manager import fm
from machine import UART

# ================= 1. 硬件初始化 =================
# 串口映射：IO10(TX) -> STM32 PB11
fm.register(10, fm.fpioa.UART1_TX, force=True)
uart = UART(UART.UART1, 115200, 8, 1, 0, timeout=1000, read_buf_len=4096)

lcd.init()
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA) # 320x240
sensor.skip_frames(time = 2000)

while(True):
    img = sensor.snapshot()
    res = img.find_qrcodes()

    if res:
        for code in res:
            # A. 获取角点坐标 [(u0,v0), (u1,v1), (u2,v2), (u3,v3)]
            # 论文指出 PnP 算法需要这 4 个特征点来求解位姿
            corners = code.corners()

            # B. 提取内容 (例如二维码内写着 "ID1,150,200")
            # 存储的是该二维码在世界坐标系中的绝对位置 [cite: 11, 316]
            payload = code.payload()

            # C. 绘制角点和边框，辅助观察
            for i in range(4):
                img.draw_circle(corners[i][0], corners[i][1], 5, color=(255, 255, 0))
            img.draw_rectangle(code.rect(), color=(0, 255, 0), thickness=2)

            # D. 构建符合论文 PnP 需求的数据包
            # 格式：$内容|u0,v0|u1,v1|u2,v2|u3,v3#
            # 这样 STM32 才能通过像素坐标解算出相对位姿 [cite: 31, 416]
            msg = "$%s|%d,%d|%d,%d|%d,%d|%d,%d#" % (
                payload,
                corners[0][0], corners[0][1],
                corners[1][0], corners[1][1],
                corners[2][0], corners[2][1],
                corners[3][0], corners[3][1]
            )
            uart.write(msg)
            print("Sent PnP Data:", msg)

    lcd.display(img)
