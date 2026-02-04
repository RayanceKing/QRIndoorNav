import sensor, image, time, math, lcd
from modules import ybserial

# 初始化串口
serial = ybserial()

lcd.init()
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)       # 320x240，用于显示（检测前会resize）
sensor.skip_frames(time = 100)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
clock = time.clock()

tag_families = image.TAG36H11   # 仅识别 TAG36H11

# 相机焦距参数（160x120分辨率）
# 基于GC2145 + 2.8mm镜头标准参数
f_x = (2.8 / 3.984) * 160  # ≈ 112.1
f_y = (2.8 / 2.952) * 120  # ≈ 113.9
c_x = 160 * 0.5  # = 80
c_y = 120 * 0.5  # = 60

def family_name(tag):
    return "TAG36H11"

def degrees(radians):
    return (180 * radians) / math.pi

# Tag ID -> 世界坐标 (cm)
tag_world_db = {
    0: (0, 0),
    1: (40, 0),
    2: (40, 40),
    3: (0, 40),
}

# 标靶边长（cm）
marker_size_cm = 7.0

# 图像坐标标定参数（需要根据实际高度调整）
# 当K210距离标签7cm高度时，1个像素对应多少cm
# 这个参数需要通过实测来标定：
# 1. 把相机正对TAG0中心（cx=160, cy=120）
# 2. 记下K210发送的坐标应该是(0,0)
# 3. 如果显示偏差，调整这个参数
IMAGE_PIXEL_TO_CM = 0.08  # 初始估计，需要标定

# 发送控制变量 - 每个标签独立的发送时间记录
tag_last_send_time = {}  # tag_id -> last_send_timestamp
tag_send_interval = 250  # 每个标签最小发送间隔 250ms
tag_send_priority = []   # 标签发送优先级队列

# 发送初始化消息
time.sleep_ms(500)
serial.send("$INIT,K210_READY\r\n")
print("K210 initialized, ready to send AprilTag pose data")

while(True):
    clock.tick()
    img = sensor.snapshot()
    img_display = img.resize(292, 210)  # 调整显示尺寸以适配LCD
    current_time = time.ticks_ms()
    
    # 为检测创建缩小版本（满足<64K限制：160x120=19.2K）
    img_detect = img.copy().resize(160, 120)
    # 使用标定的焦距参数进行位姿估计
    tags = img_detect.find_apriltags(families=tag_families, fx=f_x, fy=f_y, cx=c_x, cy=c_y)
    
    if tags:
        valid_tags = []  # 收集所有有效的tag
        
        for tag in tags:
            tag_id = tag.id()
            
            # 检查是否在世界坐标表中
            if tag_id not in tag_world_db:
                continue
            
            world_x, world_y = tag_world_db[tag_id]
            
            # 直接获取位姿信息
            tx = tag.x_translation()  # cm为单位（需缩放marker_size）
            ty = tag.y_translation()
            tz = tag.z_translation()
            rx = degrees(tag.x_rotation())  # 转换为度
            ry = degrees(tag.y_rotation())
            rz = degrees(tag.z_rotation())
            
            # 绘制可视化
            img.draw_rectangle(tag.rect(), color=(255, 0, 0))
            img.draw_cross(tag.cx(), tag.cy(), color=(0, 255, 0))
            
            # 在tag附近显示ID和距离
            img.draw_string(tag.cx() - 10, tag.cy() - 20, "ID%d" % tag_id, color=(255, 255, 0), scale=2)
            dist = math.sqrt(tx*tx + ty*ty + tz*tz) * marker_size_cm
            img.draw_string(tag.cx() - 10, tag.cy() + 10, "D:%.0f" % dist, color=(0, 255, 255), scale=1)
            
            # 打印调试信息
            print("Tag ID %d: Tx=%d Ty=%d Tz=%d, Rx=%d Ry=%d Rz=%d" %
                  (tag_id, int(tx*marker_size_cm), int(ty*marker_size_cm), int(tz*marker_size_cm), int(rx), int(ry), int(rz)))
            
            # 保存tag信息用于发送
            valid_tags.append({
                'id': tag_id,
                'world': (world_x, world_y),
                'tag': tag,  # 保存tag对象引用
                'tx': tx, 'ty': ty, 'tz': tz,
                'rx': rx, 'ry': ry, 'rz': rz
            })
        
        # 多标签三角定位：使用图像坐标法，基于标定参数
        if len(valid_tags) >= 2:
            positions = []
            for tag_info in valid_tags[:2]:  # 使用前两个tag
                world_x, world_y = tag_info['world']
                tag = tag_info['tag']
                
                # tag在图像中的位置 (像素坐标，原始320x240)
                cx = tag.cx()
                cy = tag.cy()
                
                # 图像中心 (320x240的中心是160, 120)
                center_x = 160.0
                center_y = 120.0
                
                # TAG偏离中心的像素距离
                dx_pixel = cx - center_x
                dy_pixel = cy - center_y
                
                # 转换为实际距离偏移（使用标定参数）
                offset_x = dx_pixel * IMAGE_PIXEL_TO_CM
                offset_y = dy_pixel * IMAGE_PIXEL_TO_CM
                
                # 计算机器人的世界坐标
                robot_x = world_x - offset_x  # 减号是因为TAG越靠右，机器人越靠左
                robot_y = world_y - offset_y
                
                positions.append((robot_x, robot_y, tag_info['rz']))
                print("  TAG%d: cx=%d cy=%d → offset=(%.1f,%.1f) → robot=(%.1f,%.1f)" % 
                      (tag_info['id'], int(cx), int(cy), offset_x, offset_y, robot_x, robot_y))
            
            # 取平均值
            calc_world_x = sum(p[0] for p in positions) / len(positions)
            calc_world_y = sum(p[1] for p in positions) / len(positions)
            yaw = int(sum(p[2] for p in positions) / len(positions))
            
            # 发送融合后的位置
            if time.ticks_diff(current_time, tag_last_send_time.get(-1, 0)) >= tag_send_interval:
                msg = "$POS,-1,%d,%d,%d\r\n" % (
                    int(calc_world_x),
                    int(calc_world_y),
                    yaw
                )
                serial.send(msg)
                tag_last_send_time[-1] = current_time
                print("Triangulated POS=(%d,%d) yaw=%d from %d tags [CALIBRATED]" % 
                      (int(calc_world_x), int(calc_world_y), yaw, len(positions)))
        elif len(valid_tags) == 1:
            # 只有1个tag时，使用图像坐标法单标签定位
            tag_info = valid_tags[0]
            tag_id = tag_info['id']
            last_time = tag_last_send_time.get(tag_id, 0)
            
            if time.ticks_diff(current_time, last_time) >= tag_send_interval:
                world_x, world_y = tag_info['world']
                tag = tag_info['tag']
                
                # tag在图像中的位置
                cx = tag.cx()
                cy = tag.cy()
                
                # 偏移计算
                dx_pixel = cx - 160.0
                dy_pixel = cy - 120.0
                offset_x = dx_pixel * IMAGE_PIXEL_TO_CM
                offset_y = dy_pixel * IMAGE_PIXEL_TO_CM
                
                # 机器人世界坐标
                calc_world_x = world_x - offset_x
                calc_world_y = world_y - offset_y
                yaw = int(tag_info['rz'])
                
                msg = "$POS,%d,%d,%d,%d\r\n" % (
                    tag_id,
                    int(calc_world_x),
                    int(calc_world_y),
                    yaw
                )
                serial.send(msg)
                tag_last_send_time[tag_id] = current_time
                print("Single TAG%d cx=%d cy=%d offset=(%.1f,%.1f) POS=(%d,%d) yaw=%d" % 
                      (tag_id, int(cx), int(cy), offset_x, offset_y,
                       int(calc_world_x), int(calc_world_y), yaw))

    else:
        img_display.draw_string(4, 4, "Searching...", color=(255, 255, 255), scale=2)
    
    # 在屏幕中间绘制红色中心点
    center_x = 146  # 292 / 2
    center_y = 105  # 210 / 2
    img_display.draw_cross(center_x, center_y, color=(255, 0, 0), size=10, thickness=2)
    
    lcd.display(img_display)
    #print(clock.fps())
