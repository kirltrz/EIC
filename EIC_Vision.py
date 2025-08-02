import cv2 # type: ignore
import numpy as np # type: ignore
import serial # type: ignore
from pyzbar import pyzbar # type: ignore
import threading
import time

# UART串口配置
ser = serial.Serial('/dev/serial0', 115200, timeout=1)

# 定义模式常量
MODE_IDLE = 0x00
MODE_QR_CODE = 0x01
MODE_HIGH_SATURATION = 0x02  # 高饱和度区域识别
MODE_CIRCLE = 0x03   # 圆形识别
MODE_CALI_TURNTABLE = 0x04   # 转台校准模式
MODE_GET_CIRCLE_COLOR = 0x05   # 获取圆形颜色模式

# 定义颜色常量
COLOR_RED = 0x01
COLOR_GREEN = 0x02
COLOR_BLUE = 0x04

# 定义帧头和帧尾
FRAME_HEADER = 0x7B
FRAME_FOOTER = 0x7D

# 高饱和度识别参数
SATURATION_THRESHOLD = 100  # 饱和度阈值
AREA_THRESHOLD = 3000      # 面积阈值

# DEBUG模式开关 (类似C语言宏定义)
# 设置为True: 启用图形界面显示和键盘控制
# 设置为False: 禁用图形界面，程序在后台运行，只通过UART控制
DEBUG_MODE = True

# 图像尺寸参数（用于计算偏移量）
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FRAME_CENTER_X = FRAME_WIDTH // 2   # 320
FRAME_CENTER_Y = FRAME_HEIGHT // 2  # 240

# 曝光时间控制参数
AUTO_EXPOSURE_ENABLED = False  # 是否启用自动曝光 (False=手动曝光, True=自动曝光)
MANUAL_EXPOSURE_TIME = 200      # 手动曝光时间 (较低值避免过亮，从50开始测试)
EXPOSURE_ADJUSTMENT_STEP = 10  # 键盘调整曝光时间的步长

# 曝光值尝试范围（针对你的摄像头类型调整）
EXPOSURE_MIN_RANGE = 1         # 最小曝光值（最暗）
EXPOSURE_MAX_RANGE = 10000     # 最大曝光值（最亮）

# 图像质量控制参数
DEFAULT_CONTRAST = 50          # 默认对比度 (0-100)
DEFAULT_SATURATION = 64        # 默认饱和度 (通常0-100或0-255)
CONTRAST_ADJUSTMENT_STEP = 5   # 对比度调整步长
SATURATION_ADJUSTMENT_STEP = 5 # 饱和度调整步长

# 摄像头方向补偿
# 如果摄像头安装方向相差180度，设置为True
# True: 反向偏移量输出 (画面上下左右颠倒时使用)
# False: 正常偏移量输出
CAMERA_ROTATION_180 = True

# 全局变量，用于存储当前模式和摄像头帧
current_mode = MODE_IDLE
latest_frame = None
frame_lock = threading.Lock()
target_color = 0x00  # 添加全局变量存储目标颜色
current_exposure = MANUAL_EXPOSURE_TIME  # 当前曝光时间值
current_contrast = DEFAULT_CONTRAST     # 当前对比度值
current_saturation = DEFAULT_SATURATION # 当前饱和度值
camera_object = None  # 全局摄像头对象，用于曝光控制

# 帧处理时间统计变量
frame_processing_time = 0.0  # 当前帧处理时间(毫秒)
fps = 0.0                    # 当前FPS
last_fps_time = time.time()  # 上次FPS计算时间
frame_count = 0              # 帧计数器

def calculate_bcc(data):
    bcc = 0
    for byte in data:
        bcc ^= byte
    return bcc

def calculate_offset(x, y):
    """计算相对于屏幕中心的偏移量
    坐标系：向右为x+，向上为y+
    """
    offset_x = x - FRAME_CENTER_X
    offset_y = FRAME_CENTER_Y - y  # 翻转y轴，图像坐标向下为正，转换为向上为正
    
    # 如果摄像头安装方向相差180度，则反向偏移量
    if CAMERA_ROTATION_180:
        offset_x = -offset_x
        offset_y = -offset_y
    
    return offset_x, offset_y

def send_data_packet(mode, data1, data2, color):
    # 将数据拆分为高位和低位
    data1_high = (data1 >> 8) & 0xFF
    data1_low = data1 & 0xFF
    data2_high = (data2 >> 8) & 0xFF
    data2_low = data2 & 0xFF

    # 构建数据包
    packet = [
        FRAME_HEADER,
        mode,
        data1_high,
        data1_low,
        data2_high,
        data2_low,
        color,
        0x00,  # BCC校验位，先填充为0
        FRAME_FOOTER
    ]

    # 计算BCC校验位
    packet[7] = calculate_bcc(packet[:7])

    # 发送数据包
    ser.write(bytes(packet))

def qr_code_mode(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    barcodes = pyzbar.decode(gray)

    for barcode in barcodes:
        # 提取二维码的边界框
        (x, y, w, h) = barcode.rect
        
        # 计算二维码中心点
        center_x = x + w // 2
        center_y = y + h // 2
        
        # 计算偏移量
        offset_x, offset_y = calculate_offset(center_x, center_y)
        
        if DEBUG_MODE:
            # 在图像上绘制二维码边界框
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            
            # 绘制中心点
            cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
        
        # 解码二维码数据
        barcode_data = barcode.data.decode("utf-8")
        barcode_type = barcode.type
        
        if DEBUG_MODE:
            # 在图像上显示二维码类型和数据
            text = f"{barcode_type}: {barcode_data}"
            cv2.putText(frame, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # 显示偏移量信息
            offset_text = f"Offset: ({offset_x:+d}, {offset_y:+d})"
            cv2.putText(frame, offset_text, (x + w + 10, y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
        
        # 假设二维码内容是 "123+321" 这样的格式
        if '+' in barcode_data:
            parts = barcode_data.split('+')
            if len(parts) == 2:
                try:
                    data1 = int(parts[0])  # 提取第一个数据
                    data2 = int(parts[1])  # 提取第二个数据
                    
                    if DEBUG_MODE:
                        # 显示分离后的数据
                        cv2.putText(frame, f"QR Data: {data1}+{data2}", (x, y + h + 20), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                    
                    print(f"二维码 - 中心偏移量(x, y): ({offset_x:+d}, {offset_y:+d}), 数据: {data1}+{data2}")
                    return data1, data2, 0x00  # 返回二维码数据，颜色位为0x00
                except ValueError:
                    print("二维码内容格式错误，无法转换为整数")
                    if DEBUG_MODE:
                        cv2.putText(frame, "Format Error", (x, y + h + 20), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        else:
            print("二维码内容格式错误，未包含 '+' 分隔符")
            if DEBUG_MODE:
                cv2.putText(frame, "Format Error: Missing '+'", (x, y + h + 20), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    return 0, 0, 0x00  # 未检测到二维码或格式错误

def high_saturation_mode(frame, target_color=0x00):
        # 将BGR图像转换为HSV图像
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 提取饱和度通道
        saturation = hsv[:, :, 1]

        # 设置高饱和度的阈值
        high_saturation_mask = cv2.threshold(saturation, SATURATION_THRESHOLD, 255, cv2.THRESH_BINARY)[1]

        # 创建圆形区域掩膜：以画面中心为原点，画面尺寸更短的一个边的长度为直径的圆
        frame_height, frame_width = frame.shape[:2]
        center_x, center_y = frame_width // 2, frame_height // 2
        # 圆的半径 = 较短边长度的一半
        radius = min(frame_width, frame_height) // 2
        
        # 创建圆形掩膜
        circle_mask = np.zeros((frame_height, frame_width), dtype=np.uint8)
        cv2.circle(circle_mask, (center_x, center_y), radius, 255, -1)
        
        # 将高饱和度掩膜与圆形掩膜结合：只保留圆形区域内的高饱和度像素
        high_saturation_mask = cv2.bitwise_and(high_saturation_mask, circle_mask)

        # 形态学操作优化掩膜
        kernel = np.ones((5, 5), np.uint8)
        cleaned_mask = cv2.morphologyEx(high_saturation_mask, cv2.MORPH_OPEN, kernel)

        # DEBUG模式下显示掩膜图像和圆形处理区域
        if DEBUG_MODE:
            # 在原始画面上绘制圆形处理区域边界（仅用于可视化）
            cv2.circle(frame, (center_x, center_y), radius, (255, 0, 255), 2)  # 紫色圆圈表示处理区域
            
            # 显示圆形掩膜和处理后的掩膜
            cv2.imshow("Circle Mask", circle_mask)
            cv2.imshow("Saturation Mask - Original", high_saturation_mask)
            cv2.imshow("Saturation Mask - Cleaned", cleaned_mask)

        # 查找轮廓
        contours, _ = cv2.findContours(cleaned_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 遍历所有轮廓
        for contour in contours:
            # 过滤掉过小的区域
            area = cv2.contourArea(contour)
            if area < AREA_THRESHOLD:
                continue

            # 计算轮廓的外接矩形和中心点
            M = cv2.moments(contour)
            if M["m00"] != 0:  # 防止除以零
                cx = int(M["m10"] / M["m00"])  # 中心点x坐标
                cy = int(M["m01"] / M["m00"])  # 中心点y坐标
            else:
                cx, cy = 0, 0

            # 计算偏移量
            offset_x, offset_y = calculate_offset(cx, cy)

            # 获取外接矩形
            x, y, w, h = cv2.boundingRect(contour)

            # 创建轮廓精确mask，只包含轮廓内部的像素
            contour_mask = np.zeros(frame.shape[:2], dtype=np.uint8)
            cv2.fillPoly(contour_mask, [contour], 255)
            
            # 计算该轮廓内的平均BGR值（只计算轮廓内部的像素）
            mean_bgr = cv2.mean(frame, mask=contour_mask)[:3]  # 提取前三个值 (B, G, R)

            # 转换为RGB顺序
            mean_rgb = (mean_bgr[2], mean_bgr[1], mean_bgr[0])  # 将BGR转为RGB
            mean_rgb = tuple(map(int, mean_rgb))  # 转换为整数

            # 判断RGB三个值中最大的一个对应的颜色
            max_color_index = np.argmax(mean_rgb)  # 找到最大值的索引
            dominant_color = [COLOR_RED, COLOR_GREEN, COLOR_BLUE][max_color_index]  # 对应颜色名称

            # 如果指定了目标颜色，只返回匹配的颜色区域
            if target_color != 0x00 and dominant_color != target_color:
                continue

            # 只有匹配目标颜色的区域才进行DEBUG绘制和标注
            if DEBUG_MODE:
                # 在图像上绘制矩形框
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)  # 绿色矩形框

                # 在图像上绘制中心点
                cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)  # 红色圆点表示中心

                # 在图像上标注面积、主要颜色和偏移量
                cv2.putText(frame, f"Area: {area:.0f}", (x, y - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                cv2.putText(frame, f"Color: {get_color_name(dominant_color)}", (x, y - 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                cv2.putText(frame, f"Center: ({cx}, {cy})", (x, y - 50), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                # 显示偏移量信息
                offset_text = f"Offset: ({offset_x:+d}, {offset_y:+d})"
                cv2.putText(frame, offset_text, (x + w + 10, y + h//2), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

            # 输出偏移量、面积和主要颜色到控制台
            print(f"高饱和度区域 - 中心偏移量(x, y): ({offset_x:+d}, {offset_y:+d}), 区域面积: {area:.0f}, 主要颜色: {get_color_name(dominant_color)}")
            print(f"圆形处理区域 - 中心: ({center_x}, {center_y}), 半径: {radius}px, 范围: {min(frame_width, frame_height)}px直径")
            return offset_x, offset_y, dominant_color

        return 0, 0, 0x00  # 未检测到高饱和度区域

def circle_mode(frame, min_radius=95, max_radius=100):
    # 转换为灰度图像
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # 高斯滤波降噪
    blur = cv2.GaussianBlur(gray, (5, 5), 0)

    # 霍夫圆检测
    circles = cv2.HoughCircles(
        blur, 
        cv2.HOUGH_GRADIENT, 
        dp=1, 
        minDist=50, 
        param1=50, 
        param2=30, 
        minRadius=min_radius, 
        maxRadius=max_radius
    )

    if circles is not None:
        # 将圆的坐标转换为整数
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            center = (i[0], i[1])  # 圆心坐标
            radius = i[2]  # 圆的半径
            
            # 计算偏移量
            offset_x, offset_y = calculate_offset(center[0], center[1])

            if DEBUG_MODE:
                # 在图像上绘制圆和圆心
                cv2.circle(frame, center, radius, (0, 255, 0), 2)  # 绘制圆
                cv2.circle(frame, center, 2, (0, 0, 255), 3)  # 绘制圆心

                # 提取半径（R）值并显示在图像上
                R_value = int(radius)  # 转换为整数以方便显示
                cv2.putText(frame, f"R: {R_value}", (center[0] + radius + 5, center[1]), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1, cv2.LINE_AA)
                
                # 显示圆心坐标
                cv2.putText(frame, f"Center: ({center[0]}, {center[1]})", (center[0] - 70, center[1] - radius - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1, cv2.LINE_AA)
                
                # 显示偏移量信息
                offset_text = f"Offset: ({offset_x:+d}, {offset_y:+d})"
                cv2.putText(frame, offset_text, (center[0] + radius + 5, center[1] + 20), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1, cv2.LINE_AA)
            else:
                # 提取半径值用于打印
                R_value = int(radius)
            
            # 返回偏移量和颜色
            print(f"圆形 - 中心偏移量(x, y): ({offset_x:+d}, {offset_y:+d}), 半径: {R_value}")
            return offset_x, offset_y, 0x00  # 返回偏移量，颜色位为0x00

    return 0, 0, 0x00  # 未检测到圆形

def get_circle_color_mode(frame):
    """
    获取圆形区域的主要颜色信息（基于高饱和度识别逻辑）
    
    参数:
    - frame: 输入图像
    
    返回:
    - 0, 0, 主要颜色 (不返回坐标，只返回主要颜色)
    """
    # 将BGR图像转换为HSV图像
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 提取饱和度通道
    saturation = hsv[:, :, 1]

    # 设置独立的高饱和度阈值（与其他模式独立）
    GET_CIRCLE_COLOR_SATURATION_THRESHOLD = 50
    high_saturation_mask = cv2.threshold(saturation, GET_CIRCLE_COLOR_SATURATION_THRESHOLD, 255, cv2.THRESH_BINARY)[1]

    # 创建圆形区域掩膜：以画面中心为原点，画面尺寸更短的一个边的长度为直径的圆
    frame_height, frame_width = frame.shape[:2]
    center_x, center_y = frame_width // 2, frame_height // 2
    # 圆的半径 = 较短边长度的一半
    radius = min(frame_width, frame_height) // 2
    
    # 创建圆形掩膜
    circle_mask = np.zeros((frame_height, frame_width), dtype=np.uint8)
    cv2.circle(circle_mask, (center_x, center_y), radius, 255, -1)
    
    # 将高饱和度掩膜与圆形掩膜结合：只保留圆形区域内的高饱和度像素
    high_saturation_mask = cv2.bitwise_and(high_saturation_mask, circle_mask)

    # 形态学操作优化掩膜
    kernel = np.ones((5, 5), np.uint8)
    cleaned_mask = cv2.morphologyEx(high_saturation_mask, cv2.MORPH_OPEN, kernel)

    # DEBUG模式下显示掩膜图像和圆形处理区域
    if DEBUG_MODE:
        # 在原始画面上绘制圆形处理区域边界（仅用于可视化）
        cv2.circle(frame, (center_x, center_y), radius, (255, 0, 255), 2)  # 紫色圆圈表示处理区域
        
        # 显示圆形掩膜和处理后的掩膜
        cv2.imshow("Circle Mask", circle_mask)
        cv2.imshow("Saturation Mask - Original", high_saturation_mask)
        cv2.imshow("Saturation Mask - Cleaned", cleaned_mask)

    # 查找轮廓
    contours, _ = cv2.findContours(cleaned_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        return 0, 0, 0x00  # 未检测到轮廓

    # 对所有高饱和度区域进行颜色统计
    total_red_area = 0
    total_green_area = 0
    total_blue_area = 0
    total_area = 0
    valid_contours = []

    # 遍历所有轮廓，降低面积阈值以包含更多小面积区域
    min_area_threshold = 5  # 降低面积阈值，包含小面积区域
    
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < min_area_threshold:  # 过滤掉过小的噪点
            continue
            
        valid_contours.append(contour)
        
        # 创建轮廓精确mask，只包含轮廓内部的像素
        contour_mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        cv2.fillPoly(contour_mask, [contour], 255)
        
        # 计算该轮廓内的平均BGR值
        mean_bgr = cv2.mean(frame, mask=contour_mask)[:3]
        
        # 转换为RGB顺序
        mean_rgb = (mean_bgr[2], mean_bgr[1], mean_bgr[0])
        
        # 判断RGB三个值中最大的一个对应的颜色，并按面积加权统计
        max_color_index = np.argmax(mean_rgb)
        if max_color_index == 0:  # 红色
            total_red_area += area
        elif max_color_index == 1:  # 绿色
            total_green_area += area
        elif max_color_index == 2:  # 蓝色
            total_blue_area += area
            
        total_area += area

    if total_area == 0:
        return 0, 0, 0x00  # 没有有效区域

    # 根据面积加权确定主要颜色
    if total_red_area >= total_green_area and total_red_area >= total_blue_area:
        dominant_color = COLOR_RED
    elif total_green_area >= total_blue_area:
        dominant_color = COLOR_GREEN
    else:
        dominant_color = COLOR_BLUE

    if DEBUG_MODE:
        # 绘制所有有效轮廓
        for contour in valid_contours:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 1)  # 绿色矩形框
            
        # 显示颜色统计信息
        info_y = 30
        cv2.putText(frame, f"Red: {total_red_area:.0f}", (10, info_y), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        cv2.putText(frame, f"Green: {total_green_area:.0f}", (10, info_y + 25), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(frame, f"Blue: {total_blue_area:.0f}", (10, info_y + 50), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
        cv2.putText(frame, f"Main Color: {get_color_name(dominant_color)}", (10, info_y + 80), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    # 输出统计结果
    print(f"获取圆形颜色模式 - 轮廓数量: {len(valid_contours)}, 总面积: {total_area:.0f}")
    print(f"颜色统计 - 红色面积: {total_red_area:.0f}, 绿色面积: {total_green_area:.0f}, 蓝色面积: {total_blue_area:.0f}")
    print(f"主要颜色: {get_color_name(dominant_color)}")
    print(f"圆形处理区域 - 中心: ({center_x}, {center_y}), 半径: {radius}px")
    
    return 0, 0, dominant_color  # 不返回坐标，只返回主要颜色

def uart_listener():
    global current_mode, target_color
    non_header_count = 0  # 连续非帧头字节计数器
    
    while True:
        # 读取一个字节
        byte = ser.read(1)
        if not byte:
            continue  # 如果没有读到数据，继续等待

        # 判断是否是帧头
        if byte[0] == FRAME_HEADER:
            non_header_count = 0  # 找到帧头，重置计数器
            # 读取后面的8个字节
            packet = byte + ser.read(8)
            if len(packet) == 9:
                # 检查帧尾是否正确
                if packet[8] == FRAME_FOOTER:
                    # 计算BCC校验位
                    received_bcc = packet[7]
                    calculated_bcc = calculate_bcc(packet[:7])
                    
                    # 如果BCC校验通过
                    if received_bcc == calculated_bcc:
                        mode = packet[1]
                        current_mode = mode
                        # 获取颜色位
                        target_color = packet[6]
                        print(f"收到指令，切换到模式: {mode}, 目标颜色: {target_color}")
                    else:
                        print("BCC校验失败，丢弃数据包")
                else:
                    print("帧尾错误，丢弃数据包")
            else:
                print("数据包长度错误，丢弃数据包")
        else:
            non_header_count += 1  # 非帧头字节计数
            print(f"丢弃非帧头字节: {byte[0]:02X} (连续第{non_header_count}个)")
            
            # 如果连续检测到超过10个非帧头字节，清空接收缓冲区
            if non_header_count > 10:
                print("连续收到超过10个非帧头字节，清空接收缓冲区重新开始")
                ser.reset_input_buffer()  # 清空输入缓冲区
                non_header_count = 0  # 重置计数器

def get_mode_name(mode):
    """获取模式名称"""
    mode_names = {
        MODE_IDLE: "IDLE",
        MODE_QR_CODE: "QR_CODE", 
        MODE_HIGH_SATURATION: "HIGH_SATURATION",
        MODE_CIRCLE: "CIRCLE",
        MODE_CALI_TURNTABLE: "CALI_TURNTABLE",
        MODE_GET_CIRCLE_COLOR: "GET_CIRCLE_COLOR"
    }
    return mode_names.get(mode, "UNKNOWN")

def get_color_name(color):
    """获取颜色名称"""
    color_names = {
        0x00: "ALL",
        COLOR_RED: "RED",
        COLOR_GREEN: "GREEN", 
        COLOR_BLUE: "BLUE"
    }
    return color_names.get(color, "UNKNOWN")

def update_exposure(cap, new_exposure):
    """更新摄像头曝光设置"""
    global current_exposure
    if not AUTO_EXPOSURE_ENABLED and cap is not None and cap.isOpened():
        print(f"\n调整曝光值: {current_exposure} -> {new_exposure}")
        
        # 直接尝试设置曝光值（不再依赖自动曝光禁用）
        success1 = cap.set(cv2.CAP_PROP_EXPOSURE, new_exposure)
        actual_exp = cap.get(cv2.CAP_PROP_EXPOSURE)
        print(f"  标准曝光: 设置={success1}, 目标={new_exposure}, 实际={actual_exp}")
        
        # 如果曝光设置失败或差距很大，尝试亮度控制
        if not success1 or abs(actual_exp - new_exposure) > 500:
            print("  标准曝光控制无效，尝试亮度控制...")
            
            # 将曝光值映射到亮度范围 (0-100)
            # 较低曝光值 -> 较低亮度
            brightness_val = max(0, min(100, (new_exposure / EXPOSURE_MAX_RANGE) * 100))
            success2 = cap.set(cv2.CAP_PROP_BRIGHTNESS, brightness_val)
            actual_brightness = cap.get(cv2.CAP_PROP_BRIGHTNESS)
            print(f"  亮度控制: 设置={success2}, 目标={brightness_val:.1f}, 实际={actual_brightness}")
        
        # 尝试增益控制作为补充
        # 较低曝光值 -> 较低增益
        gain_val = max(0, min(100, (new_exposure / EXPOSURE_MAX_RANGE) * 50))  # 限制增益在50以内
        success3 = cap.set(cv2.CAP_PROP_GAIN, gain_val)
        actual_gain = cap.get(cv2.CAP_PROP_GAIN)
        print(f"  增益控制: 设置={success3}, 目标={gain_val:.1f}, 实际={actual_gain}")
        
        # 更新全局变量
        current_exposure = new_exposure
        
        print(f"  完成！等待画面更新...")
        return True
    return False

def emergency_darken(cap):
    """紧急降低画面亮度"""
    if cap is not None and cap.isOpened():
        print("\n紧急降低亮度...")
        
        # 尝试设置最低曝光值
        cap.set(cv2.CAP_PROP_EXPOSURE, EXPOSURE_MIN_RANGE)
        print(f"  设置最低曝光: {EXPOSURE_MIN_RANGE}")
        
        # 设置最低亮度
        cap.set(cv2.CAP_PROP_BRIGHTNESS, 0)
        print("  设置最低亮度: 0")
        
        # 设置最低增益
        cap.set(cv2.CAP_PROP_GAIN, 0)
        print("  设置最低增益: 0")
        
        # 检查结果
        actual_exp = cap.get(cv2.CAP_PROP_EXPOSURE)
        actual_brightness = cap.get(cv2.CAP_PROP_BRIGHTNESS)
        actual_gain = cap.get(cv2.CAP_PROP_GAIN)
        
        print(f"  结果 - 曝光:{actual_exp}, 亮度:{actual_brightness}, 增益:{actual_gain}")
        return True
    return False

def update_contrast(cap, new_contrast):
    """更新摄像头对比度设置"""
    global current_contrast
    if cap is not None and cap.isOpened():
        print(f"\n调整对比度: {current_contrast} -> {new_contrast}")
        
        success = cap.set(cv2.CAP_PROP_CONTRAST, new_contrast)
        actual_contrast = cap.get(cv2.CAP_PROP_CONTRAST)
        print(f"  对比度控制: 设置={success}, 目标={new_contrast}, 实际={actual_contrast}")
        
        current_contrast = new_contrast
        print(f"  完成！等待画面更新...")
        return True
    return False

def update_saturation(cap, new_saturation):
    """更新摄像头饱和度设置"""
    global current_saturation
    if cap is not None and cap.isOpened():
        print(f"\n调整饱和度: {current_saturation} -> {new_saturation}")
        
        success = cap.set(cv2.CAP_PROP_SATURATION, new_saturation)
        actual_saturation = cap.get(cv2.CAP_PROP_SATURATION)
        print(f"  饱和度控制: 设置={success}, 目标={new_saturation}, 实际={actual_saturation}")
        
        current_saturation = new_saturation
        print(f"  完成！等待画面更新...")
        return True
    return False

def optimize_image_quality(cap):
    """优化图像质量 - 提高对比度和饱和度"""
    if cap is not None and cap.isOpened():
        print("\n优化图像质量...")
        
        # 提高对比度到70
        optimized_contrast = 70
        success1 = cap.set(cv2.CAP_PROP_CONTRAST, optimized_contrast)
        actual_contrast = cap.get(cv2.CAP_PROP_CONTRAST)
        print(f"  对比度优化: 设置={success1}, 目标={optimized_contrast}, 实际={actual_contrast}")
        
        # 提高饱和度到80
        optimized_saturation = 80
        success2 = cap.set(cv2.CAP_PROP_SATURATION, optimized_saturation)
        actual_saturation = cap.get(cv2.CAP_PROP_SATURATION)
        print(f"  饱和度优化: 设置={success2}, 目标={optimized_saturation}, 实际={actual_saturation}")
        
        # 更新全局变量
        global current_contrast, current_saturation
        current_contrast = optimized_contrast
        current_saturation = optimized_saturation
        
        print("  图像质量优化完成！")
        return True
    return False

def check_camera_properties(cap):
    """检查摄像头支持的属性和当前设置"""
    if cap is None or not cap.isOpened():
        print("摄像头未打开，无法检查属性")
        return
    
    print("\n=== 摄像头属性检查 ===")
    print(f"OpenCV版本: {cv2.__version__}")
    
    # 检查自动曝光支持
    auto_exp = cap.get(cv2.CAP_PROP_AUTO_EXPOSURE)
    print(f"自动曝光模式: {auto_exp}")
    
    # 检查当前曝光值
    current_exp = cap.get(cv2.CAP_PROP_EXPOSURE)
    print(f"当前曝光值: {current_exp}")
    
    # 检查亮度设置
    brightness = cap.get(cv2.CAP_PROP_BRIGHTNESS)
    print(f"亮度: {brightness}")
    
    # 检查对比度
    contrast = cap.get(cv2.CAP_PROP_CONTRAST)
    print(f"对比度: {contrast}")
    
    # 检查增益
    gain = cap.get(cv2.CAP_PROP_GAIN)
    print(f"增益: {gain}")
    
    # 检查饱和度
    saturation = cap.get(cv2.CAP_PROP_SATURATION)
    print(f"饱和度: {saturation}")
    
    # 检查是否支持绝对曝光时间控制（兼容性检查）
    try:
        if hasattr(cv2, 'CAP_PROP_EXPOSURE_ABSOLUTE'):
            exp_time = cap.get(cv2.CAP_PROP_EXPOSURE_ABSOLUTE)
            print(f"绝对曝光时间: {exp_time} μs")
        else:
            print("绝对曝光时间: 不支持 (OpenCV版本较老)")
    except:
        print("绝对曝光时间: 检查失败")
    
    # 检查帧率
    fps = cap.get(cv2.CAP_PROP_FPS)
    print(f"帧率: {fps}")
    
    # 分析自动曝光状态
    if auto_exp == 3.0:
        print("\n警告: 自动曝光可能仍然启用！")
        print("建议: 尝试设置为0.25或1来禁用自动曝光")
    elif auto_exp == 0.25 or auto_exp == 1.0:
        print(f"\n手动曝光模式已启用 (模式: {auto_exp})")
    else:
        print(f"\n自动曝光状态未知: {auto_exp}")
    
    # 如果曝光值过高，给出警告
    if current_exp > 1000:
        print(f"\n警告: 当前曝光值过高 ({current_exp})，可能导致画面过亮！")
        print("建议: 按 D 键紧急降低亮度")
    
    print("=====================\n")

def test_exposure_methods(cap):
    """测试不同的曝光控制方法"""
    if cap is None or not cap.isOpened():
        print("摄像头未打开，无法测试曝光")
        return
    
    print("\n=== 曝光控制方法测试 ===")
    
    # 获取当前曝光值作为参考
    current_exp = cap.get(cv2.CAP_PROP_EXPOSURE)
    print(f"当前曝光值: {current_exp}")
    
    # 测试不同曝光值（基于你的摄像头范围）
    test_values = [1, 50, 100, 500, 1000]  # 从很暗到较亮
    
    for test_val in test_values:
        print(f"\n测试曝光值: {test_val}")
        
        # 测试标准曝光控制
        success = cap.set(cv2.CAP_PROP_EXPOSURE, test_val)
        actual = cap.get(cv2.CAP_PROP_EXPOSURE)
        print(f"  标准方法 - 设置: {success}, 期望: {test_val}, 实际: {actual}")
        
        # 测试亮度调整
        brightness_val = (test_val / EXPOSURE_MAX_RANGE) * 100
        success2 = cap.set(cv2.CAP_PROP_BRIGHTNESS, brightness_val)
        actual2 = cap.get(cv2.CAP_PROP_BRIGHTNESS)
        print(f"  亮度方法 - 设置: {success2}, 期望: {brightness_val:.1f}, 实际: {actual2}")
        
        time.sleep(1.5)  # 延迟观察效果
        print("  (请观察画面亮度变化)")
    
    print("测试完成！请检查哪种方法有效果")
    print("========================\n")

def draw_status_info(frame):
    """在帧上绘制状态信息和控制说明"""
    global frame_processing_time, fps
    
    # 绘制屏幕中心十字标记
    cv2.line(frame, (FRAME_CENTER_X - 10, FRAME_CENTER_Y), (FRAME_CENTER_X + 10, FRAME_CENTER_Y), (255, 255, 255), 1)
    cv2.line(frame, (FRAME_CENTER_X, FRAME_CENTER_Y - 10), (FRAME_CENTER_X, FRAME_CENTER_Y + 10), (255, 255, 255), 1)
    cv2.circle(frame, (FRAME_CENTER_X, FRAME_CENTER_Y), 3, (255, 255, 255), 1)
    
    # 创建半透明背景
    overlay = frame.copy()
    cv2.rectangle(overlay, (10, 10), (580, 290), (0, 0, 0), -1)  # 增加高度以容纳新信息
    cv2.addWeighted(overlay, 0.7, frame, 0.3, 0, frame)
    
    # 显示当前模式和目标颜色
    mode_text = f"Mode: {get_mode_name(current_mode)} ({current_mode})"
    color_text = f"Target Color: {get_color_name(target_color)}"
    exposure_text = f"Exposure: {'Auto' if AUTO_EXPOSURE_ENABLED else f'Manual ({current_exposure})'}"
    contrast_text = f"Contrast: {current_contrast}  Saturation: {current_saturation}"
    
    # 添加帧处理时间和FPS显示
    fps_text = f"FPS: {fps:.1f}  Processing Time: {frame_processing_time:.1f}ms"
    
    cv2.putText(frame, mode_text, (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    cv2.putText(frame, color_text, (15, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    cv2.putText(frame, exposure_text, (15, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 128, 0), 2)
    cv2.putText(frame, contrast_text, (15, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
    cv2.putText(frame, fps_text, (15, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    
    # 显示坐标系说明
    rotation_status = "180° Rotated" if CAMERA_ROTATION_180 else "Normal"
    coord_text = f"Coordinate: Center({FRAME_CENTER_X}, {FRAME_CENTER_Y}) Right=X+, Up=Y+ ({rotation_status})"
    cv2.putText(frame, coord_text, (15, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (128, 128, 128), 1)
    
    # 显示控制说明
    if DEBUG_MODE:
        controls = [
            "Controls (DEBUG Mode):",
            "0-Idle  1-QR Code  2-High Saturation  3-Circle  4-Cali Turntable",
            "R-Red  G-Green  B-Blue  A-All Colors",
            "+ / - : Adjust Exposure  [ / ] : Adjust Contrast  ; / ' : Adjust Saturation",
            "P : Check Properties  E : Test Exposure  Q : Optimize Quality  D : Emergency Darken",
            "ESC-Exit"
        ]
    else:
        controls = [
            "Mode: Running in background",
            "Control via UART commands only",
            "No keyboard input available",
            "",
            "",
            ""
        ]
    
    for i, text in enumerate(controls):
        cv2.putText(frame, text, (15, 170 + i * 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

def camera_capture():
    global latest_frame, frame_lock, camera_object
    cap = None
    
    print("正在初始化摄像头设备0...")
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("无法打开摄像头设备0，程序退出")
        return
    
    # 设置摄像头参数
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # 减少缓冲区大小
    
    # 设置曝光控制
    if AUTO_EXPOSURE_ENABLED:
        # 启用自动曝光
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)  # 自动曝光模式 (0.75 = 自动, 0.25 = 手动)
        print("自动曝光已启用")
    else:
        # 禁用自动曝光，使用手动曝光
        print("尝试禁用自动曝光...")
        auto_exp_modes = [0.25, 1, 0]  # 尝试不同的禁用值
        success_disable = False
        
        for mode in auto_exp_modes:
            success = cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, mode)
            current_mode = cap.get(cv2.CAP_PROP_AUTO_EXPOSURE)
            print(f"  尝试设置自动曝光为{mode}: 成功={success}, 当前值={current_mode}")
            if current_mode != 3.0:  # 如果不是3.0（自动模式），说明成功了
                print(f"  自动曝光已禁用（模式: {current_mode}）")
                success_disable = True
                break
        
        if not success_disable:
            print("  无法禁用自动曝光，将尝试其他控制方法")
        
        # 设置手动曝光值
        cap.set(cv2.CAP_PROP_EXPOSURE, current_exposure)
        actual_exp = cap.get(cv2.CAP_PROP_EXPOSURE)
        print(f"手动曝光设置 - 目标: {current_exposure}, 实际: {actual_exp}")
        
        # 如果曝光值仍然过高，立即尝试降低
        if actual_exp > 1000:
            print(f"检测到曝光值过高 ({actual_exp})，尝试降低...")
            emergency_darken(cap)
            # 重新检查
            final_exp = cap.get(cv2.CAP_PROP_EXPOSURE)
            print(f"降低后曝光值: {final_exp}")
        
        # 设置默认对比度和饱和度
        cap.set(cv2.CAP_PROP_CONTRAST, current_contrast)
        actual_contrast = cap.get(cv2.CAP_PROP_CONTRAST)
        print(f"对比度设置 - 目标: {current_contrast}, 实际: {actual_contrast}")
        
        cap.set(cv2.CAP_PROP_SATURATION, current_saturation)
        actual_saturation = cap.get(cv2.CAP_PROP_SATURATION)
        print(f"饱和度设置 - 目标: {current_saturation}, 实际: {actual_saturation}")

    # 测试读取一帧
    ret, test_frame = cap.read()
    if not ret:
        print("摄像头打开成功但无法读取帧")
        cap.release()
        return
    
    print("摄像头初始化成功")
    
    # 设置全局摄像头对象
    camera_object = cap
    
    # 检查摄像头属性
    check_camera_properties(cap)

    # 预热摄像头，丢弃前几帧
    print("摄像头预热中...")
    for i in range(10):
        ret, frame = cap.read()
        if ret:
            time.sleep(0.1)
    
    print("摄像头就绪，开始捕获...")
    consecutive_failures = 0
    max_failures = 5
    
    while True:
        ret, frame = cap.read()
        if ret:
            consecutive_failures = 0
            with frame_lock:
                latest_frame = frame.copy()
        else:
            consecutive_failures += 1
            print(f"读取帧失败 ({consecutive_failures}/{max_failures})")
            
            if consecutive_failures >= max_failures:
                print("连续读取失败，尝试重新初始化摄像头...")
                cap.release()
                time.sleep(2)
                
                # 重新初始化摄像头
                cap = cv2.VideoCapture(0)
                if cap.isOpened():
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
                    cap.set(cv2.CAP_PROP_FPS, 30)
                    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                    
                    # 重新设置曝光控制
                    if AUTO_EXPOSURE_ENABLED:
                        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)
                        print("重新初始化时自动曝光已启用")
                    else:
                        print("重新初始化时尝试禁用自动曝光...")
                        auto_exp_modes = [0.25, 1, 0]
                        success_disable = False
                        
                        for mode in auto_exp_modes:
                            success = cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, mode)
                            current_mode = cap.get(cv2.CAP_PROP_AUTO_EXPOSURE)
                            print(f"  尝试设置自动曝光为{mode}: 成功={success}, 当前值={current_mode}")
                            if current_mode != 3.0:
                                print(f"  自动曝光已禁用（模式: {current_mode}）")
                                success_disable = True
                                break
                        
                        if not success_disable:
                            print("  无法重新初始化时禁用自动曝光")
                        
                        cap.set(cv2.CAP_PROP_EXPOSURE, current_exposure)
                        actual_exp = cap.get(cv2.CAP_PROP_EXPOSURE)
                        print(f"重新初始化时手动曝光设置 - 目标: {current_exposure}, 实际: {actual_exp}")
                        
                        # 重新设置对比度和饱和度
                        cap.set(cv2.CAP_PROP_CONTRAST, current_contrast)
                        cap.set(cv2.CAP_PROP_SATURATION, current_saturation)
                        print(f"重新初始化时图像质量已恢复")

                    ret, test_frame = cap.read()
                    if ret:
                        print("摄像头重新初始化成功")
                        consecutive_failures = 0
                        # 更新全局摄像头对象
                        camera_object = cap
                    else:
                        print("摄像头重新初始化失败，程序退出")
                        cap.release()
                        return
                else:
                    print("重新初始化摄像头失败，程序退出")
                    return
        
        time.sleep(0.01)  # 小延迟避免CPU过度使用

def camera_processor():
    global latest_frame, frame_lock, target_color, current_mode
    global frame_processing_time, fps, last_fps_time, frame_count
    
    while True:
        # 记录帧处理开始时间
        frame_start_time = time.time()
        
        with frame_lock:
            if latest_frame is not None:
                frame = latest_frame.copy()
            else:
                continue

        # 根据当前模式处理帧
        if current_mode == MODE_QR_CODE:
            data1, data2, color = qr_code_mode(frame)
            send_data_packet(current_mode, data1, data2, color)
        elif current_mode == MODE_HIGH_SATURATION:
            x, y, color = high_saturation_mode(frame, target_color)
            send_data_packet(current_mode, x, y, color)
        elif current_mode == MODE_CIRCLE:
            x, y, color = circle_mode(frame, min_radius=95, max_radius=100)
            send_data_packet(current_mode, x, y, color)
        elif current_mode == MODE_CALI_TURNTABLE:
            x, y, color = circle_mode(frame, min_radius=200, max_radius=300)
            send_data_packet(current_mode, x, y, color)
        elif current_mode == MODE_GET_CIRCLE_COLOR:
            x, y, color = get_circle_color_mode(frame)
            send_data_packet(current_mode, x, y, color)
        elif current_mode == MODE_IDLE:
            send_data_packet(current_mode, 0, 0, 0x00)

        # 计算帧处理时间
        frame_end_time = time.time()
        frame_processing_time = (frame_end_time - frame_start_time) * 1000  # 转换为毫秒
        
        # 更新FPS计算
        frame_count += 1
        current_time = time.time()
        if current_time - last_fps_time >= 1.0:  # 每秒更新一次FPS
            fps = frame_count / (current_time - last_fps_time)
            frame_count = 0
            last_fps_time = current_time

        if DEBUG_MODE:
            # 绘制状态信息
            draw_status_info(frame)

            # 显示处理后的帧
            cv2.imshow("Camera Feed", frame)
            
            # 键盘控制
            key = cv2.waitKey(1) & 0xFF
        else:
            # 非DEBUG模式下仍需要检查退出条件，但不显示界面
            key = 0xFF  # 默认无按键
        
        if DEBUG_MODE:
            if key == 27:  # ESC键退出
                break
            elif key == ord('0'):  # 切换到空闲模式
                current_mode = MODE_IDLE
                print(f"切换到模式: {get_mode_name(current_mode)}")
            elif key == ord('1'):  # 切换到二维码模式
                current_mode = MODE_QR_CODE
                print(f"切换到模式: {get_mode_name(current_mode)}")
            elif key == ord('2'):  # 切换到高饱和度模式
                current_mode = MODE_HIGH_SATURATION
                print(f"切换到模式: {get_mode_name(current_mode)}")
            elif key == ord('3'):  # 切换到圆形模式
                current_mode = MODE_CIRCLE
                print(f"切换到模式: {get_mode_name(current_mode)}")
            elif key == ord('4'):  # 切换到转台校准模式
                current_mode = MODE_CALI_TURNTABLE
                print(f"切换到模式: {get_mode_name(current_mode)}")
            elif key == ord('5'):  # 切换到同心圆检测模式
                current_mode = MODE_GET_CIRCLE_COLOR
                print(f"切换到模式: {get_mode_name(current_mode)}")
            elif key == ord('r') or key == ord('R'):  # 设置目标颜色为红色
                target_color = COLOR_RED
                print(f"设置目标颜色: {get_color_name(target_color)}")
            elif key == ord('g') or key == ord('G'):  # 设置目标颜色为绿色
                target_color = COLOR_GREEN
                print(f"设置目标颜色: {get_color_name(target_color)}")
            elif key == ord('b') or key == ord('B'):  # 设置目标颜色为蓝色
                target_color = COLOR_BLUE
                print(f"设置目标颜色: {get_color_name(target_color)}")
            elif key == ord('a') or key == ord('A'):  # 设置目标颜色为全部
                target_color = 0x00
                print(f"设置目标颜色: {get_color_name(target_color)}")
            elif key == ord('+') or key == ord('='):  # 增加曝光时间 (变亮)
                if not AUTO_EXPOSURE_ENABLED:
                    new_exposure = current_exposure + EXPOSURE_ADJUSTMENT_STEP
                    if new_exposure <= EXPOSURE_MAX_RANGE:  # 限制最大曝光值
                        update_exposure(camera_object, new_exposure)
                    else:
                        print(f"曝光时间已达到最大值 ({EXPOSURE_MAX_RANGE})")
                else:
                    print("自动曝光模式下无法手动调整")
            elif key == ord('-') or key == ord('_'):  # 减少曝光时间 (变暗)
                if not AUTO_EXPOSURE_ENABLED:
                    new_exposure = current_exposure - EXPOSURE_ADJUSTMENT_STEP
                    if new_exposure >= EXPOSURE_MIN_RANGE:  # 限制最小曝光值
                        update_exposure(camera_object, new_exposure)
                    else:
                        print(f"曝光时间已达到最小值 ({EXPOSURE_MIN_RANGE})")
                else:
                    print("自动曝光模式下无法手动调整")
            elif key == ord('p') or key == ord('P'):  # 检查摄像头属性
                check_camera_properties(camera_object)
            elif key == ord('e') or key == ord('E'):  # 测试曝光控制方法
                test_exposure_methods(camera_object)
            elif key == ord('d') or key == ord('D'):  # 紧急降低亮度
                emergency_darken(camera_object)
            elif key == ord('q') or key == ord('Q'):  # 优化图像质量
                optimize_image_quality(camera_object)
            elif key == ord('['):  # 降低对比度
                new_contrast = max(0, current_contrast - CONTRAST_ADJUSTMENT_STEP)
                update_contrast(camera_object, new_contrast)
            elif key == ord(']'):  # 增加对比度
                new_contrast = min(100, current_contrast + CONTRAST_ADJUSTMENT_STEP)
                update_contrast(camera_object, new_contrast)
            elif key == ord(';'):  # 降低饱和度
                new_saturation = max(0, current_saturation - SATURATION_ADJUSTMENT_STEP)
                update_saturation(camera_object, new_saturation)
            elif key == ord("'"):  # 增加饱和度
                new_saturation = min(100, current_saturation + SATURATION_ADJUSTMENT_STEP)
                update_saturation(camera_object, new_saturation)

        # 在非DEBUG模式下添加小延迟避免CPU过度使用
        if not DEBUG_MODE:
            time.sleep(0.01)

def main():
    # 显示程序启动信息
    print("EIC Vision 系统启动中...")
    print(f"图像尺寸: {FRAME_WIDTH}x{FRAME_HEIGHT}")
    print(f"屏幕中心: ({FRAME_CENTER_X}, {FRAME_CENTER_Y})")
    print(f"摄像头方向补偿: {'启用 (180度旋转)' if CAMERA_ROTATION_180 else '禁用 (正常方向)'}")
    print(f"曝光设置: {'自动曝光' if AUTO_EXPOSURE_ENABLED else f'手动曝光 (初始值: {MANUAL_EXPOSURE_TIME})'}")
    print(f"DEBUG模式: {'启用' if DEBUG_MODE else '禁用'}")
    print("-" * 50)
    
    if not AUTO_EXPOSURE_ENABLED:
        print("曝光控制提示:")
        print("  - 如果画面过亮，按 D 键紧急降低亮度")
        print("  - 按 + / - 键调整曝光值")
        print("  - 按 [ / ] 键调整对比度，按 ; / ' 键调整饱和度")
        print("  - 按 Q 键一键优化图像质量（推荐用于灰暗画面）")
        print("  - 按 P 键检查摄像头属性")
        print("  - 按 E 键运行曝光测试，找到有效的控制方法")
        print("-" * 50)

    # 启动UART监听线程
    uart_thread = threading.Thread(target=uart_listener)
    uart_thread.daemon = True
    uart_thread.start()

    # 启动摄像头捕获线程
    capture_thread = threading.Thread(target=camera_capture)
    capture_thread.daemon = True
    capture_thread.start()

    # 启动摄像头处理线程
    camera_thread = threading.Thread(target=camera_processor)
    camera_thread.daemon = True
    camera_thread.start()

    # 主线程保持运行
    while True:
        time.sleep(1)

if __name__ == "__main__":
    main()

