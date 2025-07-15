import cv2 # type: ignore
import numpy as np # type: ignore
import serial # type: ignore
from pyzbar import pyzbar # type: ignore
import threading
import time

# UART串口配置
ser = serial.Serial('/dev/serial0', 115200, timeout=1)

# 定义模式常量
MODE_STANDBY = 0x00
MODE_QR_CODE = 0x01
MODE_CIRCLE_COLOR = 0x02  # 原料区定位
MODE_COLOR_RING = 0x03

# 定义颜色常量
COLOR_RED = 0x01
COLOR_GREEN = 0x02
COLOR_BLUE = 0x04

# 定义帧头和帧尾
FRAME_HEADER = 0x7B
FRAME_FOOTER = 0x7D

# 全局变量，用于存储当前模式和摄像头帧
current_mode = MODE_STANDBY
latest_frame = None
frame_lock = threading.Lock()
target_color = 0x00  # 添加全局变量存储目标颜色

def calculate_bcc(data):
    bcc = 0
    for byte in data:
        bcc ^= byte
    return bcc

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
        barcode_data = barcode.data.decode("utf-8")
        # 假设二维码内容是 "123+321" 这样的格式
        if '+' in barcode_data:
            parts = barcode_data.split('+')
            if len(parts) == 2:
                try:
                    data1 = int(parts[0])  # 提取第一个数据
                    data2 = int(parts[1])  # 提取第二个数据
                    return data1, data2, 0x00  # 返回两个数据，颜色位为0x00
                except ValueError:
                    print("二维码内容格式错误，无法转换为整数")
        else:
            print("二维码内容格式错误，未包含 '+' 分隔符")

    return 0, 0, 0x00  # 未检测到二维码或格式错误

def circle_color_mode(frame, target_color=0x00):
        # 将BGR图像转换为HSV图像
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 提取饱和度通道
        saturation = hsv[:, :, 1]

        # 设置高饱和度的阈值
        saturation_threshold = 100  # 根据实际需求调整
        high_saturation_mask = cv2.threshold(saturation, saturation_threshold, 255, cv2.THRESH_BINARY)[1]

        # 形态学操作优化掩膜
        kernel = np.ones((5, 5), np.uint8)
        cleaned_mask = cv2.morphologyEx(high_saturation_mask, cv2.MORPH_OPEN, kernel)

        # 查找轮廓
        contours, _ = cv2.findContours(cleaned_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 遍历所有轮廓
        for contour in contours:
            # 过滤掉过小的区域
            area = cv2.contourArea(contour)
            if area < 10000:  # 面积阈值改为10000
                continue

            # 计算轮廓的外接矩形和中心点
            M = cv2.moments(contour)
            if M["m00"] != 0:  # 防止除以零
                cx = int(M["m10"] / M["m00"])  # 中心点x坐标
                cy = int(M["m01"] / M["m00"])  # 中心点y坐标
            else:
                cx, cy = 0, 0

            # 获取外接矩形
            x, y, w, h = cv2.boundingRect(contour)

            # 在图像上绘制矩形框
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)  # 绿色矩形框

            # 在图像上绘制中心点
            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)  # 红色圆点表示中心

            # 计算该区域的平均BGR值
            mask_roi = cleaned_mask[y:y+h, x:x+w]  # 提取区域掩膜
            mean_bgr = cv2.mean(frame[y:y+h, x:x+w], mask=mask_roi)[:3]  # 提取前三个值 (B, G, R)

            # 转换为RGB顺序
            mean_rgb = (mean_bgr[2], mean_bgr[1], mean_bgr[0])  # 将BGR转为RGB
            mean_rgb = tuple(map(int, mean_rgb))  # 转换为整数

            # 判断RGB三个值中最大的一个对应的颜色
            max_color_index = np.argmax(mean_rgb)  # 找到最大值的索引
            dominant_color = [COLOR_RED, COLOR_GREEN, COLOR_BLUE][max_color_index]  # 对应颜色名称

            # 如果指定了目标颜色，只返回匹配的颜色区域
            if target_color != 0x00 and dominant_color != target_color:
                continue

            # 在图像上标注面积和主要颜色
            cv2.putText(frame, f"Area: {area:.0f}", (x, y - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            cv2.putText(frame, f"Color: {dominant_color}", (x, y - 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            # 输出中心坐标、面积和主要颜色到控制台
            print(f"中心坐标(x, y): ({cx}, {cy}), 区域面积: {area:.0f}, 主要颜色: {dominant_color}")
            return cx, cy, dominant_color

        return 0, 0, 0x00  # 未检测到圆

def color_ring_mode(frame):
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
        minRadius=95, 
        maxRadius=100
    )

    if circles is not None:
        # 将圆的坐标转换为整数
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            center = (i[0], i[1])  # 圆心坐标
            radius = i[2]  # 圆的半径

            # 在图像上绘制圆和圆心
            cv2.circle(frame, center, radius, (0, 255, 0), 2)  # 绘制圆
            cv2.circle(frame, center, 2, (0, 0, 255), 3)  # 绘制圆心

           # 提取半径（R）值并显示在图像上
            #R_value = int(radius)  # 转换为整数以方便显示
            #text = f"R: {R_value}"
            #cv2.putText(frame, text, (center[0] + radius + 5, center[1]), cv2.FONT_HERSHEY_SIMPLEX, 
            #            0.5, (255, 0, 0), 1, cv2.LINE_AA)
            
            # 返回圆心坐标和颜色
            return center[0], center[1], 0x00  # 返回圆心位置，颜色位为0x00

    return 0, 0, 0x00  # 未检测到圆

def uart_listener():
    global current_mode, target_color
    while True:
        # 读取一个字节
        byte = ser.read(1)
        if not byte:
            continue  # 如果没有读到数据，继续等待

        # 判断是否是帧头
        if byte[0] == FRAME_HEADER:
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
            print(f"丢弃非帧头字节: {byte[0]:02X}")

def camera_capture():
    global latest_frame, frame_lock
    cap = None
    
    print("正在初始化摄像头设备0...")
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("无法打开摄像头设备0，程序退出")
        return
    
    # 设置摄像头参数
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # 减少缓冲区大小
    
    # 测试读取一帧
    ret, test_frame = cap.read()
    if not ret:
        print("摄像头打开成功但无法读取帧")
        cap.release()
        return
    
    print("摄像头初始化成功")
    
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
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                    cap.set(cv2.CAP_PROP_FPS, 30)
                    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                    
                    ret, test_frame = cap.read()
                    if ret:
                        print("摄像头重新初始化成功")
                        consecutive_failures = 0
                    else:
                        print("摄像头重新初始化失败，程序退出")
                        cap.release()
                        return
                else:
                    print("重新初始化摄像头失败，程序退出")
                    return
        
        time.sleep(0.01)  # 小延迟避免CPU过度使用

def camera_processor():
    global latest_frame, frame_lock, target_color
    while True:
        with frame_lock:
            if latest_frame is not None:
                frame = latest_frame.copy()
            else:
                continue

        # 根据当前模式处理帧
        if current_mode == MODE_QR_CODE:
            data1, data2, color = qr_code_mode(frame)
            send_data_packet(current_mode, data1, data2, color)
        elif current_mode == MODE_CIRCLE_COLOR:
            x, y, color = circle_color_mode(frame, target_color)
            send_data_packet(current_mode, x, y, color)
        elif current_mode == MODE_COLOR_RING:
            x, y, color = color_ring_mode(frame)
            send_data_packet(current_mode, x, y, color)
        elif current_mode == MODE_STANDBY:
            send_data_packet(current_mode, 0, 0, 0x00)

        # 显示处理后的帧
        cv2.imshow("Camera Feed", frame)
        if cv2.waitKey(1) == 27:
            break

def main():
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

