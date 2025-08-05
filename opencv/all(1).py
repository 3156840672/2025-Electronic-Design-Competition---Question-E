#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import cv2
import numpy as np
import RPi.GPIO as GPIO
import time


# 閹垫挸绱戦幗鍕剼婢讹拷
cap = cv2.VideoCapture(0)  # 0鐞涖劎銇氭妯款吇閹藉嫬鍎氭径杈剧礉婵″倹鐏夐張澶婎樋娑擃亝鎲氶崓蹇撱仈閸欘垯浜掔亸婵婄槸1,2缁涳拷

#cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)  # 先启用自动曝光
#cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)  # 再切换为手动曝光
#cap.set(cv2.CAP_PROP_EXPOSURE, 40)  # 曝光值

# 鐠佸墽鐤咷PIO濡€崇础娑撶瘔CM缂傛牕褰�
GPIO.setmode(GPIO.BCM)

# 鐎规矮绠烡36A妞瑰崬濮╁Ο鈥虫健娴ｈ法鏁ら惃鍕穿閼达拷
# 濮濄儴绻橀悽鍨簚1閹貉冨煑瀵洝鍓�
ST1_PIN = 17    # STEP娣団€冲娇 (PWM)
DIR1_PIN = 27   # 閺傜懓鎮滈幒褍鍩�
EN1_PIN = 22    # 娴ｈ儻鍏橀幒褍鍩�

# 濮濄儴绻橀悽鍨簚2閹貉冨煑瀵洝鍓�
ST2_PIN = 23    # STEP娣団€冲娇 (PWM)
DIR2_PIN = 24   # 閺傜懓鎮滈幒褍鍩�
EN2_PIN = 25    # 娴ｈ儻鍏橀幒褍鍩�

# 鐠佸墽鐤咷PIO瀵洝鍓煎Ο鈥崇础
GPIO.setwarnings(False)
GPIO.setup(ST1_PIN, GPIO.OUT)
GPIO.setup(DIR1_PIN, GPIO.OUT)
GPIO.setup(EN1_PIN, GPIO.OUT)
GPIO.setup(ST2_PIN, GPIO.OUT)
GPIO.setup(DIR2_PIN, GPIO.OUT)
GPIO.setup(EN2_PIN, GPIO.OUT)

# 閸掓稑缂揚WM鐎电钖�
pwm1 = GPIO.PWM(ST1_PIN, 500)  # 妫版垹宸�500Hz
pwm2 = GPIO.PWM(ST2_PIN, 500)  # 妫版垹宸�500Hz

def setup_motors():
    """閸掓繂顫愰崠鏍暩閺堥缚顔曠純锟�"""
    # 娴ｈ儻鍏橀悽鍨簚 (妤傛ḿ鏁搁獮鍏呭▏閼筹拷)
    GPIO.output(EN1_PIN, GPIO.HIGH)
    GPIO.output(EN2_PIN, GPIO.HIGH)
    
    # 鐠佸墽鐤嗘妯款吇閺傜懓鎮�
    GPIO.output(DIR1_PIN, GPIO.LOW)  # 閺傜懓鎮滈崣顖濐啎缂冾喕璐烲OW閹存湌IGH
    GPIO.output(DIR2_PIN, GPIO.LOW)
    
    # 閸氼垰濮㏄WM
    pwm1.start(50)  # 閸掓繂顫愰崡鐘碘敄濮ｏ拷50%
    pwm2.start(50)

def rotate_motor(motor_num, direction, speed_rpm, steps):
    """
    閹貉冨煑閹稿洤鐣鹃悽鍨簚閺冨娴�
    :param motor_num: 1閹达拷2閿涘矂鈧瀚ㄩ悽鍨簚
    :param direction: 0閹达拷1閿涘本妫嗘潪顒佹煙閸氾拷
    :param speed_rpm: 鏉烆剟鈧拷(RPM)
    :param steps: 濮濄儴绻橀弫锟�
    """
    if motor_num == 1:
        dir_pin = DIR1_PIN
        pwm = pwm1
    elif motor_num == 2:
        dir_pin = DIR2_PIN
        pwm = pwm2
    else:
        print("閺冪姵鏅ラ惃鍕暩閺堣櫣绱崣锟�")
        return
    
    # 鐠佸墽鐤嗛弬鐟版倻
    GPIO.output(dir_pin, direction)
    
    # 鐠侊紕鐣籔WM妫版垹宸� (閸嬪洩顔�200濮濓拷/鏉烇拷)
    steps_per_rev = 200  # 42濮濄儴绻橀悽鍨簚闁艾鐖舵稉锟�200濮濓拷/鏉烇拷
    pwm_freq = (speed_rpm * steps_per_rev) / 60
    pwm.ChangeFrequency(pwm_freq)
    
    # 鏉╂劘顢戦幐鍥х暰濮濄儲鏆�
    print(f"閻㈠灚婧€{motor_num} 閺冨娴嗘稉锟�... 閺傜懓鎮�: {'濮濓絽鎮�' if direction else '閸欏秴鎮�'}, 闁喎瀹�: {speed_rpm}RPM, 濮濄儲鏆�: {steps}")
    time.sleep(steps / (pwm_freq * 2))  # 缁犫偓閸楁洖娆㈤弮鑸靛付閸掕埖顒為弫锟�

def speed_motor(motor_num, direction, speed_rpm,duty):
    """
    閹貉冨煑閹稿洤鐣鹃悽鍨簚閺冨娴�
    :param motor_num: 1閹达拷2閿涘矂鈧瀚ㄩ悽鍨簚
    :param direction: 0閹达拷1閿涘本妫嗘潪顒佹煙閸氾拷
    :param speed_rpm: 鏉烆剟鈧拷(RPM)
    :param steps: 濮濄儴绻橀弫锟�
    """
    if motor_num == 1:
        dir_pin = DIR1_PIN
        pwm = pwm1
    elif motor_num == 2:
        dir_pin = DIR2_PIN
        pwm = pwm2
    else:
        print("閺冪姵鏅ラ惃鍕暩閺堣櫣绱崣锟�")
        return
    
    # 鐠佸墽鐤嗛弬鐟版倻
    GPIO.output(dir_pin, direction)
    
    # 鐠侊紕鐣籔WM妫版垹宸� (閸嬪洩顔�200濮濓拷/鏉烇拷)
    steps_per_rev = 200  # 42濮濄儴绻橀悽鍨簚闁艾鐖舵稉锟�200濮濓拷/鏉烇拷
    pwm_freq = (speed_rpm * steps_per_rev) / 60
    pwm.ChangeFrequency(pwm_freq)
    pwm.ChangeDutyCycle(duty)
    
   

def stop_motors():
    """閸嬫粍顒涢幍鈧張澶屾暩閺堬拷"""
    pwm1.stop()
    pwm2.stop()
    GPIO.output(EN1_PIN, GPIO.LOW)  # 缁備胶鏁ら悽鍨簚
    GPIO.output(EN2_PIN, GPIO.LOW)
    print("閹碘偓閺堝鏁搁張鍝勫嚒閸嬫粍顒�")


# 鐎规矮绠熼拑婵堜紶閼硅尙娈慔SV閼煎啫娲块敍鍫ユ付鐟曚焦鐗撮幑顔肩杽闂勫懏绺洪崗澶愵杹閼硅尪鐨熼弫杈剧礆
#lower_purple = np.array([130, 50, 50])    # 閽冩繄浼犻懝韫瑓闂勬劧绱橦,S,V閿涳拷
#upper_purple = np.array([160, 255, 255])  # 閽冩繄浼犻懝韫瑐闂勶拷

lower_purple = np.array([0, 0, 201])    # 蓝紫色下限（H,S,V閿涳拷
upper_purple = np.array([155, 255, 255])  # 蓝紫色上限

setup_motors()

f_rel_x=0
f_rel_y=0

rel_x_i=0
rel_y_i=0

while True:
    ret, image = cap.read()
    if not ret:
        print("无法获取摄像头图像")
        break
    
    # 转换为HSV颜色空间（更适合颜色检测）
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Canny边缘检测
    v = np.median(gray)
    low_thresh = int(max(0, (1.0 - 0.33) * v))
    high_thresh = int(min(255, (1.0 + 0.33) * v))
    edges = cv2.Canny(gray, low_thresh, high_thresh)
    
    # 执行轮廓检测
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    for contour in contours:
        # 计算面积和周长
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)
        
        # 多边形逼近
        epsilon = 0.02 * perimeter
        approx = cv2.approxPolyDP(contour, epsilon, True)
        
        # 检测四边形
        if len(approx) == 4 and area > 1000 and area < 100000:
            # 长宽比过滤
            x, y, w, h = cv2.boundingRect(approx)
            aspect_ratio = float(w) / h
            if 0.5 < aspect_ratio < 2:
                points = approx.reshape(-1, 2)  # 转换为4x2数组
                center = np.mean(points, axis=0).astype(int)
                cx, cy = center[0], center[1]
                
                # 绘制中心点（黄色实心圆）
                cv2.circle(image, (cx, cy), 2, (0, 255, 255), -1)
                
                # === 新增代码：在四边形区域内检测蓝紫色激光点 ===
                # 1. 创建四边形区域的mask
                mask = np.zeros(image.shape[:2], dtype=np.uint8)
                cv2.fillPoly(mask, [approx], 255)
                
                # 2. 在mask区域内检测蓝紫色
                masked_hsv = cv2.bitwise_and(hsv, hsv, mask=mask)
                purple_mask = cv2.inRange(masked_hsv, lower_purple, upper_purple)
                
                # 3. 寻找蓝紫色区域的轮廓
                purple_contours, _ = cv2.findContours(purple_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
                # 4. 处理检测到的激光点
                for cnt in purple_contours:
                    if cv2.contourArea(cnt) <800 :  # 过滤掉太大的点
                        # 计算最小外接圆
                        (px, py), radius = cv2.minEnclosingCircle(cnt)
                        center = (int(px), int(py))
                        
                        # 绘制激光点（红色圆圈）
                        cv2.circle(image, center, int(radius), (0, 0, 255), 2)
                        
                        # 打印激光点坐标（相对于整个图像）
                        print(f"激光点坐标：({center[0]}, {center[1]})")
                        
                        # 可选：计算相对于四边形中心的坐标
                        rel_x = center[0] - cx
                        rel_y = center[1] - cy
                        print(f"相对于四边形中心的坐标：({rel_x}, {rel_y})")
                        rel_x_i+=rel_x
                        rel_y_i+=rel_y
                        outx=rel_x*5+(rel_x-f_rel_x)*0.5+rel_x_i*0.1
                        outy=rel_y*5+(rel_y-f_rel_y)*0.5+rel_y_i*0.1
                        #outx=rel_x*5
                        #outy=rel_y*5
                        print(f"kd({(rel_x-f_rel_x)*0.5}, {(rel_y-f_rel_y)*0.5})")
                        print(f"out({outx}, {outy})")
                        f_rel_x=rel_x
                        f_rel_y=rel_y
                        if outx >0 :
                            speed_motor(1, GPIO.LOW,outx,50)
                        elif outx ==0 :
                            speed_motor(1, GPIO.LOW,50,100)
                        else:
                            speed_motor(1, GPIO.HIGH,-outx,50)

                        if outy >0 :
                            speed_motor(2, GPIO.LOW,outy,50)
                        elif outy ==0 :
                            speed_motor(1, GPIO.LOW,50,100)
                        else:
                            speed_motor(2, GPIO.HIGH,-outy,50)
                       # time.sleep(0.001)
                
                # 绘制四边形轮廓（绿色）
                cv2.drawContours(image, [approx], -1, (0, 255, 0), 2)
    
    # 閺勫墽銇氱紒鎾寸亯
    cv2.imshow('Rectangle Detection', image)
    
    # 閹革拷'q'闁款噣鈧偓閸戯拷
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 闁插﹥鏂佺挧鍕爱
cap.release()
cv2.destroyAllWindows()