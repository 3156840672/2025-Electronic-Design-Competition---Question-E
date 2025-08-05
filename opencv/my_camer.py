import cv2
import numpy as np
from step_thread import StepperMotor
import RPi.GPIO as GPIO
import time


# 初始化GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
time.sleep(0.01)
# 打开摄像头
#cap = cv2.VideoCapture(0)
cap = cv2.VideoCapture(0, cv2.CAP_V4L2) 
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # 减少缓冲区延迟
cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)  # 关闭自动对焦（避免运动模糊）
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)  # 先启用自动曝光
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)  # 再切换为手动曝光

#cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3) 
expo=200

#v4l2-ctl -d /dev/video0 --set-ctrl=auto_exposure=3  # 3 = Auto Mode  閼奉亜濮╅弴婵嗗帨

cap.set(cv2.CAP_PROP_EXPOSURE, expo)  # 曝光值

time.sleep(0.01)
# 定义颜色范围
lower_red = np.array([0, 0, 201])
upper_red = np.array([10, 255, 255])

lower_purple = np.array([0, 0, 201])    # 蓝紫色下限（H,S,V閿涳拷
upper_purple = np.array([155, 255, 255])  # 蓝紫色上限

# 初始化步进电机 - 使用BCM引脚编号
motorx = StepperMotor(dir_pin=27, step_pin=17, steps_per_rev=6400)  
motory = StepperMotor(dir_pin=24, step_pin=23, steps_per_rev=6400)  
ENX_PIN = 22 
ENY_PIN = 25    # 娴ｈ儻鍏橀幒褍鍩�
LASER_PIN=21

input_pin = 26
key_up=16
key_down=12

circle =5

GPIO.setup(input_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # 閸氼垳鏁ら崘鍛村劥娑撳﹥濯�

GPIO.setup(circle, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # 閸氼垳鏁ら崘鍛村劥娑撳﹥濯�

GPIO.setup(key_up, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # 閸氼垳鏁ら崘鍛村劥娑撳﹥濯�
GPIO.setup(key_down, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # 閸氼垳鏁ら崘鍛村劥娑撳﹥濯�

GPIO.setup(ENX_PIN, GPIO.OUT)
GPIO.setup(ENY_PIN, GPIO.OUT)
GPIO.setup(LASER_PIN, GPIO.OUT)
        
        # 閸掓繂顫愰悩鑸碘偓锟�
GPIO.output(ENX_PIN, GPIO.HIGH)
GPIO.output(ENY_PIN, GPIO.HIGH)
GPIO.output(LASER_PIN, GPIO.LOW)

# PID閹貉冨煑閸欏倹鏆�
Kp = 0.4
Ki = 0#0.03
Kd = 0.1
kps = 0.13
integral_max = 1000

last_error_x = 0
integral_x = 0
last_error_y = 0
integral_y = 0

count=0

no_find=0

percent=1

time.sleep(0.01)
try:
    while True:
        percent+=1
        if percent>400.0:
            percent=1

        if GPIO.input(key_up)==0:
            expo+=10
        if GPIO.input(key_down)==0:
            expo-=10
        cap.set(cv2.CAP_PROP_EXPOSURE, expo)  # 曝光值

        pin_state = GPIO.input(input_pin)
        #print(f"GPIO{input_pin} 状态: {'高电平' if pin_state else '低电平'}")
        ret, image = cap.read()
        if not ret:
            print("无法获取摄像头图像")
            time.sleep(0.1)
            continue
        
        # 图像处理
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # 边缘检测
        v = np.median(gray)
        low_thresh = int(max(0, (1.0 - 0.33) * v))
        high_thresh = int(min(255, (1.0 + 0.33) * v))
        edges = cv2.Canny(gray, low_thresh, high_thresh)
        
        # 轮廓检测
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            perimeter = cv2.arcLength(contour, True)
            epsilon = 0.02 * perimeter
            approx = cv2.approxPolyDP(contour, epsilon, True)
            
            if len(approx) == 4 and 2000 < area < 80000:
                x, y, w, h = cv2.boundingRect(approx)
                aspect_ratio = float(w) / h
                
                if 0.5 < aspect_ratio < 2:
                    no_find=0

                    points = approx.reshape(-1, 2)
                    center = np.mean(points, axis=0).astype(int)
                    cx, cy = center[0], center[1]
                    
                    # 绘制中心点
                    cv2.circle(image, (cx, cy), 2, (0, 255, 255), -1)

                    # 计算误差
                    #print(f"percent=({((1/percent)*2*np.pi)})")
                    if GPIO.input(circle)==0:
                        errorx = cx - image.shape[1] / 2 - 20
                        errory = cy - image.shape[0] / 2-4
                    else:
                        errorx = cx - image.shape[1] / 2 - 20+(20*np.cos((percent/400)*2*np.pi))
                        errory = cy - image.shape[0] / 2-4+(20*np.sin((percent/400)*2*np.pi))
                    
                    # PID计算
                    Px = Kp * errorx
                    Py = Kp * errory
                
                    integral_x += errorx
                    integral_y += errory
                    integral_x = max(min(integral_x, integral_max), -integral_max)
                    integral_y = max(min(integral_y, integral_max), -integral_max)
                    
                    Ix = Ki * integral_x
                    Iy = Ki * integral_y
                    
                    Dx = Kd * (errorx - last_error_x)
                    Dy = Kd * (errory - last_error_y)
                    
                    pid_out_x = Px + Ix + Dx
                    pid_out_y = Py + Iy + Dy
                    
                    last_error_x = errorx
                    last_error_y = errory
                    
                    # 输出限幅
                    pid_out_x = max(min(pid_out_x, 1000), -1000)
                    pid_out_y = max(min(pid_out_y, 1000), -1000)
                    
                    # 死区控制
                    #if abs(pid_out_x) >= 50:
                    motorx.step(0 if pid_out_x < 0 else 1, int(abs(pid_out_x)), rpm=abs(errorx) * kps + 10)
                    #if abs(pid_out_y) >= 50:
                    motory.step(0 if pid_out_y < 0 else 1, int(abs(pid_out_y)), rpm=abs(errory) * kps + 10)
                    
                    # 激光点检测
                    '''
                    mask = np.zeros(image.shape[:2], dtype=np.uint8)
                    cv2.fillPoly(mask, [approx], 255)
                    masked_hsv = cv2.bitwise_and(hsv, hsv, mask=mask)
                    red_mask = cv2.inRange(masked_hsv, lower_red, upper_red)
                    red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    '''
                    '''for cnt in red_contours:
                        if cv2.contourArea(cnt) < 200:
                            (px, py), radius = cv2.minEnclosingCircle(cnt)
                            center = (int(px), int(py))
                            if x < center[0] < x+w and y < center[1] < y+h:
                                print(f"濠碘偓閸忓鍋ｉ崸鎰垼閿涳拷({center[0]}, {center[1]})")
                                rel_x = center[0] - cx
                                rel_y = center[1] - cy
                                print(f"閻╃ǹ顕崸鎰垼閿涳拷({rel_x}, {rel_y})")
                                cv2.circle(image, center, 2, (0, 0, 255), 2)'''
                    print(f"errorx=({errorx})")
                    print(f"errorx=({errory})")
                    if pin_state ==0:
                        if abs(errorx)<5 and abs(errory)<5:
                            count+=1
                            if count>8:
                                GPIO.output(LASER_PIN, GPIO.HIGH)
                        else:
                            GPIO.output(LASER_PIN, GPIO.LOW)
                            count=0

                    else :
                        GPIO.output(LASER_PIN, GPIO.HIGH)
                    cv2.drawContours(image, [approx], -1, (0, 255, 0), 2)
                
        #print(f"no_find=({no_find})")    
        no_find+=1
        if no_find>20:
            print("找不到矩形")
            motorx.step(1, 200, rpm=100)
            no_find=0
                   

                
        # 显示图像（可选）
        cv2.putText(image, f"Exposure: {expo}", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow('Tracking', image)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
           break

except KeyboardInterrupt:
    print("程序被用户中断")
except Exception as e:
    print(f"发生错误: {str(e)}")
finally:
    cap.release()
    # cv2.destroyAllWindows()
    motorx.cleanup()
    motory.cleanup()
    GPIO.cleanup()

