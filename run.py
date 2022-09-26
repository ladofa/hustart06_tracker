import rospy
from turtlebot3_msgs.msg import Sound
from turtlebot3_msgs.msg import SensorState
from geometry_msgs.msg import Twist
import numpy as np
import math

operation_mode = 2 #0 - 멈춤(동시 누름),     1-동작1(1button)     2-동작2(2button)

prev_button = 0
prev_button_sim = 0 #동시에 버튼을 눌렀을 경우 1
def state_callback(data):
    global operation_mode
    global prev_button
    global prev_button_sim######################################################################
    
    #투버튼 상승엣지
    if prev_button_sim == 0 and data.button == 3:
        operation_mode = 0
        prev_button_sim = 1
        print('button sim')

        pub_motor.publish(Twist())#완전 멈춤
        sound = Sound()
        sound.value = 3
        pub_sound.publish(sound)

    #1버튼 하강엣지
    if prev_button == 1 and data.button == 0 and prev_button_sim == 0:
        operation_mode = 1
        print('button 1')
        sound = Sound()
        sound.value = 1
        pub_sound.publish(sound)

    #2버튼 하강엣지
    if prev_button == 2 and data.button == 0 and prev_button_sim == 0:
        operation_mode = 2
        print('button 2')
        sound = Sound()
        sound.value = 2
        pub_sound.publish(sound)

    if prev_button_sim == 1 and data.button == 0:
        prev_button_sim = 0
    prev_button = data.button



rospy.init_node('tracker')
pub_motor = rospy.Publisher('cmd_vel', Twist, queue_size=10)
pub_sound = rospy.Publisher('sound', Sound, queue_size=10)
sub_state = rospy.Subscriber('sensor_state', SensorState, state_callback)

print('start')


import cv2
import yolo_detector


def gstreamer_pipeline(
    sensor_id=0,
    capture_width=960,
    capture_height=540,
    display_width=960,
    display_height=540,
    framerate=10,
    flip_method=0,
):
    return (
        "nvarguscamerasrc sensor-id=%d !"
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink max-buffers=1 drop=True"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )



cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
if not cap.isOpened():
    print("camera failed")
    exit()

def calc_geo(x1, y1, x2, y2):
    w = abs(x2 - x1)
    cx = (x1 + x2) / 2
    x = cx - (960 / 2)
    D = 20100 / w
    if y2 > 520:
        D = 10
    f = 787.6
    X = D * x / f
    return D, X

def process_forward(det):
    op = 0 #0 아무것도 없음 1 직진 2 왼쪽으로 커브 3 장애물 회피
    min_D = 1000000
    min_X = 0
    list_lane = []
    for x1, y1, x2, y2, conf, cls in det:
        if cls == 0 or cls == 1:
            #???
            D, X = calc_geo(x1, y1, x2, y2)
            if D < min_D:
                min_D = D
                min_X = X
        elif cls == 2:
            cx = (x2 + x1) / 2 - (960 / 2)
            list_lane.append((y2, cx))

            
    if min_D < 15:
        op = 3
        # print(op)
    # elif min_D < 50:
    #     if abs(min_X) < 15:
    #         op = 2
    # else:
    #     op = 0
    else:
        list_lane.sort()
        if len(list_lane) == 0:
            op = 0
            cx = 0
            theta = 0
        else:
            cx = list_lane[-1][1] #lane의 위치
            if len(list_lane) == 1:
                theta = 0
            else:
                dy = list_lane[-1][0] - list_lane[-2][0]
                dx = list_lane[-2][1] - list_lane[-1][1]
                theta = math.atan2(dx, dy) / math.pi * 180 #lane의 각도
            
            if abs(cx) < 30 and abs(theta) < 20:
                op = 1
            else:
                op = 2
        # print(op, cx, theta)
    
    # t = Twist()
    # if op == 0:
    #     t.linear.x = 0.15
    # elif op == 1:
    #     t.angular.z = 2.0 #??왼쪽 맞음?
    # elif op == 2:
    #     t.linear.x = 0.1
    #     t.angular.z = 1.0  #??왼쪽?
    # pub_motor.publish(t)

s = Sound()
s.value = 0
pub_sound.publish(s)

while True:
    ret, image = cap.read()
    if not ret:
        print('camera break')
        break
    
    dst = image
    if operation_mode == 0:
        pub_motor.publish(Twist())
        break
    elif operation_mode == 1:
        det = yolo_detector.detect(image) # N by .... x1, y1, x2, y2, conf, class
        process_forward(det)
        dst = yolo_detector.draw_boxes(image, det)
    elif operation_mode == 2:
        pub_motor.publish(Twist())

    # cv2.imshow('window', dst)
    cv2.imwrite('cur.jpg', dst)
    # key = cv2.waitKey(100)
    # if key == ord('q'):
    #     break

print('END.')