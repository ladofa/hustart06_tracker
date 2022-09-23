import rospy
from turtlebot3_msgs.msg import Sound
from turtlebot3_msgs.msg import SensorState
from geometry_msgs.msg import Twist

operation_mode = 0 #0 - 멈춤(동시 누름),     1-동작1(1button)     2-동작2(2button)

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

def process_forward(det):
    op = 0 #0 앞으로 1 물체 매우 근집 2 멀리서 보여서 슬슬 회피
    min_d = 1000000
    min_x = 0
    for x1, y1, x2, y2, conf, cls in det:
        w = abs(x1 - x2)
        d = 10000 / w
        if d < min_d:
            min_d = d
            min_x = (x1 + x2 - 960) / 2
    print('min_d : ', min_d, 'min_x : ', min_x)
    
    #     t = Twist()
    #     t.angular.z = 
    #     pub_motor.publish(t)
    #     pub_motor.publish(Twist())
    # else:
    #     t = Twist()
    #     t.linear.x = 0.05
    #     pub_motor.publish(t)

while True:
    ret, image = cap.read()
    if not ret:
        print('camera break')
        exit()
    
    print('width : ', image.shape[1])

    dst = image
    if operation_mode == 0:
        pass
    elif operation_mode == 1:
        det = yolo_detector.detect(image) # N by .... x1, y1, x2, y2, conf, class
        process_forward(det)
        dst = yolo_detector.draw_boxes(image, det)
    elif operation_mode == 2:
        pass

    # cv2.imshow('window', dst)
    cv2.imwrite('cur.jpg', dst)
    # key = cv2.waitKey(100)
    # if key == ord('q'):
    #     break