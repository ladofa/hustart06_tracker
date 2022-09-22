import rospy
from turtlebot3_msgs.msg import Sound
from turtlebot3_msgs.msg import SensorState


prev_button = 0
def state_callback(data):
    global prev_button
    if prev_button == 1 and data.button == 0:
        print('button off')
        sound = Sound()
        sound.value = 1
        pub_sound.publish(sound)

    prev_button = data.button

rospy.init_node('tracker')
pub_sound = rospy.Publisher('sound', Sound, queue_size=10)
sub_state = rospy.Subscriber('sensor_state', SensorState, state_callback)

print('start')
import cv2
import numpy as np

dummy = np.zeros((100, 100, 3), np.uint8)
while True:
    cv2.imshow('window', dummy)
    key = cv2.waitKey(30)
    if key == ord('q'):
        break