from datetime import datetime
import time
import cv2

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

i = 0

while True:
    ret, image = cap.read()
    if not ret:
        print('camera erorr')
    time_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = "saved/%.5d_%s.jpg" % (i, time_str)
    i += 1
    cv2.imwrite(filename, image)
    print('write : ', filename)
    time.sleep(2)
print('done.')
