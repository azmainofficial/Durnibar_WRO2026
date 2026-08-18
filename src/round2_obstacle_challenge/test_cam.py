import cv2
import time

print("Testing Fifine K420 YUYV capture...")
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
if cap.isOpened():
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 368)
    time.sleep(0.2)
    for i in range(10):
        ret, frame = cap.read()
        print(f"Frame {i}: ret={ret}, shape={frame.shape if ret else None}")
        time.sleep(0.03)
    cap.release()
else:
    print("Could not open /dev/video0")
