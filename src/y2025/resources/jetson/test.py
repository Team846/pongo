import cv2

print("SCRIPT: ATTEMPTING CAM 1")
cap = cv2.VideoCapture("/dev/v4l/by-path/platform-70090000.xusb-usb-0:3.2:1.0-video-index0") #3.1 - white 3.2 - orange
if cap.isOpened()==False:
    print("SCRIPT: CAM CONN FAILURE")
else:
    print("SCRIPT: CAM CONN")

while (True):
    _, frame = cap.read()
    cv2.imwrite("frame.png", frame)