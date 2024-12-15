import cv2
from ultralytics import YOLO
import numpy as np
import threading
import math

from networktables import NetworkTables
from pref import NumericPref, BooleanPref, KillSwitch

import time


model = YOLO('best.onnx', task="detect")

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 256)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 256)



queue = []

NetworkTables.initialize(server='10.8.46.2')

gpd_table = NetworkTables.getTable('gpd')
preferenceTable = NetworkTables.getTable("RPIPrefs")

kill_switch = KillSwitch(preferenceTable)

gpd_table.putNumberArray('note_x', [])
gpd_table.putNumberArray('note_y', [])

def readCamera():
    while True:
        ret, frame = cap.read()
        if ret:
            queue.append(frame)
        if (len(queue) > 5):
            queue.pop(0)
        
        if cv2.waitKey(1) == ord('q'):
            break



def processFrame():
    while True:
        start = time.perf_counter()
        if len(queue) == 0:
            continue
        frame = queue.pop(0)
        result = model.predict(frame, imgsz=256, conf=0.2, iou=0.5)[0]

        if result is None: continue

        theta_h = []
        theta_v = []
        for box in result.boxes:
            bounding_box = box.xyxy[0]
            x1, y1, x2, y2 = bounding_box
            width = x2 - x1
            height = y2 - y1
            point_width = x1 + width/2
            point_height = y1 + height/2
            theta_h.append((133.8 / 2.0) * (point_width - 0.5*256) / 256)
            theta_v.append(-(133.8 / 2.0) * (point_height - 0.5*256) / 256)
    
        
        gpd_table.putNumberArray('theta_h', theta_h)
        gpd_table.putNumberArray('theta_v', theta_v)


        end = time.perf_counter() - start

        gpd_table.putNumber('latency', end)
        NetworkTables.flush()

if __name__ == "__main__":
    t1 = threading.Thread(target=readCamera)
    t2 = threading.Thread(target=processFrame)


    t1.start()
    t2.start()

    t1.join()
    t2.join()


cap.release()