from pupil_apriltags import Detector
import numpy as np
import time
import cv2
import threading
from networktables import NetworkTables
import subprocess
from pref import NumericPref, BooleanPref, KillSwitch

NetworkTables.initialize(server='10.8.46.2')
table = NetworkTables.getTable("AprilTags")

preferenceTable = NetworkTables.getTable("JetsonPreferences")

kill_switch = KillSwitch(preferenceTable)

exposure_pref = NumericPref(preferenceTable, "camera_exposure", 1)
horizontal_fov_pref = NumericPref(preferenceTable, "h_fov", 63.1)
vertical_fov_pref = NumericPref(preferenceTable, "v_fov", 50.0)

h_frame_size = NumericPref(preferenceTable, "h_frame", 640)
v_frame_size = NumericPref(preferenceTable, "v_frame", 480)

cam_latency = NumericPref(preferenceTable, "camera_latency", 0.01)

target_mean_brightness = NumericPref(preferenceTable, "target_mean_brightness", 120)

clip_limit = NumericPref(preferenceTable, "clip_limit", 2.0)

debug_mode = BooleanPref(preferenceTable, "debug_mode", False)

at_detector = Detector(families='tag36h11',
                       nthreads=1,
                       quad_decimate=1.0,
                       quad_sigma=0.2,
                       refine_edges=1,
                       decode_sharpening=0.5,
                       debug=0)

cameraMatrix = np.array([(336.7755634193813, 0.0, 333.3575643300718), (0.0, 336.02729840829176, 212.77376312080065), (0.0, 0.0, 1.0)])

camera_params = ( cameraMatrix[0,0], cameraMatrix[1,1], cameraMatrix[0,2], cameraMatrix[1,2] )

queue = []

def adjust_gamma(image, gamma=1.0):
	invGamma = 1.0 / gamma
	table = np.array([((i / 255.0) ** invGamma) * 255
		for i in np.arange(0, 256)]).astype("uint8")
	return cv2.LUT(image, table)

def pre_process(frame):
    global clip_limit
    global target_mean_brightness
    
    clahe = cv2.createCLAHE(clipLimit=clip_limit.get(), tileGridSize=(8,8))

    tmean_b = target_mean_brightness.get()
    result = clahe.apply(frame)

    mean_brightness = np.mean(result)

    if (mean_brightness != 0): result = adjust_gamma(result, tmean_b / mean_brightness)

    return result

def process_frames():
    global table
    global queue

    global cam_latency
    global horizontal_fov_pref
    global vertical_fov_pref
    global h_frame_size
    global v_frame_size

    global debug_mode

    validTagIds = [4, 7]

    CAM_FOV_H = horizontal_fov_pref.get()
    CAM_FOV_V = vertical_fov_pref.get()

    CAM_SZ_H = h_frame_size.get()
    CAM_SZ_V = v_frame_size.get()

    while True:
        if len(queue) == 0: continue

        frame = queue.pop(0)

        process_start_time = time.time()

        orig_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        img = pre_process(orig_img)

        tags = at_detector.detect(img, False, camera_params, 0.065)

        table.putNumber("tx", 0.0)
        table.putNumber("ty", 0.0)
        table.putNumber("tid", -1)

        try:
            for tag in tags:
                if not (tag.tag_id in validTagIds): continue

                cx, cy = tag.center

                tx = (cx - CAM_SZ_H/2) * (CAM_FOV_H / CAM_SZ_H)
                ty = -((cy - CAM_SZ_V/2) * (CAM_FOV_V / CAM_SZ_V))

                table.putNumber("tid", tag.tag_id)
                table.putNumber("tx", tx)

                table.putNumber("ty", ty)

        except:
            pass

        latency = cam_latency.get() + time.time() - process_start_time


        if (debug_mode.get()): 
            cv2.putText(orig_img, f"Original Image: ({CAM_SZ_H}, {CAM_SZ_V}).", 
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 3, cv2.LINE_AA)
            cv2.imwrite("/home/nvidia/apriltags/orig.png", img)

            img_copy = img.copy()

            cv2.putText(img_copy, f"Processed Image: ({CAM_SZ_H}, {CAM_SZ_V}).", 
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 3, cv2.LINE_AA)
            cv2.imwrite("/home/nvidia/apriltags/proc.png", img_copy)

            for tag in tags:
                for idx in range(len(tag.corners)):
                    cv2.line(img, tuple(tag.corners[idx-1, :].astype(int)), 
                            tuple(tag.corners[idx, :].astype(int)), (0, 255, 0), thickness=4)

                    cv2.putText(img, f"ID: {tag.tag_id}", 
                                org=(tag.corners[0, 0].astype(int), tag.corners[0, 1].astype(int)), 
                                fontFace=cv2.FONT_HERSHEY_SIMPLEX, 
                                fontScale=0.8, color=(0, 255, 0), thickness=4)

            cv2.putText(img, f"Resultant Image. {len(tags)} Detections. Latency: {latency}.", 
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 3, cv2.LINE_AA)
            
            cv2.imwrite("/home/nvidia/apriltags/res.png", img)

        table.putNumber("latency", latency)
        NetworkTables.flush()

def getCamera() -> cv2.VideoCapture:
    print("\n")
    cap = cv2.VideoCapture("/dev/video0")
    if cap.isOpened() == False:
        print("CAM 1 FAILURE, ATTEMPTING CAM 2.")
        cap = cv2.VideoCapture("/dev/video1")
    if cap.isOpened() == False:
        print("CAM CONN FAILURE.")
    else:
        print("CAM CONN SUCCESS.")
    print("\n")

    return cap

if __name__ == "__main__":
    cap = getCamera()

    if cap is None:
        print("Exiting, camera is None.")
        exit(1)

    subprocess.run(['v4l2-ctl', f'-c exposure_absolute={exposure_pref.get()}'])
    

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(h_frame_size.get()))
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(v_frame_size.get()))
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 0)

    if cap.isOpened():
        t1 = threading.Thread(target=process_frames)
        t1.start()
        
        while True:
            _, frame = cap.read()

            if frame is None:
                continue

            queue.append(frame)

            if (len(queue) > 5):
                queue.pop(0)

    cap.release()