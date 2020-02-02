import os
import numpy as np
import cv2
import time
# import picamera
# import datetime as dt
# import board
# import busio
# import adafruit_sgp30
# import adafruit_lsm9ds1
# import csv
# from datetime import datetime
# from utils import CFEVideoConf, image_resize
# import math
# import imutils


filename = 'video.avi' # .avi .mp4
frames_per_seconds = 30.0
my_res = (640, 480) #'480p' # 1080p

cap = cv2.VideoCapture(0)

video_type_cv2 = cv2.VideoWriter_fourcc(*'XVID')
save_path = os.path.join(filename)

out = cv2.VideoWriter('video.avi', video_type_cv2,frames_per_seconds,  my_res)


while(True):
    ret, img = cap.read()
    #img = np.zeros((512, 512, 3), dtype = "uint8")
    penta = np.array([[[40,160],[120,100],[200,160],[160,240],[80,240]]], np.int32)
    triangle = np.array([[[240, 130], [380, 230], [190, 280]]], np.int32)
    cv2.polylines(img, [triangle], True, (0,255,0), thickness=3)

    img_mod = cv2.polylines(img, [penta], True, (255,120,255),3)

    out.write(img_mod)
    
    cv2.imshow('Shapes', img_mod)
    if cv2.waitKey(20) & 0xFF == ord('q'):
        break
    
f.close()

