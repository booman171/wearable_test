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
import math
# import imutils


filename = 'video.avi' # .avi .mp4
frames_per_seconds = 30.0
my_res = (640, 480) #'480p' # 1080p

cap = cv2.VideoCapture(0)

video_type_cv2 = cv2.VideoWriter_fourcc(*'XVID')
save_path = os.path.join(filename)

out = cv2.VideoWriter('video.avi', video_type_cv2,frames_per_seconds,  my_res)
alpha = 0.5
heading = 360
angle = heading - 90
length = 50;
p1x = 590
p1y = 110

def pol2cart(rho, phi):
    x = rho * math.cos(math.radians(phi))
    y = rho * math.sin(math.radians(phi))
    return(x, y)

while(True):
    ret, img = cap.read()
    #img = np.zeros((512, 512, 3), dtype = "uint8")
    overlay = img.copy()
    # (2) draw shapes:
    #penta = np.array([[[540,110],[590,60],[640,110],[590,160]]], np.int32)
    #triangle = np.array([[[240, 130], [380, 230], [190, 280]]], np.int32)
    p2x =  int(p1x + length * math.cos(angle * math.pi / 180.0));
    p2y =  int(p1y + length * math.sin(angle * math.pi / 180.0));
    print("x: ", p2x)
    print("y: ", p2y)
    cv2.circle(overlay, (590, 110), 15, (0, 255, 255), -1)
    cv2.arrowedLine(overlay, (590,110), (p2x, p2y), (0, 255, 0), 2)
    cv2.putText(overlay, "N",(585,72),cv2.FONT_HERSHEY_SIMPLEX,0.5,(51, 51, 0),1,cv2.LINE_AA)
    cv2.putText(overlay, "E",(630,115),cv2.FONT_HERSHEY_SIMPLEX,0.5,(51, 51, 0),1,cv2.LINE_AA)
    cv2.putText(overlay, "S",(585,158),cv2.FONT_HERSHEY_SIMPLEX,0.5,(51, 51, 0),1,cv2.LINE_AA)
    cv2.putText(overlay, "W",(542,115),cv2.FONT_HERSHEY_SIMPLEX,0.5,(51, 51, 0),1,cv2.LINE_AA)
    #cv2.circle(overlay, (166, 132), 12, (0, 255, 0), -1)
    # (3) blend with the original:
    #img_mod = cv2.polylines(overlay, [penta], True, (0,120,255),3)
    opacity = 0.4
    cv2.addWeighted(overlay, opacity, img, 1 - opacity, 0, img)
    out.write(img)

    cv2.imshow('Shapes', img)
    if cv2.waitKey(20) & 0xFF == ord('q'):
        break
