from __future__ import division
from __future__ import print_function
from __future__ import absolute_import

import os
import numpy as np
import cv2
import time
import datetime as dt




filename = 'video.avi' # .avi .mp4
frames_per_seconds = 24.0
my_res = (640, 480) #'480p' # 1080p


cap = cv2.VideoCapture(0)

video_type_cv2 = cv2.VideoWriter_fourcc(*'XVID')
save_path = os.path.join(filename)
out = cv2.VideoWriter('video.avi', video_type_cv2, frames_per_seconds, my_res)

elapsed_sec = 0

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    out.write(frame)
    # Display the resulting frame
    cv2.imshow('frame',frame)
    
    if cv2.waitKey(20) & 0xFF == ord('q'):
        break
    
