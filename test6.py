from __future__ import division
from __future__ import print_function
from __future__ import absolute_import

import os
import numpy as np
import cv2
import time
import picamera
import datetime as dt
import board
import busio
import adafruit_sgp30
import adafruit_lsm9ds1
import csv
from datetime import datetime
from utils import CFEVideoConf, image_resize



f = open("accel.csv", "w", newline="")
c=csv.writer(f)

start = time.monotonic()

logo = cv2.imread('images/compass_letters.png', -1)
watermark = image_resize(logo, height=100)
watermark = cv2.cvtColor(watermark, cv2.COLOR_BGR2BGRA)



xyz = 5
x = 0
y = 0
z = 0
declination = -0.106988683
filename = 'video.avi' # .avi .mp4
frames_per_seconds = 24.0
my_res = (640, 480) #'480p' # 1080p


cap = cv2.VideoCapture(0)

video_type_cv2 = cv2.VideoWriter_fourcc(*'XVID')
save_path = os.path.join(filename)

out = cv2.VideoWriter('video.avi', video_type_cv2, frames_per_seconds, my_res)

# I2C connection:
i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)
sensor = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)

# Create library object on our I2C port
#sgp30 = adafruit_sgp30.Adafruit_SGP30(i2c)
#print("SGP30 serial #", [hex(i) for i in sgp30.serial])
#sgp30.iaq_init()
#sgp30.set_iaq_baseline(0x8973, 0x8aae)

elapsed_sec = 0

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2BGRA)
    frame_h, frame_w, frame_c = frame.shape
    # overlay with 4 channels BGR and Alpha
    overlay = np.zeros((frame_h, frame_w, 4), dtype='uint8')
    
    watermark_h, watermark_w, watermark_c = watermark.shape
    # replace overlay pixels with watermark pixel values
    for i in range(0, watermark_h):
        for j in range(0, watermark_w):
            if watermark[i,j][3] != 0:
                offset = 10
                h_offset = frame_h - watermark_h - offset
                w_offset = frame_w - watermark_w - offset
                overlay[i, w_offset+ j] = watermark[i,j]

    point = cv2.imread('images/compass_point.png', -1)
    compass = image_resize(point, height=100)
    compass = cv2.cvtColor(compass, cv2.COLOR_BGR2BGRA)
    
    compass_h, compass_w, compass_c = compass.shape
    # replace overlay pixels with compass pixel values
    for i in range(0, compass_h):
        for j in range(0, compass_w):
            if compass[i,j][3] != 0:
                offset = 10
                h_offset = frame_h - compass_h - offset
                w_offset = frame_w - compass_w - offset
                overlay[i, w_offset+ j] = compass[i,j]
                
    cv2.addWeighted(overlay, 0.25, frame, 1.0, 0, frame)

    frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
    
    # Read acceleration, magnetometer, gyroscope, temperature.
    accel_x, accel_y, accel_z = sensor.acceleration
    mag_x, mag_y, mag_z = sensor.magnetic
    gyro_x, gyro_y, gyro_z = sensor.gyro
    temp = sensor.temperature
    now = datetime.now()
    c.writerow([time.monotonic(), accel_x, accel_y, accel_z, temp])
    
    heading = math.atan2(float(mag_y),float(mag_z)) + declination
    if heading > 2*math.pi:
        heading = heading - 2*math.pi
        
    if heading < 0:
        heading = heading + 2*math.pi
    angle = (heading* 180 / math.pi)
    
    # Print values.
    print('Heading Angle: ({0:0.3f})'.format(angle))
    #print('Acceleration (m/s^2): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(accel_x, accel_y, accel_z))
    #print('Magnetometer (gauss): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(mag_x, mag_y, mag_z))
    #print('Gyroscope (degrees/sec): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(gyro_x, gyro_y, gyro_z))
    #print('Temperature: {0:0.3f}C'.format(temp))
    
    # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Draw framerate in corner of frame
    #print("eCO2 = %d ppm \t TVOC = %d ppb" % (sgp30.eCO2, sgp30.TVOC))
    
    #cv2.putText(frame,now.strftime("%H:%M:%S"),(30,50),cv2.FONT_HERSHEY_SIMPLEX,1,(51, 51, 0),2,cv2.LINE_AA)
    #cv2.putText(frame,str(sgp30.eCO2),(30,90),cv2.FONT_HERSHEY_SIMPLEX,1,(51, 51, 0),2,cv2.LINE_AA)
    #print("eCO2 = %d ppm \t TVOC = %d ppb" % (sgp30.eCO2, sgp30.TVOC))
    #time.sleep(1)
    elapsed_sec += 1
    out.write(frame)
    # Display the resulting frame
    cv2.imshow('frame',frame)
    
    
    if cv2.waitKey(20) & 0xFF == ord('q'):
        break
    
f.close()
