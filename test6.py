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
import math
import imutils


f = open("accel.csv", "w", newline="")
c=csv.writer(f)

start = time.monotonic()

logo = cv2.imread('images/compass_letters.png', -1)
watermark = image_resize(logo, height=100)
watermark = cv2.cvtColor(watermark, cv2.COLOR_BGR2BGRA)

point = cv2.imread('images/compass_point.png', -1)

xyz = 5
x = 0
y = 0
z = 0
declination = -0.106988683
filename = 'video.avi' # .avi .mp4
frames_per_seconds = 30.0
my_res = (640, 480) #'480p' # 1080p


cap = cv2.VideoCapture(0)

video_type_cv2 = cv2.VideoWriter_fourcc(*'XVID')
save_path = os.path.join(filename)

out = cv2.VideoWriter('video.avi', video_type_cv2,frames_per_seconds,  my_res)

# I2C connection:
i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)
sensor = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)

# Create library object on our I2C port
#sgp30 = adafruit_sgp30.Adafruit_SGP30(i2c)
#print("SGP30 serial #", [hex(i) for i in sgp30.serial])
#sgp30.iaq_init()
#sgp30.set_iaq_baseline(0x8973, 0x8aae)

def overlay_transparent(background_img, img_to_overlay_t, x, y, overlay_size=None):
	"""
	@brief      Overlays a transparant PNG onto another image using CV2
	
	@param      background_img    The background image
	@param      img_to_overlay_t  The transparent image to overlay (has alpha channel)
	@param      x                 x location to place the top-left corner of our overlay
	@param      y                 y location to place the top-left corner of our overlay
	@param      overlay_size      The size to scale our overlay to (tuple), no scaling if None
	
	@return     Background image with overlay on top
	"""
	
	bg_img = background_img.copy()
	
	if overlay_size is not None:
		img_to_overlay_t = cv2.resize(img_to_overlay_t.copy(), overlay_size)

	# Extract the alpha mask of the RGBA image, convert to RGB 
	b,g,r,a = cv2.split(img_to_overlay_t)
	overlay_color = cv2.merge((b,g,r))
	
	# Apply some simple filtering to remove edge noise
	mask = cv2.medianBlur(a,5)

	h, w, _ = overlay_color.shape
	roi = bg_img[y:y+h, x:x+w]

	# Black-out the area behind the logo in our original ROI
	img1_bg = cv2.bitwise_and(roi.copy(),roi.copy(),mask = cv2.bitwise_not(mask))
	
	# Mask out the logo from the logo image.
	img2_fg = cv2.bitwise_and(overlay_color,overlay_color,mask = mask)

	# Update the original image with our new ROI
	bg_img[y:y+h, x:x+w] = cv2.add(img1_bg, img2_fg)

	return bg_img

elapsed_sec = 0

length = 50;
p1x = 590
p1y = 110
overlay_t = cv2.imread('therm.png',-1)

while(True):
    ret, frame = cap.read()
    #overlay = frame.copy()
    overlay = overlay_transparent(frame, overlay_t, 10, 80, (50,50))

    
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

    theta = angle - 90
    
    p2x =  int(p1x + length * math.cos(theta * math.pi / 180.0));
    p2y =  int(p1y + length * math.sin(theta * math.pi / 180.0));
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
    opacity = 0.8
    cv2.putText(overlay,now.strftime("%H:%M:%S"),(30,50),cv2.FONT_HERSHEY_SIMPLEX,0.8,(51, 51, 0),2,cv2.LINE_AA)
    cv2.putText(overlay,"Heading: " + str(round(angle, 1)),(500,50),cv2.FONT_HERSHEY_SIMPLEX,0.5,(51, 51, 0),2,cv2.LINE_AA)
    cv2.putText(overlay,"Temp(C): ",(50,95),cv2.FONT_HERSHEY_SIMPLEX,0.3,(51, 51, 0),1,cv2.LINE_AA)
    cv2.putText(overlay,str(temp),(50,120),cv2.FONT_HERSHEY_SIMPLEX,0.8,(51, 51, 0),2,cv2.LINE_AA)
    #print("eCO2 = %d ppm \t TVOC = %d ppb" % (sgp30.eCO2, sgp30.TVOC))

    cv2.addWeighted(overlay, opacity, frame, 1 - opacity, 0, frame)
    #elapsed_sec += 1
    out.write(frame)
    # Display the resulting frame
    cv2.imshow('frame',frame)
    
    
    if cv2.waitKey(20) & 0xFF == ord('q'):
        break
    
f.close()
