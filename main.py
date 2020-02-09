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
import math
# import imutils

# Create accel.csv file to write sensor data to
f = open("accel.csv", "w", newline="")
c=csv.writer(f)

# Set time at start of program
start = time.monotonic()

# Set declination based on location http://www.magnetic-declination.com/
declination = -0.106988683

# Set filename video.avi for recording camera output
filename = 'video.avi' # .avi .mp4
# Set picamera standard frame rate
frames_per_seconds = 30.0
# Set recording resolution
my_res = (640, 480) #'480p' # 1080p
# Read from default (0) camera
cap = cv2.VideoCapture(0)
# Set video format
video_type_cv2 = cv2.VideoWriter_fourcc(*'XVID')
# Save video.avi to current directory
save_path = os.path.join(filename)
# Create video
out = cv2.VideoWriter('video.avi', video_type_cv2,frames_per_seconds,  my_res)

# I2C connection for sensor LSM9DS1 https://www.adafruit.com/product/3387
i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)
sensor = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)

# Setup for gas sensor https://www.adafruit.com/product/3709
sgp30 = adafruit_sgp30.Adafruit_SGP30(i2c)
print("SGP30 serial #", [hex(i) for i in sgp30.serial])
sgp30.iaq_init()
sgp30.set_iaq_baseline(0x8973, 0x8aae)

# Define text-to-speak  function
def speak(text):
	os.system("espeak ' " + text + " ' ")

# Define method for overlaying trasnparent images
# copied from https://gist.github.com/clungzta/b4bbb3e2aa0490b0cfcbc042184b0b4e
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

# Var for radius of compass
length = 50;
# (x,y) position for start of compass arrow
p1x = 590
p1y = 110
# Read in transparent image of thermostat icon
overlay_t = cv2.imread('therm.png',-1)

max = 2400
min = 400
breath = 0
warmup = 0
# Video processing
while(True):
    # set each frame from camera as 'frame'
    ret, frame = cap.read()
    # Create overlay of frame add transparent image at screen coordinates (10, 80)
    overlay = overlay_transparent(frame, overlay_t, 10, 80, (50,50))
    
    warmup += 1
    # Read acceleration, magnetometer, gyroscope,
    # and temperature from the LSM9DS1 Sensor
    accel_x, accel_y, accel_z = sensor.acceleration
    mag_x, mag_y, mag_z = sensor.magnetic
    gyro_x, gyro_y, gyro_z = sensor.gyro
    temp = sensor.temperature
    # Read eCO2 rom SGP30 Sensor
    eco2 = sgp30.eCO2
    # Set var now to current date/time
    now = datetime.now()
    # Write time and sensor data to csv by column
    c.writerow([time.monotonic(), eco2, accel_x, accel_y, accel_z, temp])
    
    # Calculate heading from magnetometer data
    # Code adapted from https://www.electronicwings.com/avr-atmega/magnetometer-hmc5883l-interfacing-with-atmega16
    heading = math.atan2(float(mag_y),float(mag_z)) + declination
    if heading > 2*math.pi:
        heading = heading - 2*math.pi
    if heading < 0:
        heading = heading + 2*math.pi
    angle = (heading* 180 / math.pi)
    theta = angle - 90
    
    # Calculate commpass head position (x, y) corresponding to heading angle
    p2x =  int(p1x + length * math.cos(theta * math.pi / 180.0));
    p2y =  int(p1y + length * math.sin(theta * math.pi / 180.0));
    #print("x: ", p2x)
    #print("y: ", p2y)
    
    # Normalization equation: (x - min)/(max - min)
    # This will convert a data range [min,max] to the range [0,1]
    # To correpond the normalized eCO2 to the screen I need to
    # invert the normalized range by subtracting theefunction from 1:
    # 1-(x - min)/(max-min) or (max - x)/(max - min)
    gas = (max - eco2) / (max - min)
    breath = int(gas*100 + 250)
    # Gas sensor warmup complete around warmup = 120
    # print(str(warmup))
    print("breath = %2f" % (breath))
    # Overlay 'Breath: '
    cv2.putText(overlay,"Breath: ",(25,370),cv2.FONT_HERSHEY_SIMPLEX,0.5,(51, 51, 0),1,cv2.LINE_AA)
    # Overlay eCO2 data
    cv2.putText(overlay,str(eco2) + "ppm",(25,390),cv2.FONT_HERSHEY_SIMPLEX,0.5,(51, 51, 0),1,cv2.LINE_AA)
    # Breath meter
    cv2.rectangle(overlay, (30, breath), (35,350), (255, 0, 0), 5)
    # Overlay circle as center of compass
    cv2.circle(overlay, (590, 110), 15, (0, 255, 255), -1)
    # Overlay compass arrow, start point at center of compass, point at heading (px2, pxy)
    cv2.arrowedLine(overlay, (590,110), (p2x, p2y), (0, 255, 0), 2)
    # Overlay the four Cardinal directions for the compass
    cv2.putText(overlay, "N",(585,72),cv2.FONT_HERSHEY_SIMPLEX,0.5,(51, 51, 0),1,cv2.LINE_AA)
    cv2.putText(overlay, "E",(630,115),cv2.FONT_HERSHEY_SIMPLEX,0.5,(51, 51, 0),1,cv2.LINE_AA)
    cv2.putText(overlay, "S",(585,158),cv2.FONT_HERSHEY_SIMPLEX,0.5,(51, 51, 0),1,cv2.LINE_AA)
    cv2.putText(overlay, "W",(542,115),cv2.FONT_HERSHEY_SIMPLEX,0.5,(51, 51, 0),1,cv2.LINE_AA)

    # Set opacity for overlay transparency, the closer to 0 the more transparent
    opacity = 0.8
     
    # Overlay date text
    cv2.putText(overlay,now.strftime("%H:%M:%S"),(30,50),cv2.FONT_HERSHEY_SIMPLEX,0.8,(51, 51, 0),2,cv2.LINE_AA)
    # Overlay 'Heading' heading
    cv2.putText(overlay,"Heading: " + str(round(angle, 1)),(500,50),cv2.FONT_HERSHEY_SIMPLEX,0.5,(51, 51, 0),2,cv2.LINE_AA)
    # Overlay 'Temp(C):' heading
    cv2.putText(overlay,"Temp(C): ",(50,95),cv2.FONT_HERSHEY_SIMPLEX,0.3,(51, 51, 0),1,cv2.LINE_AA)
    # Overlay temp data
    cv2.putText(overlay,str(temp),(50,120),cv2.FONT_HERSHEY_SIMPLEX,0.8,(51, 51, 0),2,cv2.LINE_AA)
# 
    # Combine overlay to frame, apply transparency
    cv2.addWeighted(overlay, opacity, frame, 1 - opacity, 0, frame)

    # Write frame to video
    out.write(frame)

    # Display the resulting frame
    # Comment out if using ssh to run script
    cv2.imshow('frame',frame)

    # Break loop
    if cv2.waitKey(20) & 0xFF == ord('q'):
        break

	# Close csv file   
f.close()
