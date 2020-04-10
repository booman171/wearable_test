from __future__ import division

from __future__ import print_function
from __future__ import absolute_import

import threading
import os
import numpy as np
import cv2
import time
import picamera
import datetime as dt
import csv
from datetime import datetime
import math
import pygame
from pygame.locals import *
import sys
import RPi.GPIO as GPIO
import serial
import board
import busio
import adafruit_lsm9ds1
import adafruit_mcp9808
from overlay import overlay_transparent
from firebase import firebase
#from metawear import MWBoard

pygame.init()
pygame.mouse.set_visible(False)
pygame.display.set_caption("OpenCV camera stream on Pygame")
screen = pygame.display.set_mode([320,240])

font = pygame.font.SysFont("comicsansms", 72)
text = font.render("Hello, World", True, (0, 128, 0))

os.putenv('SDL_FBDEV', '/dev/fb1')

GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(22, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(23, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_UP)

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0)
#ser.flushInput()

filename1 = "data_ecgggggg" + str(time.time()) + ".csv"
f = open("serial_data.csv", "a")
f.write("Epoch,ECG,BPM" + "\n")
#f.close

# Set time at start of program
start = time.monotonic()
timestr = time.strftime("%Y%m%d-%H%M%S")
# Set declination based on location http://www.magnetic-declination.com/
declination = -0.106988683
# Set filename video.avi for recording camera output
filename = 'video' + timestr + '.avi' # .avi .mp4
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

# I2C connection:
#i2c = busio.I2C(board.SCL, board.SDA)
#imu = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)
#mcp = adafruit_mcp9808.MCP9808(i2c)

#accel_x = 0
#accel_y = 0
#accel_z = 0
#mag_x = 0
#mag_y = 0
#mag_z = 0
#gyro_x = 0
#gyro_y = 0
#gyro_z = 0
#imu_temp = 0
#tempC = 0
#tempF = 0

# Read in transport image of thermostat icon
overlay_t = cv2.imread('therm.png', -1)
thermometer = pygame.image.load('therm.png')
thermometer = pygame.transform.rotozoom(thermometer, 0, 0.08)

firebase = firebase.FirebaseApplication('https://wear1-38901.firebaseio.com/')

# Define method for overlaying trasnparent images
def handle_data(data):
    print(data)

def read_from_port():
    cnt = 0
    while True:
        if(ser.inWaiting()>0):
            ser_bytes = ser.read(ser.inWaiting()).decode('ascii')
            message = str(ser_bytes)
            message = message.replace("\r","")
            message = message.replace("\n","")
            sensors = message.split(",")
            if(len(sensors) == 4):
               global tempC
               print(sensors)
               tempC = sensors[len(sensors)-1]
               global tempF
               tempF = sensors[len(sensors)-2]
               global bpm
               bpm = sensors[len(sensors)-3]
               global ecg
               ecg = sensors[len(sensors)-4]

               #store the Host ID(provided in firebase database) in variable where you want to send the real time sensor data.
               #firebase = firebase.FirebaseApplication('https://wear1-38901.firebaseio.com/')

               #store the readings in variable and convert it into string and using firbase.post then data will be posted to databse of firebase
               result = firebase.post('wear1', {'ECG':str(ecg),'BPM':str(bpm), 'Temp F':str(tempF)})

               with open("serial_data.csv", 'a') as n:
                  n.write(str(time.time()) + "," + ecg + "," + bpm)
        time.sleep(0.001)

thread = threading.Thread(target=read_from_port)
thread.start()


tag_date = ""
basicfont = pygame.font.SysFont(None, 48)

show = 400
show2 = 400
cam = False
main = True
recording = False
showSensors = False
ser.readline()

#mwboard = MWBoard()

time.sleep(1)
# Video processing
while(True):
    #accel_x, accel_y, accel_z = imu.acceleration
    #mag_x, mag_y, mag_z = imu.magnetic
    #gyro_x, gyro_y, gyro_z = imu.gyro
    #imu_temp = imu.temperature
    #tempC = mcp.temperature
    #tempF = tempC * 9 / 5 + 32
    #mwboard.stream()
    if main == True:
        now = datetime.now()
        bigFont = pygame.font.SysFont(None, 48)
        medFont = pygame.font.SysFont(None, 32)
        smallFont = pygame.font.SysFont(None, 24)
        clock = bigFont.render(now.strftime("%H:%M:%S"), True, (0, 0, 0))
        exit_button = medFont.render("Exit", True, (0, 255, 0))
        cam_button = medFont.render("Cam", True, (0, 255, 0))
        temp = medFont.render(str(tempF) + " C", True, (255, 0, 0))
        showBPM = medFont.render("BPM: " + bpm, True, (255, 0, 0))
        #p = medFont.render("Pitch: " + pitch, True, (0, 0, 255))
        #r = medFont.render("Roll: " + roll, True, (0, 0, 255))
        #y = medFont.render("Yaw: " + yaw, True, (0, 0, 255))
        screen.fill((255, 149, 0))
        screen.blit(clock, (5, 5))
        screen.blit(exit_button, (270,210))
        screen.blit(cam_button, (270,160))
        screen.blit(temp, (30, 190))
        screen.blit(thermometer, (1,180))
        screen.blit(showBPM, (10,60))
        #screen.blit(p, (10,100))
        #screen.blit(r, (10,120))
        #screen.blit(y, (10,140))
        pygame.display.update()
        
        if GPIO.input(23) == False:
            main = False
            cam = True
        
        # Break loop
        if GPIO.input(27) == False:
            pygame.quit()
            break
    
    if cam == True:
        # set each frame from camera as 'frame'
        ret, frame = cap.read()
        frame1 = frame.copy()
        screen.fill([0,0,0])
        # Create overlay of frame add transparent image at screen coordinates (10, 80)
        #overlay = overlay_transparent(frame, overlay_t, 10, 80, (50,50))
        #ov = overlay_transparent(frame1, overlay_t, 5, 80, (50,50))
        
        # Set var now to current date/time
        now = datetime.now()
        
        # Set opacity for overlay transparency, the closer to 0 the more transparent
        opacity = 0.8

        # Overlay date text
        #cv2.putText(overlay,now.strftime("%H:%M:%S"),(30,50),cv2.FONT_HERSHEY_SIMPLEX,0.8,(1, 1, 0),2,cv2.LINE_AA)
        cv2.putText(frame1,now.strftime("%H:%M:%S"),(20,50),cv2.FONT_HERSHEY_SIMPLEX,0.8,(0,255,0),2,cv2.LINE_AA)
        cv2.putText(frame1,"Rec",(275,55),cv2.FONT_HERSHEY_SIMPLEX,0.8,(0,255,0),1,cv2.LINE_AA)
        cv2.putText(frame1,str(tempF),(show2,200),cv2.FONT_HERSHEY_SIMPLEX,0.8,(0,255,0),1,cv2.LINE_AA)
        cv2.putText(frame1,bpm,(show2,230),cv2.FONT_HERSHEY_SIMPLEX,0.8,(0,255,0),1,cv2.LINE_AA)
        cv2.putText(frame1,"Rec-Bio",(215,120),cv2.FONT_HERSHEY_SIMPLEX,0.8,(0,255,0),1,cv2.LINE_AA)
        cv2.putText(frame1,"Snap",(255,175),cv2.FONT_HERSHEY_SIMPLEX,0.8,(0,255,0),1,cv2.LINE_AA)
        cv2.putText(frame1,"Menu",(255,230),cv2.FONT_HERSHEY_SIMPLEX,0.8,(0,255,0),1,cv2.LINE_AA)
        cv2.putText(frame1,"Recording",(show,20),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,255),1,cv2.LINE_AA)
        
        if recording == True:
            out.write(frame)

        if showSensors == True:
            out.write(frame)

        # Combine overlay to frame, apply transparency
        #cv2.addWeighted(overlay, opacity, frame, 1 - opacity, 0, frame)
        #cv2.addWeighted(ov, opacity, frame1, 1 - opacity, 0, frame1)
        #out.write(frame)

        frame1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2RGB)
        frame1 =  np.rot90(frame1)
        frame1 = cv2.flip(frame1, 0)
        frame1 = pygame.surfarray.make_surface(frame1)
        screen.blit(frame1, (0,0))
        pygame.display.update()
        #print(str(show))
        
        if GPIO.input(17) == False:
            if recording == True:
                recording = not recording
                show = 500
                time.sleep(0.5)
            elif recording == False:
                recording = not recording
                show = 245
                # Create video
                out = cv2.VideoWriter(save_path, video_type_cv2,frames_per_seconds,  my_res)
                time.sleep(0.5)
        
        if GPIO.input(22) == False:
            if showSensors == True:
                showSensors = not showSensors
                show2 = 500
                time.sleep(0.5)
            elif showSensors == False:
                showSensors = not showSensors
                show2 = 2
                # Create video
                out = cv2.VideoWriter(save_path, video_type_cv2,frames_per_seconds,  my_res)
                time.sleep(0.5)
                
        if GPIO.input(23) == False and main == False:
            filename = "image_" + now.strftime("%H:%M:%S") + ".jpg"
            save_path = os.path.join(filename)
            cv2.imwrite(save_path, frame)
    
        if GPIO.input(27) == False:
            cam = False
            time.sleep(0.5)
            main = True

	# Break loop
    if cv2.waitKey(50) & 0xFF == ord('q'):
        break

