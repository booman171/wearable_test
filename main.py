from __future__ import division
from __future__ import print_function
from __future__ import absolute_import

import threading
import os
import numpy as np
import cv2
import time
#import picamera
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
#import adafruit_lsm9ds1
#import adafruit_mcp9808
from overlay import overlay_transparent
#from firebase import firebase
import glob
import bluetooth
#from streamer import StreamingOutput, StreamingHandler, StreamingServer
import argparse
#import os
#import cv2
from get_video import VideoGet
from flask import Flask, render_template, Response
import importlib.util
from read_usb import readUSB

#from sensor_serial import read_from_port
#from metawear import MWBoard

#os.putenv('SDL_MOUSEDEV', '/dev/input/mouse')
os.putenv('SDL_MOUSEDEV', '/dev/input/mouse0')
pygame.init()
#pygame.display.init()
#time.sleep(1)
pygame.mouse.set_visible(False)

pygame.display.set_caption("OpenCV camera stream on Pygame")
screen = pygame.display.set_mode([320,240])
font = pygame.font.SysFont("comicsansms", 72)
text = font.render("Hello, World", True, (0, 128, 0))

os.putenv('SDL_FBDEV', '/dev/fb1')
os.system('modprobe w1-gpio')
os.system('modprobe w1-therm')

#host = ""
#port = 1    # Raspberry Pi uses port 1 for Bluetooth Communication
# Creaitng Socket Bluetooth RFCOMM communication
#server = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
#print('Bluetooth Socket Created')

GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(22, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(23, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_UP)

#ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0)
#ser.flushInput()

filename1 = "data_ecgggggg" + str(time.time()) + ".csv"
f = open("serial_data.csv", "a")
f.write("Epoch,ECG,BPM" + "\n")
#f.close

# Set time at start of program
#start = time.monotonic()
#timestr = time.strftime("%Y%m%d-%H%M%S")
# Set declination based on location http://www.magnetic-declination.com/
#declination = -0.106988683
# Set filename video.avi for recording camera output
#filename = 'video' + timestr + '.avi' # .avi .mp4
# Set picamera standard frame rate
#fps = 5.0
# Set recording resolution
#capture = cv2.VideoCapture(0)
#frame_width = int(capture.get(3))
#frame_height = int(capture.get(4))
# Read from default (0) camera
#cap = cv2.VideoCapture(-1)
# Set video format
#video_writer = cv2.VideoWriter_fourcc('M','J','P','G')
#video_out = cv2.VideoWriter(filename, video_writer, fps, (640, 480))
# Save video.avi to current directory
#save_path = os.path.join(filename)

# Read in transport image of thermostat icon
overlay_t = cv2.imread('therm.png', -1)
thermometer = pygame.image.load('therm.png')
thermometer = pygame.transform.rotozoom(thermometer, 0, 0.08)

#firebase = firebase.FirebaseApplication('https://wear1-38901.firebaseio.com/')'

#global connected
#connected = False

global send
send = False
host = ""
port = 1    # Raspberry Pi uses port 1 for Bluetooth Communication
# Creaitng Socket Bluetooth RFCOMM communication


server = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
print('Bluetooth Socket Created')
global connected
connected = False
try:  #to connect
   server.bind((host, port))
   #connected = True
   print("Bluetooth Binding Completed")
except:
   print("Bluetooth Binding Failed")
server.listen(1) # One connection at a time
# Server accepts the clients request and assigns a mac address.
client, address = server.accept()
print("Connected To", address)
print("Client:", client)
connected = True

def recvMSG():
   global recv
   while True:
      # Receivng the data.
      data = client.recv(1024) # 1024 is the buffer size.
      data = str(data,"utf-8")
      #print("Received: " + data)
      recv = data.split(",")

def sendMSG():
   global key
   start = False
   key = 1
   while True:
      if GPIO.input(17) == False:
         if key == 1:
            key = 7
         else:
            key = key - 1
         print("KEY: " + str(key))
         time.sleep(1.0)
      if GPIO.input(22) == False:
          if key == 7:
            key = 1
          else:
            key = key + 1
          time.sleep(1.0)
          print("KEY: " + str(key))

      if GPIO.input(23) == False:
         print("sent: " + str(key))
         send = False
         send_data = str(key) + "\r\n"
         client.send(send_data)
         time.sleep(0.5)

      if GPIO.input(27) == False:
         if start  == False:
            send_data = "record" + "\r\n"
            client.send(send_data)
            start = True
         elif start == True:
            send_data = "stop" + "\r\n"
            client.send(send_data)
            start = False
         time.sleep(0.5)

#thread = threading.Thread(target=read_from_port)
#thread.start()

sendThread = threading.Thread(target=sendMSG)
sendThread.start()
recvThread = threading.Thread(target=recvMSG)
recvThread.start()

#global connected
tag_date = ""
basicfont = pygame.font.SysFont(None, 48)

show = 400
show2 = 400
cam = False
main = True

showSensors = False
#ser.readline()
WHITE = (255, 255, 255)
#mwboard = MWBoard()
menu_items = ["J", "Play/P", "Next", "Prev", "Vol+", "Vol-", "voice", "Back"]
temperature = 0

arduino1 = readUSB("ttyACM0", 9600)
arduino1.startSensorRead()

ds_factor = 0.6
# Initialize video stream
# videostream = VideoStream(resolution=(imW,imH),framerate=30).start()
time.sleep(1)

#global frame
'''
Thread for getting video from get_video
'''
#video_getter = VideoGet(source).start()
global menu_key
menu_key = 1
record = False
recording = False
width = 320
height = 240

blackColor =  pygame.Color(  0,  0, 0)
yellowColor = pygame.Color(255,255, 0)
redColor =    pygame.Color(255,  0, 0)
greenColor =  pygame.Color(  0,255, 0)
rec_color = pygame.Color(255,255, 0)

alpha = 0.4
oldX = 0
oldY = 0

frames = []

check_usb = True
var_set = False
tempF = ""
bpm = ""

# set SCALE
J = 100000
start = 0
print("vfvdsvdF")

video_getter = VideoGet(0).start()
#cps = CountsPerSec().start()
while True:
        if (cv2.waitKey(15) == ord("q")) or video_getter.stopped:
                video_getter.stop()
                break
        _, frame = video_getter.frame
        frame1 = frame.copy()

        frame1=cv2.resize(frame1,None,fx=ds_factor,fy=ds_factor,interpolation=cv2.INTER_AREA)
        gray=cv2.cvtColor(frame1,cv2.COLOR_BGR2GRAY)
        ret, jpeg = cv2.imencode('.jpg', frame1)
        #cv2.imshow("Video", frame1)
        jpg = jpeg.tobytes()

        #converting frame(img i.e BGR) to HSV (hue-saturation-value)
        hsv=cv2.cvtColor(frame1,cv2.COLOR_BGR2HSV)
        #definig the range of red color
        red_lower=np.array([136,87,111],np.uint8)
        red_upper=np.array([180,255,255],np.uint8)
        #defining the Range of Blue color
        blue_lower=np.array([99,115,150],np.uint8)
        blue_upper=np.array([110,255,255],np.uint8)
        #defining the Range of yellow color
        yellow_lower=np.array([22,60,200],np.uint8)
        yellow_upper=np.array([60,255,255],np.uint8)
        #finding the range of red,blue and yellow color in the image
        red=cv2.inRange(hsv, red_lower, red_upper)
        blue=cv2.inRange(hsv,blue_lower,blue_upper)
        yellow=cv2.inRange(hsv,yellow_lower,yellow_upper)
        #Morphological transformation, Dilation  	
        kernal = np.ones((5 ,5), "uint8")
        
        data = arduino1.getData().split(",")
        if len(arduino1.getData()) >= 2:
           bpm = "BPM: " + data[0]
           tempF = "Temp: " + data[1][0:3] + " F"

        if main == True:
            now = datetime.now()
            #ret, frame = cap.read()
            cam_out = frame.copy()
            output = frame.copy()

            red=cv2.dilate(red, kernal)
            res=cv2.bitwise_and(frame, frame, mask = red)

            blue=cv2.dilate(blue,kernal)
            res1=cv2.bitwise_and(frame, frame, mask = blue)

            yellow=cv2.dilate(yellow,kernal)
            res2=cv2.bitwise_and(frame, frame, mask = yellow)
            #Tracking the Red Color
            (_,contours,hierarchy)=cv2.findContours(red,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            screen.fill([0,0,0])

            bigFont = pygame.font.SysFont(None, 48)
            medFont = pygame.font.SysFont(None, 32)
            smallFont = pygame.font.SysFont(None, 18)
            clock = bigFont.render(now.strftime("%H:%M:%S"), True, (0, 255, 0))
            up_button = medFont.render("/\\", True,(0, 0, 255))
            down_button = medFont.render("\\/", True, (0, 0, 255))
            select_button = medFont.render(menu_items[key], True, (0, 0, 255))

            rec_button  = medFont.render("REC", True, rec_color)
            #exit_button = medFont.render("Exit", True, (0, 255, 0))
            if len(recv) > 1:
               lattitude = smallFont.render("LAT: " + recv[0], True, (0, 0, 255))
            if len(recv) > 2:
               longitude = smallFont.render("LON: " + recv[1], True, (0, 0, 255))
            if len(recv) > 3:
               alt = smallFont.render("ALT: " + recv[2], True, (0, 0, 255))
               cv2.putText(cam_out, "Altitude: " + recv[2],(10, 430), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 3)
            if len(recv) > 4:
               bea = smallFont.render("BEA: " + recv[3], True, (0, 0, 255))
            if len(recv) > 5:
               spe = smallFont.render("SPE: " + recv[4], True, (0, 0, 255))
               cv2.putText(cam_out, "Speed: " + recv[4],(10, 390), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 3)

            #lat = float(recv[0])
            #lon = float(recv[1])
            #speed = float(recv[4])


            #screen.fill((255, 149, 0))
            screen.blit(clock, (190, 205))
            screen.blit(rec_button, (270, 180))
            #screen.blit(exit_button, (10,210))
            #screen.blit(cam_button, (10,5))
            #screen.blit(lattitude, (5, 90))
            #screen.blit(longitude, (5, 110))
            screen.blit(alt, (5, 130))
            #screen.blit(bea, (5, 150))
            screen.blit(spe, (5, 150))

            cv2.putText(cam_out, now.strftime("%H:%M:%S"),(10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 3)

            for pic, contour in enumerate(contours):
               area = cv2.contourArea(contour)
               if(area>300):
                  x,y,w,h = cv2.boundingRect(contour)	
                  frame1 = cv2.rectangle(frame1,(x,y),(x+w,y+h),(0,0,255),2)
                  cv2.putText(frame1,"RED color",(x,y),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255))
            #Tracking the Blue Color
            (_,contours,hierarchy)=cv2.findContours(blue,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            for pic, contour in enumerate(contours):
               area = cv2.contourArea(contour)
               if(area>300):
                  x,y,w,h = cv2.boundingRect(contour)	
                  frame1 = cv2.rectangle(frame1,(x,y),(x+w,y+h),(255,0,0),2)
                  cv2.putText(frame1,"Blue color",(x,y),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,0,0))
            #Tracking the yellow Color
            (_,contours,hierarchy)=cv2.findContours(yellow,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            for pic, contour in enumerate(contours):
               area = cv2.contourArea(contour)
               if(area>300):
                  x,y,w,h = cv2.boundingRect(contour)	
                  frame1 = cv2.rectangle(frame1,(x,y),(x+w,y+h),(0,255,0),2)
                  cv2.putText(frame1,"yellow  color",(x,y),cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,255,0))
                  
            #if os.path.exists('/dev/ttyACM0') == True and var_set == True:
            temp = medFont.render(tempF, True, (0, 0, 255))
            showBPM = medFont.render(bpm, True, (0, 0, 255))
            screen.blit(temp, (5, 205))
            screen.blit(showBPM, (5, 180))
            cv2.putText(cam_out, tempF,(440, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 0), 3)
            cv2.putText(cam_out, bpm,(10, 350), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)

            # apply the overlay
            cv2.addWeighted(cam_out, alpha, output, 1 - alpha, 0, output)
            frames.append(output)

            if connected == True:
               screen.blit(select_button, (5, 10))

            frame1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2RGB)
            frame1 =  np.rot90(frame1)
            frame1 = cv2.flip(frame1, 0)

            
            # Displays live camera output on screen
            frame1 = pygame.surfarray.make_surface(frame1)
            screen.blit(frame1, (80,5), (0, 0, 240, 160))
            
            #screen.blit(image, (200, 300), (640,512,200,200))

            #screen.blit(thermometer, (190,200))
            #screen.blit(showBPM, (10,210))

            pygame.display.update()

            if GPIO.input(17) == False:
                if menu_key == 1:
                   menu_key = 7
                else:
                   menu_key = menu_key - 1
                time.sleep(0.5)

            if GPIO.input(22) == False:
                if menu_key == 7:
                   menu_key = 1
                else:
                   menu_key = menu_key + 1
                time.sleep(0.5)

            if GPIO.input(23) == False:
                send_data = str(menu_key) + "\r\n"
                send = True
                print("send")
                time.sleep(0.5)
                #send = False

            if GPIO.input(27) == False:
                if record == True:
                   print("Recording stopped: " + str(record))
                   record = False
                   rec_color = pygame.Color(255,255, 0)
                   time.sleep(0.5)
                else:
                   print("Recording: " + str(record))
                   rec_color = pygame.Color(255,  0, 0)
                   record = True
                   time.sleep(0.5)
        #yield (b'--frame\r\n'b'Content-Type: image/jpeg\r\n\r\n' + jpg + b'\r\n\r\n')


