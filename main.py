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

#from sensor_serial import read_from_port
#from metawear import MWBoard

#os.putenv('SDL_MOUSEDEV', '/dev/input/mouse')
os.putenv('SDL_MOUSEDEV', '/dev/input/mouse0')
pygame.init()
#pygame.display.init()
#time.sleep(1)
#pygame.mouse.set_visible(False)

pygame.display.set_caption("OpenCV camera stream on Pygame")
#screen = pygame.display.set_mode([320,240])

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
#cap = cv2.VideoCapture(-1)
# Set video format
video_type_cv2 = cv2.VideoWriter_fourcc(*'XVID')
# Save video.avi to current directory
save_path = os.path.join(filename)

# Read in transport image of thermostat icon
overlay_t = cv2.imread('therm.png', -1)
thermometer = pygame.image.load('therm.png')
thermometer = pygame.transform.rotozoom(thermometer, 0, 0.08)

#firebase = firebase.FirebaseApplication('https://wear1-38901.firebaseio.com/')

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
               #print(sensors)
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
               #result = firebase.post('wear1', {'ECG':str(ecg),'BPM':str(bpm), 'Temp F':str(tempF)})

               with open("serial_data.csv", 'a') as n:
                  n.write(str(time.time()) + "," + ecg + "," + bpm)
        time.sleep(0.001)

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
recording = False
showSensors = False
#ser.readline()
WHITE = (255, 255, 255)
#mwboard = MWBoard()
menu_key = 1
menu_items = ["J", "Play/P", "Next", "Prev", "Vol+", "Vol-", "voice", "Back"]
time.sleep(1)

ds_factor = 0.6

#global frame
def threadVideoGet(source=0):
        '''
        Thread for getting video from get_video
        '''
        video_getter = VideoGet(source).start()
        global menu_key

        width = 320
        height = 240

        blackColor =  pygame.Color(  0,  0, 0)
        yellowColor = pygame.Color(255,255, 0)
        redColor =    pygame.Color(255,  0, 0)
        greenColor =  pygame.Color(  0,255, 0)

        oldX = 0
        oldY = 0

        # set SCALE
        J = 100000
        start = 0

        while True:
                if (cv2.waitKey(1) == ord("q")) or video_getter.stopped:
                        video_getter.stop()
                        break
                frame = video_getter.frame
                frame=cv2.resize(frame,None,fx=ds_factor,fy=ds_factor,interpolation=cv2.INTER_AREA)
                gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
                ret, jpeg = cv2.imencode('.jpg', frame)
                #cv2.imshow("Video", frame)
                jpg = jpeg.tobytes()

                if main == True:
                    now = datetime.now()
                    #ret, frame = cap.read()
                    frame1 = frame.copy()
                    screen.fill([0,0,0])

                    bigFont = pygame.font.SysFont(None, 48)
                    medFont = pygame.font.SysFont(None, 32)
                    smallFont = pygame.font.SysFont(None, 18)
                    clock = bigFont.render(now.strftime("%H:%M:%S"), True, (255, 0, 0))
                    up_button = medFont.render("/\\", True,(0, 0, 255))
                    down_button = medFont.render("\\/", True, (0, 0, 255))
                    select_button = medFont.render(menu_items[key], True, (0, 0, 255))

                    #cam_button = medFont.render("Cam", True, (0, 255, 0))
                    #exit_button = medFont.render("Exit", True, (0, 255, 0))
                    #temp = medFont.render(str(tempF) + " C", True, (255, 0, 0))
                    #showBPM = medFont.render("BPM: " + bpm, True, (255, 0, 0))
                    lattitude = smallFont.render("LAT: " + recv[0], True, (0, 0, 255))
                    longitude = smallFont.render("LON: " + recv[1], True, (0, 0, 255))
                    alt = smallFont.render("ALT: " + recv[2], True, (0, 0, 255))
                    bea = smallFont.render("BEA: " + recv[3], True, (0, 0, 255))
                    spe = smallFont.render("SPE: " + recv[4], True, (0, 0, 255))
                    lat = float(recv[0])
                    lon = float(recv[1])
                    speed = float(recv[4])
                    pygame.draw.rect(screen,redColor,(156,116,10,10))
                    if start < 4:
                       A = lat
                       B = lon
                       pygame.draw.rect(screen, redColor,Rect((width/2)- 4,(height/2) - 4,100,50),2)
                       start += 1
                    if start == 4:
                       #now = datetime.datetime.now()
                       #timestamp = now.strftime("%y/%m/%d-%H:%M:%S")
                       #timp = "TIME:, " + str(timestamp) + ", LAT:, " + str(lat) + ", LON:, " + str(lon) + ", SPEED:, " + str(speed) + ", ANGLE:, " + str(angle)  + "\n"
                       #with open("/run/shm/log.txt", "a") as file:
                       #   file.write(timp)
                       start +=1
                    else:
                       start += 1
                       X = lat
                       Y = lon
                       pygame.draw.rect(screen, redColor,Rect(((A-X)* J)+(width/2),((B-Y)*J)+ (height/2),10,10),2)
                       #pygame.display.update()
                       time.sleep(.25)
                       pygame.draw.rect(screen, yellowColor,Rect(((A-X)* J)+(width/2),((B-Y)*J)+ (height/2),10,10),2)
                       if oldX != 0:
                          pygame.draw.line(screen, yellowColor,  (((A-oldX)* J)+(width/2),((B-oldY)*J)+ (height/2)), (((A-X)* J)+(width/2),((B-Y)*J)+ (height/2)))

                    #screen.fill((255, 149, 0))
                    screen.blit(clock, (190, 205))
                    #screen.blit(exit_button, (10,210))
                    #screen.blit(cam_button, (10,5))
                    #screen.blit(temp, (5, 50))
                    screen.blit(lattitude, (5, 90))
                    screen.blit(longitude, (5, 110))
                    screen.blit(alt, (5, 130))
                    screen.blit(bea, (5, 150))
                    screen.blit(spe, (5, 170))

                    #screen.blit(thermometer, (190,200))
                    #screen.blit(showBPM, (10,210))

                    if connected == True:
                       screen.blit(select_button, (5, 10))

                    frame1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2RGB)
                    frame1 =  np.rot90(frame1)
                    frame1 = cv2.flip(frame1, 0)
                    frame1 = pygame.surfarray.make_surface(frame1)
                    screen.blit(frame1, (80,5), (0, 0, 240, 160))
                    #screen.blit(image, (200, 300), (640,512,200,200))

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
                #yield (b'--frame\r\n'b'Content-Type: image/jpeg\r\n\r\n' + jpg + b'\r\n\r\n')

while True:
	threadVideoGet()

