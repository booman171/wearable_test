from __future__ import division
from __future__ import print_function
from __future__ import absolute_import

import threading
import os
import numpy as np
#import cv2
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
from bluetooth import *
#from streamer import StreamingOutput, StreamingHandler, StreamingServer
import argparse
#import os
#import cv2
#from get_video import VideoGet
from flask import Flask, render_template, Response
import importlib.util
from read_usb import readUSB
from pulseoximeter import MAXREFDES117
import remi.gui as gui
from remi import start, App
import multiprocessing

#from testWeb import MyApp
#from sensor_serial import read_from_port
#from metawear import MWBoard

#os.putenv('SDL_MOUSEDEV', '/dev/input/mouse')
os.putenv('SDL_MOUSEDEV', '/dev/input/mouse0')
os.putenv('SDL_FBDEV', '/dev/fb1')
pygame.init()
pygame.display.init()
#time.sleep(1)
pygame.mouse.set_visible(False)

#pygame.display.set_caption("OpenCV camera stream on Pygame")
screen = pygame.display.set_mode((240,320))
font = pygame.font.SysFont("comicsansms", 72)
text = font.render("Hello, World", True, (0, 128, 0))
screen.fill((255,0,0))
pygame.display.update()

#os.putenv('SDL_FBDEV', '/dev/fb1')
os.system('modprobe w1-gpio')
os.system('modprobe w1-therm')

#host = ""
#port = 1    # Raspberry Pi uses port 1 for Bluetooth Communication
# Creaitng Socket Bluetooth RFCOMM communication
#server = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
#print('Bluetooth Socket Created')
# Setup for physical buttons
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
#overlay_t = cv2.imread('therm.png', -1)
#thermometer = pygame.image.load('therm.png')
#thermometer = pygame.transform.rotozoom(thermometer, 0, 0.08)

#firebase = firebase.FirebaseApplication('https://wear1-38901.firebaseio.com/')'

#global connected
#connected = False


global send
send = False
host = ""
#port = 1    # Raspberry Pi uses port 1 for Bluetooth Communication
# Creaitng Socket Bluetooth RFCOMM communication

#server = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
print('Bluetooth Socket Created')
global connected, client, recv
connected = False

server =  BluetoothSocket(RFCOMM)
server.bind(("", PORT_ANY))
server.listen(1)
port = server.getsockname()[1]
uuid = "94f39d29-7d6d-437d-973b-fba39e49d4ee"
advertise_service( server, "roboserver-bt",
                        service_id = uuid,
                        service_classes = [uuid, SERIAL_PORT_CLASS],
                        profiles = [SERIAL_PORT_PROFILE] )
client, client_info = server.accept()
connected = True
print("Connected")

def recvMSG():
   global recv
   while True:
      # Receivng the data.
      data = client.recv(1024) # 1024 is the buffer size.
      data = str(data,"utf-8")
      #print("Received: " + data)
      recv = data.split(",")
      #recv = data

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
if connected:
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

print("HERE")

ds_factor = 0.6
# Initialize video stream
# videostream = VideoStream(resolution=(imW,imH),framerate=30).start()
#time.sleep(1)

#global frame

#Thread for getting video from get_video

#video_getter = VideoGet(source).start()

global menu_key
menu_key = 1
record = False
recording = False
width = 240
height = 320

blackColor =  pygame.Color(  0,  0, 0)
yellowColor = pygame.Color(255,255, 0)
redColor =    pygame.Color(255,  0, 0)
greenColor =  pygame.Color(  0,255, 0)
rec_color = pygame.Color(255, 204, 51)

alpha = 0.4
oldX = 0
oldY = 0

frames = []

check_usb = True
var_set = False

#global bpm, emg, accelX, accelY, accelZ, tempF, hr, spo2

bpm = ""
emg = ""
accelX = ""
accelY = ""
accelZ = ""
tempF = ""
leftButton = ""
rightButton = ""
myoHeight = 0
# set SCALE
J = 100000
#start = 0
hr = 0
spo2 = 0
maxrefdes = MAXREFDES117()

class MyApp(App):
    #global bpm, emg, accelX, accelY, accelZ, tempF, hr, spo2
    def __init__(self, *args):
        super(MyApp, self).__init__(*args)

    def idle(self):
        self.emgLabel.set_text("EMG: " + emg)
        self.tempLabel.set_text("Temp: " + tempF)
        self.hrLabel.set_text("HR: " + str(hr))
        self.spo2Label.set_text("SpO2: " + str(spo2))
        self.xLabel.set_text("Accel-X: " + accelX)
        self.yLabel.set_text("Accel-Y: " + accelY)
        self.zLabel.set_text("Accel-Z: " + accelZ)

    def setVals(self, emg, temp, hr, spo2, x, y, z):
        self.emg = emg
        self.temp = temp
        self.hr = hr
        self.spo2 = spo2
        self.x = x
        self.y = y
        self.z = z

    def main(self):
        while True:
           print("Inside")
           global bpm, emg, accelX, accelY, accelZ, tempF, hr, spo2, leftButton, rightButton, data
           container = gui.VBox(width=500, height=200)
           self.lbl = gui.Label('Hello world!')
           self.emgLabel = gui.Label("EMG: ")
           self.tempLabel = gui.Label("Temp: ")
           self.hrLabel = gui.Label("HR: ")
           self.spo2Label = gui.Label("SpO2: ")
           self.xLabel = gui.Label("Accel-X: ")
           self.yLabel = gui.Label("Accel-Y: ")
           self.zLabel = gui.Label("Accel-Z: ")
           self.bt = gui.Button('Press me!')

           self.update = True
           # setting the listener for the onclick event of the button
           self.bt.onclick.do(self.on_button_pressed)

           # appending a widget to another, the first argument is a string key
           container.append(self.lbl)
           container.append(self.emgLabel)
           container.append(self.tempLabel)
           container.append(self.hrLabel)
           container.append(self.spo2Label)
           container.append(self.xLabel)
           container.append(self.yLabel)
           container.append(self.zLabel)
           container.append(self.bt)

           # returning the root widget
           return container


    # listener function
    def on_button_pressed(self, widget):
        self.lbl.set_text('Button pressed!')
        self.bt.set_text('Hi!')
        f = open("data.csv")
        while True:
           line = f.readline()
           data = line.split(",")
           #print(len(data))
           if len(data) == 8:
              emg = data[1]
              hr = data[2]
              spo2 = data[3]
              accelX = data[4]
              accelY = data[5]
              accelZ = data[6]
              tempF = data[7]
              #return deta

def startWeb():
   start(MyApp, address='192.168.1.11', port=8040, multiple_instance=False, enable_file_cache=True, update_interval=0.1, start_browser=True)

def csv_writer(file_name, title, write):
   with open(file_name, 'w', newline='')as csv_file:
       writer = csv.writer(csv_file)
       writer.writerow(title)
       writer.writerow(write)

# Set time at start of program
#start = time.monotonic()
#timestr = time.strftime("%Y%m%d-%H%M%S")
# Set declination based on location http://www.magnetic-declination.com/
#declination = -0.106988683
# Set filename video.avi for recording camera output
#data_row = datetime.now() + "," + emg + "," + accelX + "," + accelY + "," + accelZ + "," + tempF
#csv_writer(filename, "emg,accel-x,accel-y,accel-z,temp", data_row)

timestr = time.strftime("%Y%m%d-%H%M%S")
# Set declination based on location http://www.magnetic-declination.com/
#declination = -0.106988683
# Set filename video.avi for recording camera output
file_name = "data.csv"
#file_name = timestr + '.csv' # .avi .mp4
file = open(file_name, 'w', newline='')
file.write("time,emg,hr,spo2,accel-x,accel-y,accel-z,temp" + "\n")
file.close()

#fileName = "data.csv"
#dataFile = open(fileName, 'w', newline='')
#dataFile.write("time,emg,hr,spo2,accel-x,accel-y,accel-z,temp" + "\n")
#dataFile.close()

numbers = 3
#q = multiprocessing.Queue()
#webThread = threading.Thread(target=startWeb)
#webThread.start()

#print("vfvdsvdF")
#video_getter = VideoGet(0).start()
#cps = CountsPerSec().start()
while True:
        #frame = video_getter.frame
        #frame1 = frame.copy()

        #frame1=cv2.resize(frame1,None,fx=ds_factor,fy=ds_factor,interpolation=cv2.INTER_AREA)
        #gray=cv2.cvtColor(frame1,cv2.COLOR_BGR2GRAY)
        #ret, jpeg = cv2.imencode('.jpg', frame1)
        #cv2.imshow("Video", frame1)
        #jpg = jpeg.tobytes()
        #print(q.get())
        data = arduino1.getData().split(",")
        if len(arduino1.getData()) >= 7:
           #bpm = "BPM: " + data[0]
           #tempF = "Temp: " + data[1][0:3] + " F"
           emg = data[0]
           accelX = data[1]
           accelY = data[2]
           accelZ = data[3]
           tempF = data[4]
           leftButton = data[5]
           rightButton = data[6]
           
           data_row = str(time.monotonic()) + "," + emg + "," + str(hr) + "," + str(spo2) + "," + accelX + "," + accelY + "," + accelZ + "," + tempF
           file = open(file_name, "a")
           file.write(data_row)
           #file.write("\n")
           #dataFile = open(fileName, "w")
           #dataFile.write(data_row)
           #dataFile.write("\n")


        #start(MyApp, address='192.168.1.11', port=8040, multiple_instance=False, enable_file_cache=True, update_interval=0.1, start_browser=True)

        if main == True:
            now = datetime.now()
            #start(MyApp, address='192.168.1.11', port=8081, multiple_instance=False, enable_file_cache=True, update_interval=0.1, start_browser=True)
            #ret, frame = cap.read()
            #cam_out = frame.copy()
            #output = frame.copy()
            #maxrefdes = MAXREFDES117()
            screen.fill([0,0,0])

            pygame.draw.rect(screen, (194, 156, 43), (0,0,240,320), 5)
            bigFont = pygame.font.SysFont(None, 66)
            medFont = pygame.font.SysFont(None, 32)
            smallFont = pygame.font.SysFont(None, 18)
            clock = medFont.render(now.strftime("%H:%M:%S"), True, (18, 122, 23))

            emgText = medFont.render(emg, True, (18, 122, 23))
            xText = medFont.render("Accel-X: " + accelX, True, (18, 122, 23))
            yText = medFont.render("Accel-Y: " + accelY, True, (18, 122, 23))
            zText = medFont.render("Accel-Z: " + accelZ, True, (18, 122, 23))
            tempText = medFont.render("Temp: " + tempF + "F", True, (18, 122, 23))
            leftText = medFont.render("CPX-Left: " + leftButton, True, (18, 122, 23))
            rightText = medFont.render("CPX-Right: " + rightButton, True, (18, 122, 23))
            bloodText = medFont.render("HR: " + str(hr) + ", SpO2: " + str(spo2), True, (138, 3, 3))
            up_button = medFont.render("/\\", True,(0, 0, 255))
            down_button = medFont.render("\\/", True, (0, 0, 255))
            #select_button = medFont.render(menu_items[key], True, (18, 122, 23))

            
            #rec_button  = medFont.render("REC", True, rec_color)
            #exit_button = medFont.render("Exit", True, (0, 255, 0)
            '''
            try:
               if recv is not None:
                  print(recv)
                  if len(recv) > 1:
                     act = medFont.render(recv[0], True, (255, 204, 51))
                     screen.blit(act, (10, 125))
                  if len(recv) > 2:
                     timer = bigFont.render(recv[1], True, (255, 204, 51))
                     screen.blit(timer, (10, 80))
            except NameError:
               print("recv not defined")
            '''
            #lat = float(recv[0])
            #lon = float(recv[1])
            #speed = float(recv[4])

            #screen.fill((255, 149, 0))
            screen.blit(clock, (10, 5))

            emgLabel = medFont.render("EMG", True, (18, 122, 23))
            screen.blit(emgText, (180, 260))
            screen.blit(emgLabel, (180, 290))
            screen.blit(xText, (10, 80))
            screen.blit(yText, (10, 120))
            screen.blit(zText, (10, 160))
            screen.blit(tempText, (10, 200))
            screen.blit(leftText, (10, 240))
            screen.blit(rightText, (10, 280))
            screen.blit(bloodText, (10, 50))

            #print(emg)
            if len(emg) > 1:
               myoHeight = int(float(emg)) * 40
            pygame.draw.rect(screen, (18, 122, 23), (189,250,17,-170), 2)
            pygame.draw.rect(screen, (194, 156, 43), (190,249,15,-myoHeight), 0)
            #screen.blit(rec_button, (265, 210))
            #screen.blit(exit_button, (10,210))
            #screen.blit(cam_button, (10,5))
            #screen.blit(lattitude, (5, 90))
            #screen.blit(longitude, (5, 110))
            #screen.blit(alt, (5, 130))
            #screen.blit(bea, (5, 150))

            #cv2.putText(cam_out, now.strftime("%H:%M:%S"),(10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 3)

            #if os.path.exists('/dev/ttyACM0') == True and var_set == True:
            #temp = medFont.render(tempF, True, (18, 122, 23))
            #showBPM = medFont.render(bpm, True, (18, 122, 23))
            #screen.blit(temp, (5, 210))
            #screen.blit(showBPM, (5, 180))
            #cv2.putText(cam_out, tempF,(440, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 0), 3)
            #cv2.putText(cam_out, bpm,(10, 350), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)

            # apply the overlay
            #cv2.addWeighted(cam_out, alpha, output, 1 - alpha, 0, output)
            #frames.append(output)

            #if connected == True:
            #   screen.blit(select_button, (5, 10))

            #frame1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2RGB)
            #frame1 =  np.rot90(frame1)
            #frame1 = cv2.flip(frame1, 0)

            
            # Displays live camera output on screen
            #frame1 = pygame.surfarray.make_surface(frame1)
            #screen.blit(frame1, (80,5), (0, 0, 240, 160))
            
            #screen.blit(image, (200, 300), (640,512,200,200))

            #screen.blit(thermometer, (190,200))
            #screen.blit(showBPM, (10,210))

            pygame.display.update()

            if GPIO.input(17) == False:
                hr, spo2 = maxrefdes.get()
                print(hr, spo2)

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
file.close()
