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
start = time.monotonic()
timestr = time.strftime("%Y%m%d-%H%M%S")
# Set declination based on location http://www.magnetic-declination.com/
declination = -0.106988683
# Set filename video.avi for recording camera output
filename = 'video' + timestr + '.avi' # .avi .mp4
# Set picamera standard frame rate
fps = 30.0
# Set recording resolution
#capture = cv2.VideoCapture(0)
#frame_width = int(capture.get(3))
#frame_height = int(capture.get(4))
# Read from default (0) camera
#cap = cv2.VideoCapture(-1)
# Set video format
video_writer = cv2.VideoWriter_fourcc('M','J','P','G')
video_out = cv2.VideoWriter(filename, video_writer, fps, (640, 480))
# Save video.avi to current directory
save_path = os.path.join(filename)

# Read in transport image of thermostat icon
overlay_t = cv2.imread('therm.png', -1)
thermometer = pygame.image.load('therm.png')
thermometer = pygame.transform.rotozoom(thermometer, 0, 0.08)

#firebase = firebase.FirebaseApplication('https://wear1-38901.firebaseio.com/')

'''
Tensorflow Code

# Define VideoStream class to handle streaming of video from webcam in separate processing thread
# Source - Adrian Rosebrock, PyImageSearch: https://www.pyimagesearch.com/2015/12/28/increasing-raspberry-pi-fps-with-python-and-opencv/
class VideoStream:
    """Camera object that controls video streaming from the Picamera"""
    def __init__(self,resolution=(640,480),framerate=30):
        # Initialize the PiCamera and the camera image stream
        self.stream = cv2.VideoCapture(0)
        ret = self.stream.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        ret = self.stream.set(3,resolution[0])
        ret = self.stream.set(4,resolution[1])
            
        # Read first frame from the stream
        (self.grabbed, self.frame) = self.stream.read()

	# Variable to control when the camera is stopped
        self.stopped = False

    def start(self):
	# Start the thread that reads frames from the video stream
        Thread(target=self.update,args=()).start()
        return self

    def update(self):
        # Keep looping indefinitely until the thread is stopped
        while True:
            # If the camera is stopped, stop the thread
            if self.stopped:
                # Close camera resources
                self.stream.release()
                return

            # Otherwise, grab the next frame from the stream
            (self.grabbed, self.frame) = self.stream.read()

    def read(self):
	# Return the most recent frame
        return self.frame

    def stop(self):
	# Indicate that the camera and thread should be stopped
        self.stopped = True

# Define and parse input arguments
parser = argparse.ArgumentParser()
parser.add_argument('--modeldir', help='Folder the .tflite file is located in',
                    required=True)
parser.add_argument('--graph', help='Name of the .tflite file, if different than detect.tflite',
                    default='detect.tflite')
parser.add_argument('--labels', help='Name of the labelmap file, if different than labelmap.txt',
                    default='labelmap.txt')
parser.add_argument('--threshold', help='Minimum confidence threshold for displaying detected objects',
                    default=0.5)
parser.add_argument('--resolution', help='Desired webcam resolution in WxH. If the webcam does not support the resolution entered, errors may occur.',
                    default='1280x720')
parser.add_argument('--edgetpu', help='Use Coral Edge TPU Accelerator to speed up detection',
                    action='store_true')

args = parser.parse_args()

MODEL_NAME = args.modeldir
GRAPH_NAME = args.graph
LABELMAP_NAME = args.labels
min_conf_threshold = float(args.threshold)
resW, resH = args.resolution.split('x')
imW, imH = int(resW), int(resH)
use_TPU = args.edgetpu

# Import TensorFlow libraries
# If tensorflow is not installed, import interpreter from tflite_runtime, else import from regular tensorflow
# If using Coral Edge TPU, import the load_delegate library
pkg = importlib.util.find_spec('tensorflow')
if pkg is None:
    from tflite_runtime.interpreter import Interpreter
    if use_TPU:
        from tflite_runtime.interpreter import load_delegate
else:
    from tensorflow.lite.python.interpreter import Interpreter
    if use_TPU:
        from tensorflow.lite.python.interpreter import load_delegate

# If using Edge TPU, assign filename for Edge TPU model
if use_TPU:
    # If user has specified the name of the .tflite file, use that name, otherwise use default 'edgetpu.tflite'
    if (GRAPH_NAME == 'detect.tflite'):
        GRAPH_NAME = 'edgetpu.tflite'       

# Get path to current working directory
CWD_PATH = os.getcwd()

# Path to .tflite file, which contains the model that is used for object detection
PATH_TO_CKPT = os.path.join(CWD_PATH,MODEL_NAME,GRAPH_NAME)

# Path to label map file
PATH_TO_LABELS = os.path.join(CWD_PATH,MODEL_NAME,LABELMAP_NAME)

# Load the label map
with open(PATH_TO_LABELS, 'r') as f:
    labels = [line.strip() for line in f.readlines()]

# Have to do a weird fix for label map if using the COCO "starter model" from
# https://www.tensorflow.org/lite/models/object_detection/overview
# First label is '???', which has to be removed.
if labels[0] == '???':
    del(labels[0])

# Load the Tensorflow Lite model.
# If using Edge TPU, use special load_delegate argument
if use_TPU:
    interpreter = Interpreter(model_path=PATH_TO_CKPT,
                              experimental_delegates=[load_delegate('libedgetpu.so.1.0')])
    print(PATH_TO_CKPT)
else:
    interpreter = Interpreter(model_path=PATH_TO_CKPT)

interpreter.allocate_tensors()

# Get model details
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()
tf_height = input_details[0]['shape'][1]
tf_width = input_details[0]['shape'][2]

floating_model = (input_details[0]['dtype'] == np.float32)

input_mean = 127.5
input_std = 127.5

# Initialize frame rate calculation
frame_rate_calc = 1
freq = cv2.getTickFrequency()


End Tensorflow Code
'''


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

showSensors = False
#ser.readline()
WHITE = (255, 255, 255)
#mwboard = MWBoard()
menu_key = 1
menu_items = ["J", "Play/P", "Next", "Prev", "Vol+", "Vol-", "voice", "Back"]
temperature = 0

arduino1 = readUSB("ttyACM0", 9600)
arduino1.startSensorRead()

ds_factor = 0.6
# Initialize video stream
# videostream = VideoStream(resolution=(imW,imH),framerate=30).start()
time.sleep(1)

#global frame
def threadVideoGet(source=0):
        '''
        Thread for getting video from get_video
        '''
        video_getter = VideoGet(source).start()
        global menu_key
        global record
        global recording
        record = False
        recording = False
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
                frame1 = frame.copy()
                   
                frame1=cv2.resize(frame1,None,fx=ds_factor,fy=ds_factor,interpolation=cv2.INTER_AREA)
                gray=cv2.cvtColor(frame1,cv2.COLOR_BGR2GRAY)
                ret, jpeg = cv2.imencode('.jpg', frame1)
                #cv2.imshow("Video", frame1)
                jpg = jpeg.tobytes()

                temperature = arduino1.getData()
                tempF = "Temp: " + temperature

                '''
                # Start timer (for calculating frame1 rate)
                t1 = cv2.getTickCount()

                frame1_rgb = cv2.cvtColor(frame1, cv2.COLOR_BGR2RGB)
                frame1_resized = cv2.resize(frame1_rgb, (tf_width, tf_height))
                input_data = np.expand_dims(frame1_resized, axis=0)
                
                # Normalize pixel values if using a floating model (i.e. if model is non-quantized)
                if floating_model:
                    input_data = (np.float32(input_data) - input_mean) / input_std

                # Perform the actual detection by running the model with the image as input
                interpreter.set_tensor(input_details[0]['index'],input_data)
                interpreter.invoke()

                # Retrieve detection results
                boxes = interpreter.get_tensor(output_details[0]['index'])[0] # Bounding box coordinates of detected objects
                classes = interpreter.get_tensor(output_details[1]['index'])[0] # Class index of detected objects
                scores = interpreter.get_tensor(output_details[2]['index'])[0] # Confidence of detected objects
                #num = interpreter.get_tensor(output_details[3]['index'])[0]  # Total number of detected objects (inaccurate and not needed)

                # Loop over all detections and draw detection box if confidence is above minimum threshold
                for i in range(len(scores)):
                    if ((scores[i] > min_conf_threshold) and (scores[i] <= 1.0)):

                        # Get bounding box coordinates and draw box
                        # Interpreter can return coordinates that are outside of image dimensions, need to force them to be within image using max() and min()
                        ymin = int(max(1,(boxes[i][0] * imH)))
                        xmin = int(max(1,(boxes[i][1] * imW)))
                        ymax = int(min(imH,(boxes[i][2] * imH)))
                        xmax = int(min(imW,(boxes[i][3] * imW)))
            
                        cv2.rectangle(frame1, (xmin,ymin), (xmax,ymax), (10, 255, 0), 2)

                        # Draw label
                        object_name = labels[int(classes[i])] # Look up object name from "labels" array using class index
                        label = '%s: %d%%' % (object_name, int(scores[i]*100)) # Example: 'person: 72%'
                        labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2) # Get font size
                        label_ymin = max(ymin, labelSize[1] + 10) # Make sure not to draw label too close to top of window
                        cv2.rectangle(frame1, (xmin, label_ymin-labelSize[1]-10), (xmin+labelSize[0], label_ymin+baseLine-10), (255, 255, 255), cv2.FILLED) # Draw white box to put label text in
                        cv2.putText(frame1, label, (xmin, label_ymin-7), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2) # Draw label text

                # Draw framerate in corner of frame
                #cv2.putText(frame,'FPS: {0:.2f}'.format(frame_rate_calc),(30,50),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,0),2,cv2.LINE_AA)
                
                # Calculate framerate
                t2 = cv2.getTickCount()
                time1 = (t2-t1)/freq
                frame_rate_calc= 1/time1
                '''
                if main == True:
                    now = datetime.now()
                    #ret, frame = cap.read()
                    #frame1 = frame.copy()
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
                    temp = medFont.render(tempF + " C", True, (255, 0, 0))
                    #showBPM = medFont.render("BPM: " + bpm, True, (255, 0, 0))
                    if len(recv) > 1:
                       lattitude = smallFont.render("LAT: " + recv[0], True, (0, 0, 255))
                    if len(recv) > 2:
                       longitude = smallFont.render("LON: " + recv[1], True, (0, 0, 255))
                    if len(recv) > 3:
                       alt = smallFont.render("ALT: " + recv[2], True, (0, 0, 255))
                    if len(recv) > 4:
                       bea = smallFont.render("BEA: " + recv[3], True, (0, 0, 255))
                    if len(recv) > 5:
                       spe = smallFont.render("SPE: " + recv[4], True, (0, 0, 255))
                    #lat = float(recv[0])
                    #lon = float(recv[1])
                    #speed = float(recv[4])
                    '''
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
                    '''
                    
                    if record == True:
                       video_out.write(frame)
                   
                    #screen.fill((255, 149, 0))
                    screen.blit(clock, (190, 205))
                    #screen.blit(exit_button, (10,210))
                    #screen.blit(cam_button, (10,5))
                    screen.blit(temp, (5, 205))
                    screen.blit(lattitude, (5, 90))
                    screen.blit(longitude, (5, 110))
                    screen.blit(alt, (5, 130))
                    screen.blit(bea, (5, 150))
                    screen.blit(spe, (5, 170))

                    if connected == True:
                       screen.blit(select_button, (5, 10))

                    frame1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2RGB)
                    frame1 =  np.rot90(frame1)
                    frame1 = cv2.flip(frame1, 0)
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
                           record = False
                           video_out.release()
                        if record == False:
                           record = True
                        
                        
                #yield (b'--frame\r\n'b'Content-Type: image/jpeg\r\n\r\n' + jpg + b'\r\n\r\n')

while True:
	threadVideoGet()

