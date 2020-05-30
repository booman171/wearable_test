import cv2
from read_usb import readUSB

capture = cv2.VideoCapture(0)
frame_width = int(capture.get(3))
frame_height = int(capture.get(4))

print("Height: " + str( frame_height) + ", Width: " + str(frame_width) + ", Frame Rate: " + str(capture.get(cv2.CAP_PROP_FPS)))


arduino1 = readUSB("ttyACM0", 9600)
arduino1.startSensorRead()


while True:
   data = arduino1.getData().split(",")
   print(data)
   #temperature = arduino1.getData()[1]
   #bpm = "BPM: " + arduino1.getData()[0]
   #tempF = "Temp: " + temperature + "F"
