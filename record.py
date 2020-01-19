import picamera, os, time
from time import sleep
import datetime as dt

var = 1

while var == 1:
	camera = picamera.PiCamera()
	try:
		print()
		fileTime = str(time.time())
		fileName = "video.h264"
		print("Newest File Is: " + fileTime)
