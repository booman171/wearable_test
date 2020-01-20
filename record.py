import picamera
from time import sleep
camera = picamera.PiCamera()
while True:
	camera.start_preview()
	camera.start_recording('my_video.h264')
	camera.wait_recording(10)
	camera.stop_recording()
	camera.stop_preview()
