import picamera,os,time
from time import sleep
import datetime as dt

var = 1

while var == 1 :
    camera = picamera.PiCamera()
    try:
        print()
        #list = os.listdir("/home/pi/Desktop/Videos")
        #list.sort()
        #print(list)
        #os.remove("/home/pi/Desktop/Videos/"+list[0])
        #list1 = os.listdir("/home/pi/Desktop/Videos")
        #print(list1)
        fileTime = str(time.time())
        fileName = "video.h264"
	print("Newest File Is: " + fileTime)
        camera.resolution = (1280, 720)

        #camera.sensor_mode = 1 #closer to 1 lighter video is
        camera.start_recording(fileName)
        start = dt.datetime.now()

	while (dt.datetime.now() - start).seconds < 600 : #600 10 mins
            camera.annotate_text = dt.datetime.now().strftime('%d-%m-%Y %H:%M:%S')
            camera.wait_recording(0.2)
        camera.stop_recording()

    finally:
        camera.close()
