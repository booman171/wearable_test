import picamera,os,time
from time import sleep
import datetime as dt
import board
import busio
import adafruit_lsm9ds1

#I@C Connection:
i2c = busio,I2C(board.SCL, board.SDA)
sensor = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)

var = 1

while var == 1 :
    camera = picamera.PiCamera()
    try:
		#IMU
		accel_x, accel_y, accel_z = sensor.acceleration
		mag_x, mag_y, mag_z = sensor.magnetic
		gyro_x, gyro_y, gyro_z = sensor.gyro
		temp = sensor.temperature
		print('Acceleration (m/s^2): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(accel_x, accel_y, accel_z))
		print('Magnetometer (gauss): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(mag_x, mag_y, mag_z))
		print('Gyroscope (degrees/sec): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(gyro_x, gyro_y, gyro_z))
		print('Temperature: {0:0.3f}C'.format(temp))

		#Video
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
