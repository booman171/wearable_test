import picamera,os,time
from time import sleep
import datetime as dt
import board
import busio
import adafruit_lsm9ds1

#I@C Connection:
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)

var = 1

with picamera.PiCamera() as camera:
	camera.resolution = (640, 480)
	camera.start_recording('my_video.h264')
	#camera.wait_recording(60)
	camera.stop_recording()
	#IMU
	accel_x, accel_y, accel_z = sensor.acceleration
	mag_x, mag_y, mag_z = sensor.magnetic
	gyro_x, gyro_y, gyro_z = sensor.gyro
	temp = sensor.temperature
	print('Acceleration (m/s^2): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(accel_x, accel_y, accel_z))
	print('Magnetometer (gauss): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(mag_x, mag_y, mag_z))
	print('Gyroscope (degrees/sec): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(gyro_x, gyro_y, gyro_z))
	print('Temperature: {0:0.3f}C'.format(temp))
