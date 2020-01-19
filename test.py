import picamera
import time
import board
import busio
import adafruit_lsm9ds1
from PIL import Image

# I2C connection:
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)

camera = picamera.PiCamera()
camera.resolution = (640, 480)
camera.start_recording('my_video.h264')

img = Image.open('overlay.png')
pad = Image.new('RGB', (((img.size[0] + 31) // 32) * 32, ((img.size[1] + 15) // 16) * 16,))
pad.paste(img, (0, 0))
o = camera.add_overlay(pad.tobytes(), size = img.size)
o.alpha = 128
o.layer = 3

while True:

    # Read acceleration, magnetometer, gyroscope, temperature.
    accel_x, accel_y, accel_z = sensor.acceleration
    mag_x, mag_y, mag_z = sensor.magnetic
    gyro_x, gyro_y, gyro_z = sensor.gyro
    temp = sensor.temperature
    camera.annotate_text = str(accel_x)
    # Print values.
    print('Acceleration (m/s^2): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(accel_x, accel_y, accel_z))
    print('Magnetometer (gauss): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(mag_x, mag_y, mag_z))
    print('Gyroscope (degrees/sec): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(gyro_x, gyro_y, gyro_z))
    print('Temperature: {0:0.3f}C'.format(temp))
    # Delay for a second.
    time.sleep(1.0)


camera.stop_recording()
