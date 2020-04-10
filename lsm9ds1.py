# Simple demo of the LSM9DS1 accelerometer, magnetometer, gyroscope.
# Will print the acceleration, magnetometer, and gyroscope values every second.
import time
import board
import busio
import adafruit_lsm9ds1
from firebase import firebase
# I2C connection:
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)
time.sleep(1)

# Main loop will read the acceleration, magnetometer, gyroscope, Temperature
# values every second and print them out.
while True:
    # Read acceleration, magnetometer, gyroscope, temperature.
    accel_x, accel_y, accel_z = sensor.acceleration
    mag_x, mag_y, mag_z = sensor.magnetic
    gyro_x, gyro_y, gyro_z = sensor.gyro
    temp = sensor.temperature

    #store the Host ID(provided in firebase database) in variable where you want to send the real time sensor data.  
    #firebase = firebase.FirebaseApplication('https://wear1-38901.firebaseio.com/')

    #store the readings in variable and convert it into string and using firbase.post then data will be posted to databse of firebase 
    #result = firebase.post('wear1', {'Accel X':str(accel_x),'Accel Y':str(accel_y), 'Accel Z':str(accel_z)}) 

    # Print values.
    print(
        "Acceleration (m/s^2): ({0:0.3f},{1:0.3f},{2:0.3f})".format(
            accel_x, accel_y, accel_z
        )
    )
    print(
        "Magnetometer (gauss): ({0:0.3f},{1:0.3f},{2:0.3f})".format(mag_x, mag_y, mag_z)
    )
    print(
        "Gyroscope (degrees/sec): ({0:0.3f},{1:0.3f},{2:0.3f})".format(
            gyro_x, gyro_y, gyro_z
        )
    )
    print("Temperature: {0:0.3f}C".format(temp))
    # Delay for a second.
    time.sleep(1.0)

