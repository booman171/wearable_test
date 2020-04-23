# Importing the Bluetooth Socket library
import bluetooth
# Importing the GPIO library to use the GPIO pins of Raspberry pi
import RPi.GPIO as GPIO
import datetime
from datetime import timedelta
import threading
import time


led_pin = 16    # Initializing pin 40 for led
fan_pin = 13
GPIO.setmode(GPIO.BCM)  # Using BCM numbering
GPIO.setup(led_pin, GPIO.OUT)   # Declaring the pin 40 as output pin
GPIO.setup(fan_pin, GPIO.OUT)
host = ""
port = 1    # Raspberry Pi uses port 1 for Bluetooth Communication
# Creaitng Socket Bluetooth RFCOMM communication
server = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
print('Bluetooth Socket Created')
interval = 15
duration = 5  #in seconds
nextShoot = datetime.datetime.now()+timedelta(minutes=interval);
schedule_running = False

def sch_timer(interval):
    print("New thread created")
    global schedule_running
    schedule_running = True
    nextShoot = datetime.datetime.now() + timedelta(minutes=interval)
    print("The next shoot will happen : "+str(nextShoot))
    while schedule_running == True:
        if datetime.datetime.now() > nextShoot:
                print("Schedule shoot!!" + str(datetime.datetime.now()))
                GPIO.output(led_pin, True)
                #print("LED will be up until: "+str(datetime.datetime.now() + timedelta(minutes=duration)))
                print(str(datetime.datetime.now()))
                print(str(datetime.datetime.now() + timedelta(minutes=duration)))
                time.sleep(duration)
                GPIO.output(led_pin, False)
                print("duration has expired, new scheduled shoot is at: " + str(datetime.datetime.now() + timedelta(minutes=interval)))
                nextShoot = datetime.datetime.now() + timedelta(minutes=interval)
    print("End of thread")      
    return

class Schedule():
    def __init__(self):
            try:  #to connect
                    server.bind((host, port))
                    print("Bluetooth Binding Completed")
            except:
                    print("Bluetooth Binding Failed")
            server.listen(1) # One connection at a time
            # Server accepts the clients request and assigns a mac address.
            client, address = server.accept()
            print("Connected To", address)
            print("Client:", client)
            t1 = threading.Thread()
        #   try:
            while True:
                    # Receivng the data.
                    data = client.recv(1024) # 1024 is the buffer size.
                    data = str(data,"utf-8")
                    print(data)

                    send_data = "received\r\n"
                    # Sending the data.
                    #TODO Maybe encode data to UTF-8 before sending
                    client.send(send_data)
            #~ except:
                    #~ # Making all the output pins LOW
                    #~ GPIO.cleanup()
                    #~ # Closing the client and server connection
                    #~ client.close()
                    #~ server.close()

def main():
    sch = Schedule()

main()
