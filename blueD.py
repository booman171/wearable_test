# Importing the Bluetooth Socket library
import bluetooth
# Importing the GPIO library to use the GPIO pins of Raspberry pi
import RPi.GPIO as GPIO
import datetime
from datetime import timedelta
import threading
import time


host = ""
port = 1    # Raspberry Pi uses port 1 for Bluetooth Communication
# Creaitng Socket Bluetooth RFCOMM communication
server = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
print('Bluetooth Socket Created')

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
            while True:
                    # Receivng the data.
                    data = client.recv(1024) # 1024 is the buffer size.
                    data = str(data,"utf-8")
                    print(data)

                    send_data = "received\r\n"
                    client.send(send_data)
def main():
    sch = Schedule()

main()
