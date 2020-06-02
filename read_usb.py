import time
import serial
import threading
import os

class readUSB:
	def __init__(self, port, baud):
		self.port = "/dev/" + port
		self.baud = baud
		self.ser = ""
		self.data = 0
		self.message = ""
		self.check_usb = True
	def getData(self):
		return self.message

	def readPort(self):
		#while not self.thread.stopped:
		while True:
			#print("ffsdvfs")
			#self.ser = serial.Serial("/dev/" + self.port, self.baud, timeout=0)
			if os.path.exists('/dev/ttyACM0') == True:
				if self.check_usb == True:
					self.ser = serial.Serial(self.port, self.baud, timeout=0)
					self.check_usb = False
				if(self.ser.inWaiting()>0):
					ser_bytes = self.ser.read(self.ser.inWaiting()).decode('ascii')
					self.message = str(ser_bytes)
					self.message = self.message.replace("\r","")
					self.message = self.message.replace("\n","")
					#self.sensors = self.message.split(",")
					#self.data = message.split(",")
					#self.temperature = sensors[len(sensors)-2]
                #comment
				time.sleep(0.01)
			if  os.path.exists(self.port) == False:
				self.message = "-,-"
				self.check_usb = True
	# Start getBPMLoop routine which saves the BPM in its variable
	def startSensorRead(self):
		self.thread = threading.Thread(target=self.readPort)
		self.thread.stopped = False
		self.thread.start()
		return

	# Stop the routine
	def stopSensorRead(self):
		self.thread.stopped = True
		return

'''
arduino1 = readUSB("ttyACM0", 9600)
arduino1.startSensorRead()

while True:
	print(arduino1.getData())
'''
