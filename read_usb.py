import time
import serial
import threading

class readUSB:
	def __init__(self, port, baud):
		self.port = "/dev/" + port
		self.baud = baud
		self.ser = serial.Serial(self.port, self.baud, timeout=0)
		self.data = 0
		self.message = ""

	def getData(self):
		return self.message

	def readPort(self):
		#while not self.thread.stopped:
		while True:
			#print("ffsdvfs")
			#self.ser = serial.Serial("/dev/" + self.port, self.baud, timeout=0)
			if(self.ser.inWaiting()>0):
				ser_bytes = self.ser.read(self.ser.inWaiting()).decode('ascii')
				self.message = str(ser_bytes)
				#self.data = message.split(",")
				#self.temperature = sensors[len(sensors)-2]
			time.sleep(0.01)

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

arduino1 = readUSB("ttyACM0", 9600)
arduino1.startSensorRead()

try:
	while True:
		print(arduino1.getData())
except:
	arduino1.stopSensorRead()
