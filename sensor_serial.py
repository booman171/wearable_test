import threading
import time
import serial
from pulsesensor import Pulsesensor

class Sensorserial:
   def __init__(self):
      self.temperature = 0
      self.ecg = 0
      self.pitch = 0
      self.roll = 0
      self.yaw = 0

   def read_from_port(self):
      ser =  serial.Serial('/dev/ttyACM0', 115200, timeout=0)
      while True:
         if(ser.inWaiting()>0):
            ser_bytes = ser.read(ser.inWaiting()).decode('ascii')
            message = str(ser_bytes)
            sensors = message.split(",")
            self.temperature = sensors[len(sensors)-2]
            self.ecg = sensors[len(sensors)-1]
            selfpitch = sensors[len(sensors)-5]
            self.roll = sensors[len(sensors)-4]
            self.yaw = sensors[len(sensors)-3]
         time.sleep(0.01)
   def getECG(self):
      return  float(self.ecg)

   def getTemp(self):
      return float(self.temperature)

   def getPitch(self):
      return float(self.pitch)

   def getRoll(self):
      return float(self.roll)

   def getYaw(self):
      return float(self.yaw)

   def startSerialRead(self):
      self.thread = threading.Thread(target=self.read_from_port)
      self.thread.stopped = False
      self.thread.start()
      return

   def stopSerialRead(self):
      self.thread.stopped = True
      return



s = Sensorserial()
s.startSerialRead()
p = Pulsesensor()
p.startAsyncBPM()

try:
   while True:
      p.ecg_signal = s.getECG()
      #print(p.ecg_signal)
      bpm = p.BPM
      if bpm > 0:
         print("BPM: %d" % bpm)
      else:
         print("No Heartbeat found")
      time.sleep(0.1)
except:
   s.stopSerialRead()
