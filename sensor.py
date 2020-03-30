# extended from https://github.com/WorldFamousElectronics/PulseSensor_Amped_Arduino
import time
import threading
import serial

class Sensor:
    def __init__(self):
        self.BPM = 0
        self.temperature = 0
        self.ecg = 0
        self.pitch = 0
        self.roll = 0
        self.yaw = 0

    def getBPM(self):
        return self.BPM

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

    def read_from_port(self):
        # init variables
        ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0)
        rate = [0] * 10         # array to hold last 10 IBI values
        sampleCounter = 0       # used to determine pulse timing
        lastBeatTime = 0        # used to find IBI
        P = 512                 # used to find peak in pulse wave, seeded
        T = 512                 # used to find trough in pulse wave, seeded
        thresh = 525            # used to find instant moment of heart beat, seeded
        amp = 100               # used to hold amplitude of pulse waveform, seeded
        firstBeat = True        # used to seed rate array so we startup with reasonable BPM
        secondBeat = False      # used to seed rate array so we startup with reasonable BPM

        IBI = 600               # int that holds the time interval between beats! Must be seeded!
        Pulse = False           # "True" when User's live heartbeat is detected. "False" when not a "live beat". 
        lastTime = int(time.time()*1000)

        #while not self.thread.stopped:
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
            
    # Start getBPMLoop routine which saves the BPM in its variable
    def startSensorRead(self):
        self.thread = threading.Thread(target=self.read_from_port)
        self.thread.stopped = False
        self.thread.start()
        return

    # Stop the routine
    def stopSensorRead(self):
        self.thread.stopped = True
        self.BPM = 0
        return


s = Sensor()
s.startSensorRead()

try:
   while True:
      signal = s.getECG()
      print(signal)
      bpm = s.BPM
      if bpm > 0:
         print("BPM: %d" % bpm)
      #else:
         #print("No Heartbeat found")
      time.sleep(0.1)
except:
   s.stopSensorRead()
