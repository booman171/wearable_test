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
        #while not self.thread.stopped:
        while True:
            if(ser.inWaiting()>0):
               ser_bytes = ser.read(ser.inWaiting()).decode('ascii')
               message = str(ser_bytes)
               sensors = message.split(",")
               self.temperature = sensors[len(sensors)-2]
               self.ecg = sensors[len(sensors)-1]
               self.pitch = sensors[len(sensors)-5]
               self.roll = sensors[len(sensors)-4]
               self.yaw = sensors[len(sensors)-3]
               self.processECG(float(self.ecg))
            time.sleep(0.01)
            
    def processECG(self, signal):
        rate = [0] * 10         # array to hold last 10 IBI values
        sampleCounter = 0       # used to determine pulse timing
        lastBeatTime = 0        # used to find IBI
        P = 512                 # used to find peak in pulse wave, seeded
        T = 512                 # used to find trough in pulse wave, seeded
        thresh = 500            # used to find instant moment of heart beat, seeded
        amp = 100               # used to hold amplitude of pulse waveform, seeded
        firstBeat = True        # used to seed rate array so we startup with reasonable BPM
        secondBeat = False      # used to seed rate array so we startup with reasonable BPM
        IBI = 600               # int that holds the time interval between beats! Must be seeded!
        Pulse = False           # "True" when User's live heartbeat is detected. "False" when not a "live beat".
        lastTime = int(time.time()*1000)
        currentTime = int(time.time()*1000)
        #print(signal)
        sampleCounter += currentTime - lastTime
        lastTime = currentTime
        N = sampleCounter - lastBeatTime

        # find the peak and trough of the pulse wave
        if signal < thresh:     # avoid dichrotic noise by waiting 3/5 of last IBI   and N > (IBI/5.0)*3
            if signal < T:                          # T is the trough
                T = signal                          # keep track of lowest point in pulse wave 

        if signal > thresh and signal > P:
            P = signal

        # signal surges up in value every time there is a pulse
        if N > 250:                                 # avoid high frequency noise
            print("N > 250")
            if signal > thresh and Pulse == False and N > (IBI/5.0)*3:
                print(" signal > thresh and Pulse == False and N > (IBI/5.0)*3")
                Pulse = True                        # set the Pulse flag when we think there is a pulse
                IBI = sampleCounter - lastBeatTime  # measure time between beats in mS
                lastBeatTime = sampleCounter        # keep track of time for next pulse
                if secondBeat:                      # if this is the second beat, if secondBeat == TRUE
                    print("secondBeat")
                    secondBeat = False;             # clear secondBeat flag
                    for i in range(len(rate)):      # seed the running total to get a realisitic BPM at startup
                      rate[i] = IBI
                if firstBeat:                       # if it's the first time we found a beat, if firstBeat == TRUE
                    print("firstBeat")
                    firstBeat = False;              # clear firstBeat flag
                    secondBeat = True;              # set the second beat flag

                # keep a running total of the last 10 IBI values
                rate[:-1] = rate[1:]                # shift data in the rate array
                rate[-1] = IBI                      # add the latest IBI to the rate array
                runningTotal = sum(rate)            # add upp oldest IBI values
                runningTotal /= len(rate)           # average the IBI values
                self.BPM = 60000/runningTotal       # how many beats can fit into a minute? that's BPM!

        if signal < thresh and Pulse == True:       # when the values are going down, the beat is over
            print("signal < thresh and Pulse == True: ")
            Pulse = False                           # reset the Pulse flag so we can do it again
            amp = P - T                             # get amplitude of the pulse wave
            thresh = amp/2 + T                      # set thresh at 50% of the amplitude
            P = thresh                              # reset these for next time
            T = thresh

        if N > 2500:                                # if 2.5 seconds go by without a beat
            print("N > 2500:")
            thresh = 512                            # set thresh default
            P = 512                                 # set P default
            T = 512                                 # set T default
            lastBeatTime = sampleCounter            # bring the lastBeatTime up to date        
            firstBeat = True                        # set these to avoid noise
            secondBeat = False                      # when we get the heartbeat back
            self.BPM = 0

        #time.sleep(0.005)
        
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
      signal = s.getBPM()
      print(signal)
      bpm = s.BPM
      if bpm > 0:
         print("BPM: %d" % bpm)
      #else:
         #print("No Heartbeat found")
      time.sleep(0.1)
except:
   s.stopSensorRead()
