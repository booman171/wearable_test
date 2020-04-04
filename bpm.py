import time

class BPM:
    def __init__(self):
        #self.ecg = ecg
        self.MAX_BUFFER = 100
        self.prevData = [None] * self.MAX_BUFFER
        self.sumData=0
        self.maxData=0
        self.avgData=0
        self.roundrobin=0
        self.countData=0
        self.period=0
        self.lastperiod=0
        self.millistimer=time.time()
        self.frequency=0
        self.beatspermin=0

    def getBPM(self):
        return self.beatspermin

    def freqDetec(self):
        if(self.countData == self.MAX_BUFFER):
            if(self.prevData[self.roundrobin] < self.avgData*1.5 and self.newData >= self.avgData*1.5):
                self.period = time.time()-self.millistimer
                self.millistimer = time.time()
                self.maxData = 0
        self.roundrobin += 1
        if(self.roundrobin >= self.MAX_BUFFER):
            self.roundrobin=0
        if(self.countData<self.MAX_BUFFER):
            self.countData += 1
            self.sumData+=self.newData
        else:
            self.sumData+=self.newData-self.prevData[self.roundrobin];
        self.avgData = self.sumData/self.countData;
        if(self.newData>self.maxData):
            self.maxData = self.newData
        self.prevData[self.roundrobin] = self.newData; #store previous value
        print("hjh" + str(self.maxData))

    def calcBPM(self, ecg):
        self.newData = ecg
        self.freqDetec();
        if (self.period!=self.lastperiod):
            self.frequency = 1000/float(self.period); #timer rate/period
            if (self.frequency*60 > 20 and self.frequency*60 < 200): # supress unrealistic Data
                self.beatspermin=self.frequency*60;
                self.lastperiod=self.period;
        time.sleep(0.01)
