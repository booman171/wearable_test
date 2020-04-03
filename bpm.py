import time

max_buffer = 100
prevData = [None] * MAX_BUFFER
sumData=0
maxData=0
avgData=0
roundrobin=0
countData=0
period=0
lastperiod=0
millistimer=time.time()
frequency=0
beatspermin=0
newData = 0

def freqDetec(){
   if(countData == MAX_BUFFER):
      if(prevData{roundrobin] < avgData*1.5 and newData >= avgData*1.5:
         period = time.time()-millistimer
         millistimer = time.time()
         maxData = 0
   roundrobin++
   if(roundrobin >= MAX_BUFFER):
      roundrobin=0
   if(countData<MAX_BUFFER):
      countData++
      sumData+=newData
   else:
      sumData+=newData-prevData[roundrobin];
   avgData = sumData/countData;
   if(newData>maxData):
      maxData = newData
   prevData[roundrobin] = newData; #store previous value


