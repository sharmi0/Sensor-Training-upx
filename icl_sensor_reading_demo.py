import serial
import time
import io
from pathlib import Path
from datetime import datetime
import numpy as np
# -----------------------------------
file_postfix = "Sensitivity"


baro = serial.Serial('/dev/tty.usbmodem1101', 2000000, timeout=0.5)



def readBaroSerial():
    output = baro.read().decode()
    return output

def readBaroSerialLine():
    output = ''
    if (baro.in_waiting>=114):
        output = baro.readline().decode()
    return output




now = datetime.now()
now_formatted = now.strftime("%Y-%m-%d_%Hh%Mm%Ss")
path = f"{Path(__file__).parent}/Logs/{now_formatted}_{file_postfix}.csv"
logF = open(path,'w')

while(baro.in_waiting < 1):
    pass #wait for serial port to establish comms


for _ in range(10):
    baro.readline()



startTime = datetime.now()

while 1:
    baroData = readBaroSerialLine()
    if (len(baroData)>1):
        milliseconds = int((datetime.now() - startTime).total_seconds() * 1000.0)
        logLine = str(milliseconds) + ','
        logLine = logLine+baroData
        logF.write(logLine)