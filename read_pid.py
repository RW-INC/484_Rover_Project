import numpy as np 
import serial
import csv
import sys
import pandas as pd
import time
import matplotlib.pyplot as plt

output_file = "fuckthisshit.csv"

serial_port = '/dev/ttyACM0'
baud_rate = 115200

ser = serial.Serial(serial_port, baudrate=baud_rate, timeout=2)
with open(output_file, mode='w') as file1:
    pass

test = False
start = time.time()
try:
    while time.time() - start < 60:
        
        if not test:
            line = ser.read(4096)
            test = True
            continue
        line = ser.read(4096).decode().strip()[:-4].split('\r\n')
        line = line[5:-5]
        if line:
            data = [i.split(",") for i in line]
            if len(data):
                with open(output_file, mode='a') as file1:
                    writer = csv.writer(file1)
                    for i in data:
                        writer.writerow(i)
            else:
                print("FUCK THIS SHIT")
        else:
            print("there's no data here fucker")
except KeyboardInterrupt:
    print("hey why'd you stop")
ser.close()
data = np.loadtxt(output_file, delimiter=",").T
print(np.shape(data))

title = ["FL", "FR", "BL", "BR"]
for i in range(len(data)):
    plt.plot(data[i])
plt.legend(title)
plt.show()



