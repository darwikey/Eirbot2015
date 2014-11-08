# -*- coding: cp1252 -*-
import sys
#sys.path.insert(0,"../")
import time
import serial
import matplotlib.pyplot as plt


# Initialisation des variables
WAIT = 0
READ = 1
STOP = 2



# Opening port for serial communication
ser = serial.Serial(3, 115200,timeout = 1)

size = 2000
plt.axis([-size, size, -size, size])
plt.ion()
plt.show()



while 1:
  x = []
  y = []
  state = WAIT

  
  #x.append(0)



