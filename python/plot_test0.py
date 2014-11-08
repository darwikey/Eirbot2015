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
  while state != STOP:
    line = ser.readline()

    if len(line) > 0:
      if line[0] == 'b': #begin
        state = READ;
        print "begin"
      elif line[0] == 'e' and state == READ: #end
        state = STOP
        print "stop"

      else:
        
        if state == READ:
          line = line.split('\n')[0]

            #add a coord
          if line[0] == 'x':
            x.append(int(line.split('x')[1]))
          elif line[0] == 'y':
            y.append(int(line.split('y')[1]))
          



  print "nombre points", len(x), len (y)
  plt.clf()
  plt.scatter(x, y)
  plt.draw()



