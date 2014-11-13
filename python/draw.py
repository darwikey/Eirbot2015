import pygame
from pygame.locals import *
import sys
import time
import serial
import threading


# Initialisation des variables
WAIT = 0
READ = 1
STOP = 2


# Opening port for serial communication
ser = serial.Serial(3, 115200,timeout = 0.1)


pygame.init()
screen_width = 700
screen_height = 700
screen = pygame.display.set_mode((screen_width, screen_height))
continuer = 1

def convert_coord(x, y):
  capture_range = 3000.0
  x = x / (2 * capture_range) + 0.5
  x = x * screen_width
  y = y / (2 * capture_range) + 0.5
  y = y * screen_height
  return (int(x), int(y))


while continuer:
  state = WAIT
  x = 0



  line = ser.readline()

  if len(line) > 0:
    l = line.split('#')

    print "ok"
    screen.fill((0,0,0))
    for i in range(-6, 6):
      pygame.draw.line(screen, (128,128,128), convert_coord(i*1000,-6000), convert_coord(i*1000,6000))
      pygame.draw.line(screen, (128,128,128), convert_coord(-6000, i*1000), convert_coord(6000,i*1000))


    for el in l:
      try:
      #add a coord
        if el[0] == 'x':
          x = int(el.split('x')[1])

          #x.append(int(line.split('x')[1]))
        elif el[0] == 'y':
          y = int(el.split('y')[1])
          pygame.draw.circle(screen, (255, 200, 0), convert_coord(x, y), 2)

      except:
        continue

  
  pygame.display.flip()

  for event in pygame.event.get():
    if event.type == QUIT:
      continuer = 0
      pygame.quit()


  

  
