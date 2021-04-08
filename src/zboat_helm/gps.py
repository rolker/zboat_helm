#!/usr/bin/env python

import serial

class GPS:
  def __init__(self):
    self.port = serial.Serial(port='/dev/ttyUSB0', baudrate=19200, timeout=5)

  def read(self):
    return self.port.readline(250)

if __name__ == '__main__':
  gps = GPS()
  while True:
    print (gps.read())

