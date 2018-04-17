#!/usr/bin/python
import sys, struct, serial
import readchar

ser = serial.Serial('/dev/tty.usbserial-A504QPCN', 115200)
ser2 = serial.Serial('/dev/tty.usbserial', 115200)
ser.flushInput()

while 1:

  print("Ready to Read boys")
  input2 = ser2.read()
  print("Read Byte: " + input2)
  ser.write(struct.pack('B', ord(input2)))