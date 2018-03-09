#!/usr/bin/python
import sys, struct, serial
import readchar

if len(sys.argv) < 2:
   print("No device specified.")
   print("Specify the serial port of the device you wish to connect to.")
   print("Example:")
   print("   pololu_cmd_tx.py /dev/ttyUSB0")
else:
   ser = serial.Serial(sys.argv[1], 115200)
   ser.flushInput()
   print 'Enter your commands below.\r\nInsert "x" to leave the application.'
   input=1
   while 1:
   # get keyboard input
      input = readchar.readchar()
      # Python 3 users
      # input = input(">> ")
      if input == 'x':
        ser.close()
        exit()
      else:
        ser.write(struct.pack('B', ord(input)))

      # elif input == 'w':
      #   ser.write(struct.pack('B', 119))
      # elif input == 'a':
      #   ser.write(struct.pack('B', 97))
      # elif input == 's':
      #   ser.write(struct.pack('B', 115))
      # elif input == 'd':
      #   ser.write(struct.pack('B', 100))
        # w=119 a=97 s=115 d=100

# except (KeyboardInterrupt):
#    ser.close()
#    print("All done!")