from __future__ import print_function
import sys
import re
# import socket
from time import sleep, time
from struct import pack
from serial import Serial
# from subprocess import run
# from subprocess import call

# Default port that openmote connects to.
port_usb = '0'

#Distance being tested - ~1-10 ft. 
# dist = input('Enter distance: ')
dist = 2

#Number of samples - ~200.
# samp = input('Enter number of samples: ')
samp = 20

#Threshold - ~45.
# thresh = input('Enter threshold: ')
thresh = 45

#Delay
delay = 1

#Need sudo to make term for ttyUSB - call the following command in terminal:
#sudo adduser 'Your username here' dialout
#Adds you to the dialout group - no need to call sudo to flash/access ttyUSB.
# call(['sudo make term PORT=ttyUSB' + port_usb])
# call(['ls'])
# call(['sudo make term'])
# run(["ls"])
# run("sudo make term", shell=True)

#read from term?
# port.readline()

#readlines?
# port.write(b'ifconfig\n')

# print('test1')
# call(['range_rx ' + thresh])
# run('range_rx ' + thresh, shell=True)
# 115200

def script(port):

    port.write(b'reboot\n')

    line = b'sample'
    while line is not b'':
        print(line)
        line = port.readline()

    # port.write(b'range_rx 45\n')

    port.write(('scan_rx_start %d %d\n' % (delay, samp)).encode())

    while True:
        line = port.readline()
        print(line)
        # line.split()
        # if

    # command = input('Awaiting your command: ')
    # port.write(command.encode())

def configure_interface(port, channel):
    line = ""
    iface = 0
    port.write(b'ifconfig\n')
    while True:
        line = port.readline()
        if line == '':
            print("Application has no network interface defined",
                  file=sys.stderr)
            sys.exit(2)
        match = re.search(r'^Iface +(\d+)', line.decode())
        if match is not None:
            iface = int(match.group(1))
            break

    script(port)

# def connect(argv):
def connect():
    # connType = argv[1]
    connType = "serial"

    conn = None
    if connType == "serial":
        # open serial port
        try:
            # conn = Serial(argv[2], argv[3], dsrdtr=0, rtscts=0,
                        #   timeout=1)
            conn = Serial('/dev/ttyUSB0', '115200', dsrdtr=0, rtscts=0,
                          timeout=1)
        except IOError:
            print("error opening serial port", file=sys.stderr)
            sys.exit(2)
    else:
        print("error: unsupported connection type. Use \"serial\" or \"socket\"")
        sys.exit(2)

    return conn


# def main(argv):
def main():
    # if len(argv) < 5:
    #     print("Usage: %s serial tty baudrate channel [outfile]\n" % (argv[0]),
    #     file=sys.stderr)
    #     print("       channel = 11-26", file=sys.stderr)
    #     sys.exit(2)

    # conn = connect(argv)
    print('Connecting...')
    conn = connect()

    print('Connected!')
    print('Configuring...')

    sleep(1)
    # configure_interface(conn, int(argv[4]))
    configure_interface(conn, 0)
    sleep(1)


if __name__ == "__main__":
    # main(sys.argv)
    main()
