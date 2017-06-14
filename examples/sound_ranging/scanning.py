#pylint: disable=C0103, C0111, C0301, C0330, W0603, W0611, W0612, W0613
from __future__ import print_function
import sys
import re
import os.path

# import socket
from time import sleep, time
from struct import pack
from serial import Serial
# from subprocess import run
# from subprocess import call

# Default port that openmote connects to.
port_usb = '0'

#Boolean flag for testing quickly.
quick = True

#Number of samples - ~200.
samp = 2000
if not quick:
    samp = input('Enter number of samples: ')


#Threshold - ~45.
thresh = 45
if not quick:
    thresh = input('Enter threshold: ')

#orientation based on angle between sensor and vector pointing from tx to rx
tx_orient = 0
rx_orient = 180


#Delay
delay = 200
if not quick:
    delay = input('Enter delay: ')


#Need sudo to make term for ttyUSB - call the following command in terminal:
#sudo adduser 'Your username here' dialout
#Adds you to the dialout group - no need to call sudo to flash/access ttyUSB.
# call(['ls'])
# call(['sudo make term PORT=/dev/ttyUSB' + port_usb])
# call(['sudo make term'])
# run(["ls"])
# run("sudo make term", shell=True)

# call(['range_rx ' + thresh])
# run('range_rx ' + thresh, shell=True)

def script(port):
    file_exists = True
    while file_exists:
        filename = raw_input("\nFile name: ")
        userinput = " "
        
        try:
            filecheck = open(filename + ".txt", 'r')
            
            while userinput != "y" and userinput != "n":
                userinput = raw_input("File already exists, would you like to override it? (y/n) ")
            file_exists= True

        except IOError as e:
            print("File name is available")
            file_exists=False

        if file_exists and userinput == "y":
                break

    output1 = open(filename + ".txt", 'w')
    output1.write('Ultrasound scanning\n')
    output1.write('TX Orientation: ' + str(tx_orient) + '\n')
    output1.write('RX Orientation: ' + str(rx_orient) + '\n')
    output1.write('Samples: ' + str(samp) + '\n')
    output1.write('Threshold: ' + str(thresh) + '\n')
    output1.write('Delay: ' + str(delay) + '\n')
    output1.write('\n')
    output1.write('Time, Value \n')

    #port.write(b'reboot\n')
    dist = " "
    while True:
        
        
        line = b' '
        
        print("writing scan_rx_stop")

        port.write('scan_rx_stop\n')

        while line is not b'':
            line = port.readline()

        port.write(('scan_rx_start %d -s %d\n' % (delay, samp)).encode())
        print("writing scan_rx_start")

        line = port.readline()
        print(line)

        data = []

        i = 0
        # while b'Ping' in line or line is b'':
        while not (b'stopped' in line):
            
            print(line[:-1])
            if b',' in line:
                output1.write(line)

            line = port.readline()
            print(line)

        print("Data gathered")
        dist = raw_input("Quit? ")
        if dist == "quit" or dist == "q":
            break

    output1.close()
    print("File written")

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
            conn = Serial('/dev/ttyUSB' + port_usb, '115200', dsrdtr = 0, rtscts = 0,
                          timeout = 1)
        except IOError:
            print("error opening serial port", file = sys.stderr)
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
    if conn.is_open:
        print('Connected!')
    else:
        return

    sleep(1)
    print('Configured!')

    print('Running script...')
    script(conn)
    print('Script complete!')

if __name__ == "__main__":
    # main(sys.argv)
    main()
