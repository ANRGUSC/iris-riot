#pylint: disable=C0103, C0111, C0301, C0330, W0603, W0611, W0612, W0613
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

# Boolean flag for testing quickly.
quick = True

# Distance being tested - ~1-10 ft.
dist = 2
if not quick:
    dist = input('Enter distance: ')

#Number of samples - ~200.
samp = 1000
loc = "Room"

if not quick:
    samp = input('Enter number of samples: ')

# Orientation based on angle between sensor and vector pointing from tx to rx.
tx_orient = 0
rx_orient = 180

#Threshold - ~45.
thresh = 38
if not quick:
    thresh = input('Enter threshold: ')

#delay between samples
samp_delay= 0.5


#Delay
# delay = 1
# if not quick:
#     delay = input('Enter delay: ')


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

    # Outputs are labeled r_out[distance].txt
    # Example: r_out3.txt = ranging output for 3 ft.

    # Creating and naming the output file.
    # file_exists actually means whether the file exists or not -
    #   if file_exists is true, then it doesn't exist and we're clear to make the file.
    #   if false, then it does exist
    file_exists = True
    missed_pings = 0
    dist_list = []
    missed_pings_list = []

    while file_exists:
        filename = raw_input("\nFile name: ")
        userinput = " "
        
        try:
            filecheck = open(filename + ".txt", 'r')
            
            while userinput != "y" and userinput != "n":
                userinput = raw_input("File already exists, would you like to override it? (y/n) ")
            
            file_exists = True

        except IOError as e:
            print("File name is available")
            file_exists = False

        if file_exists and userinput == "y":
                break

    output1 = open(filename + ".txt", 'w')
    output1.write('Ultrasound scanning\n')
    output1.write('TX Orientation: ' + str(tx_orient) + '\n')
    output1.write('RX Orientation: ' + str(rx_orient) + '\n')
    output1.write('Samples: ' + str(samp) + '\n')
    output1.write('Threshold: ' + str(thresh) + '\n')
    output1.write('Location: ' + loc + '\n')
    output1.write('\n')
    output1.write('Distance, Avg, STDev, Data \n')

    # Rebooting node for safety.
    port.write(b'reboot\n')

    choice = 'y'
    failed = 0
    i = samp
    #start of loop
    while True:
        failed = 0

        if choice == 'n':
            i = samp

        if i >= samp - 1:
            missed_pings_list.append(missed_pings)
            dist = raw_input("Distance: ")
            if dist == 'quit' or dist == 'q':
                break
            else:
                i = -1
                output1.write('\n'+dist+',,,')
                missed_pings = 0
                dist_list.append(dist)

        i += 1

        #Wait until setup completes before entering command.
        # line = b' '
        # while line is not b'':
        #     line = port.readline()
        #     print(line[:-1])

        sleep(samp_delay)

        # # Runs the range_rx command.
        print("writing range_rx")
        port.write(('range_rx %d\n' % (thresh)).encode())

        # Checks for errors, if none, adds the final TDoA to data[].
        while True:

            line = port.readline()
            #print(line[:-1])
            if b'Timed out' in line:
                failed = 1
                print("Timed out")
                i -= 1
                break

            if b'failed' in line:
                failed = 1
                print("Ranging failed")
                i -= 1
                break

            if b'TDoA' in line:
                #print(str(i) + ": " + line[:-1])
                failed = 0
                words = line.split("= ")
                val=words[1][:-1]
                print(str(i)+":"+str((int(val)-24830)/888.06))
                output1.write(val + ",")
                break

            if line == b'':
                #print("Got blank")
                break

            if b'unknown' in line:
            	failed = 1
            	print("unknown packet")
            	i -= 1
            	break

            if b'missed' in line:
            	print("Ping missed")
            	missed_pings += 1
            	break

        if failed:
            choice = ' '
            while choice != 'y' and choice != 'n':
                choice = raw_input('TDoA failed, continue? (y/n) ')
            failed = 1

    i = 0
    output1.write("\nDist, Pings missed")
    for val in missed_pings_list[1:]:
    	output1.write ("\n"+str(dist_list[i])+","+str(val))
    	i += 1

    print("Data gathered")
    output1.close()
    print("File written")

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
            conn = Serial('/dev/ttyUSB' + port_usb, '115200', dsrdtr=0, rtscts=0,
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

    print('Running script...')
    script(conn)
    print('Script complete!')
    
if __name__ == "__main__":
    # main(sys.argv)
    main()
