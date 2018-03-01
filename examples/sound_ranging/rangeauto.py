from __future__ import print_function
import sys
import re
from time import sleep, time
from struct import pack
from serial import Serial
import os
import math

# Default port that openmote connects to.
port_usb = '1'

# Boolean flag for testing quickly.
quick = True

# Checks if the line has any of the following error messages:
# Timed out, failed, or unknown.
# Returns True if they exist.
# 
d = 3.5/12

def tdoa_to_dist(tdoa):
    return (tdoa -19628.977) / 885.274 + (2.5 / 12)

def od_to_angle(a,b):
    
    x = calc_x(a,b)
    # print(x);
    # print(a);
    # print(b);
    try:
        ans = math.asin((b*b - a*a)/(2*d*x))*180/math.pi 
    except:
        ans = -1
    return ans

def calc_x(a,b):
    return math.sqrt( a*a/2 + b*b/2 - d*d/4 )

def is_failing(line):

    failed = True

    if b'Timed out' in line or b'tout' in line:
        print("Timed out.")
    elif b'unknown' in line:
        print("Unknown packet.")
    elif b'Error' in line:
        print(line)
    else:
        failed = False

    return failed


def script(port):


#-----------------------------------------------------------------------------#

    # Rebooting node for safety.
    port.write(b'reboot\n')
    delay = 1
    datalist = [()]
    
#-----------------------------------------------------------------------------#
#######################
# Start of main loop. #
#######################
    while True:
        mode_input = ''
        mode = 0
        samp_input = 0
        samp = 0
        choice = 'y'
        quit = 0
        a = 0
        b = 0

        # Set options.
        Freq = 1500 # Set Frequency To 2500 Hertz
        Dur = 0.5 # Set Duration To 1000 ms == 1 second
        os.system('play --no-show-progress --null --channels 1 synth %s sine %f' % ( Dur, Freq))
        
        while ((mode < 1) or (mode > 3)):
            mode_input = raw_input("\nRanging modes: \nONE SENSOR = 1\nTWO SENSOR = 2\nXOR SENSOR = 3\nInput: ")
            if mode_input == 'q':
                quit = 1
                break
            if mode_input == 'save':
                filename = raw_input("File name: ")
                filename = filename + ".csv"
                file = open(filename, 'w')
                print(datalist)
                for data in datalist:
                    if len(data) == 2:
                        file.write(str(data[0])+','+str(data[1])+'\n')
                    else: 
                        file.write(str(data[0]))
                file.write("Pings Missed:, "+str(missed_pings))
                file.close()
                continue
            try:
                mode = int(mode_input)
            except:
                print("Input must be a valid integer")
            
        if(quit):
            break;

        while(samp <= 0):
            samp_input = raw_input("\nNumber of samples: ")
            if samp_input == 'q':
                quit = 1
                break
            try:
                samp = int(samp_input)
            except:
                print("Input must be a valid integer")
        
        if(quit):
            break;


        missed_pings = 0
        failed_counter = 0
        tdoa_skipped = 0
        i = 0
        j = -1
        deadair = 0

        del datalist[:]
        datalist = []

        # trials = raw_input("Enter number of trials: ")
        # don't forget to cast trials to an int!
        #trials = 1
        
        port.write(('range_rx %d %d %d\n' % (samp, delay, mode-1)).encode())

        while i < int(samp):

            #Wait until setup completes before entering command.
            # line = b' '
            # while line is not b'':
            #     line = port.readline()
            #     print(line[:-1])

            # Runs the range_scan_rx command.

            # print("Writing range_scan_rx.")
            
            # port.write(b'range_rx')

            # Line processing.
            while True:
                line = port.readline()
                if line == "":
                    deadair += 1
                    if deadair > 4:
                        print("Rewriting range_rx")
                        port.write(('range_rx %d %d %d\n' % (samp-i, delay, mode-1)).encode())
                else:
                    deadair = 0

                # print(line)
                # TDoA
               
                if b'TDoA' in line:
                    failed_counter = 0
                    words = line.split("= ")
                    val = words[1][:-1]
                    # Formula for calculating distance, derived through past tests.
                    # print(str(i) + ": " + str((int(val)-24713.5)/892.07)) 
                    print("Trial: " + str(i) + " of " + samp_input)
                    
                    print("TDoA: " + val)
                    datalist.append([val,0])
                    j=j+1
                    # if mode == 1:
                    print("\n****************************")
                    print("Dist = "+ str(tdoa_to_dist(int(val))))
                    print("****************************\n")
                    # break
                    a = int(val);
                    i+=1
            
                if b'OD ' in line:
                    if b'failed' in line:
                        print("\n****************************")
                        # print("Dist = "+ str(tdoa_to_dist(a)))
                        print("OD failed")
                        print("****************************\n")
                        val = ""
                        break
                    else:
                        words = line.split("= ")
                        val = words[1][:-1]
                    # Formula for calculating distance, derived through past tests.

                    # print(str(i) + ": " + val)
                    print("OD  : " + val)
                    b = a + int(val)
                    a = tdoa_to_dist(a)
                    b = tdoa_to_dist(b)
                    print("\n****************************")
                    print("Dist = "+ str(calc_x(a,b)))
                    print("Angle = "+ str(od_to_angle(a,b)))
                    print("****************************\n")
                    datalist[j][1] = val
                    break

                if b'All up' in line:
                    break

                if b'missed' in line:
                    failed_counter = 0
                    print("Ping missed")
                    missed_pings += 1
                    print("Trial: " + str(i) + " of " + samp_input)
                    i+=1
                    break

                # Checking for errors.
                failed = is_failing(line)
                if failed:
                    if failed_counter >= 5:                    
                        choice = ''
                        while choice != 'y' and choice != 'n':
                            choice = raw_input('TDoA failed, continue? (y/n) ')
                            # print('TDoA failed, continuing')
                            # choice = 'y'
                        if choice == 'n':
                            i = int(samp)
                            break
                    
                    failed_counter += 1
                    i -= 1
                    tdoa_skipped += 1
                    break
            
            # Wait.
            sleep(0.01)

# End of main loop.
#-----------------------------------------------------------------------------#
    i = 0
#-----------------------------------------------------------------------------#


def connect():
    try:
        conn = Serial('/dev/ttyUSB'+port_usb,
                        '115200',
                        dsrdtr=0,
                        rtscts=0,
                        timeout=1)
    except IOError:
        print("Error opening serial port.", file=sys.stderr)
        sys.exit(2)

    return conn


def main():
    print("Connecting...")
    conn = connect()
    print("Connected!")

    run_script = 'y'

    while run_script == 'y':
        print("Running script...")
        script(conn)
        print("Script complete!")

        run_script = ''
        while run_script != 'y' and run_script != 'n':
            # run_script = input("Run script again? (y/n) ") # Python 3
            run_script = raw_input("Run script again? (y/n) ")


if __name__ == "__main__":
    main()
