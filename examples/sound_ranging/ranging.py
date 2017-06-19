#pylint: disable=C0103, C0111, C0301, C0326, C0330, W0603, W0611, W0612, W0613
from __future__ import print_function
import sys
import re
from time import sleep, time
from struct import pack
from serial import Serial


# Default port that openmote connects to.
port_usb = '0'

# Boolean flag for testing quickly.
quick = True

# Checks if the line has any of the following error messages:
# Timed out, failed, or unknown.
# Returns True if they exist.
def is_failing(line):

    failed = True

    if b'Timed out' in line or b'tout' in line:
        print("Timed out.")
    elif b'failed' in line:
        print("Ranging failed.")
    elif b'unknown' in line:
        print("Unknown packet.")
    elif b'Error' in line:
        print(line)
    else:
        failed = False

    return failed


def script(port):
#-----------------------------------------------------------------------------#
###############################
# Setting up the output file. #
###############################
    user_input = 'n'
    is_filename_open = True
    filename = ''

    while is_filename_open and user_input == 'n':
        # Input filename.
        # filename = input("\nFile name: ") # Python 3
        filename = raw_input("\nFile name: ")

        try:
            # Check if filename already exists.
            filecheck = open(filename + ".txt", 'r')

            user_input = ''
            # filecheck successful, filename already exists.
            # Check if user wants to overwrite.
            while user_input != 'y' and user_input != 'n':
                # user_input = input("File already exists. Overwrite? (y/n) ") # Python 3
                user_input = raw_input("File already exists. Overwrite? (y/n) ")

        except IOError as e:
            #filecheck unsuccessful, filename does not already exist.
            print("File name is available.")
            is_filename_open = False

    output1 = open(filename + ".txt", 'w')
#-----------------------------------------------------------------------------#
#####################################
# Writing preliminary info to file. #
#####################################
    output1.write('Ultrasound scanning\n')

    # Get settings from user.
    if not quick:
        # samp        = input("Enter number of samples: ") # Python 3
        # loc         = input("Enter testing location: ") # Python 3
        # tx_orient   = input("Enter tx orientation (0-180): ") # Python 3
        # rx_orient   = input("Enter rx orientation (0-90): ") # Python 3
        # thresh      = input("Enter threshold: ") # Python 3
        # samp_delay  = input("Enter sample delay: ") # Python 3
        samp        = raw_input("Enter number of samples: ")
        loc         = raw_input("Enter testing location: ")
        tx_orient   = raw_input("Enter tx orientation (0-180): ")
        rx_orient   = raw_input("Enter rx orientation (0-90): ")
        thresh      = raw_input("Enter threshold: ")
        samp_delay  = raw_input("Enter sample delay: ")
    # Quick test settings.
    else:
        samp        = 100
        loc         = "Room"
        tx_orient   = 0
        rx_orient   = 0
        thresh      = 38
        samp_delay  = 0.1

    output1.write("Samples: " + str(samp) + '\n')
    output1.write("Location: " + loc + '\n')
    output1.write("TX Orientation: " + str(tx_orient) + '\n')
    output1.write("RX Orientation: " + str(rx_orient) + '\n')
    output1.write("Threshold: " + str(thresh) + '\n')
    output1.write('\n')
    output1.write("Distance, Avg, STDev, Data \n")
#-----------------------------------------------------------------------------#
############################
# Setting up the openmote. #
############################
    missed_pings = 0
    dist_list = []
    missed_pings_list = []

    # Rebooting node for safety.
    port.write(b'reboot\n')

    choice = 'y'
    i = samp
#-----------------------------------------------------------------------------#
#######################
# Start of main loop. #
#######################
    while True:
        if i >= samp - 1 or choice == 'n':
            i = samp
            missed_pings_list.append(missed_pings)

            dist = raw_input("Distance: ")
            if dist == 'quit' or dist == 'q':
                break
            else:
                i = -1
                output1.write('\n' + dist + ',,,')
                missed_pings = 0
                dist_list.append(dist)

        i += 1

        #Wait until setup completes before entering command.
        # line = b' '
        # while line is not b'':
        #     line = port.readline()
        #     print(line[:-1])

        # Runs the range_rx command.
        print("Writing range_rx.")
        port.write(('range_rx %d\n' % (thresh)).encode())

        # Line processing.
        line = port.readline()

        # TDoA 
        if b'TDoA' in line:
            words = line.split("= ")
            val = words[1][:-1]
            # Formula for calculating distance, derived through past tests.
            print(str(i) + ":" + str((int(val)-24830)/888.06))
            output1.write(val + ",")

        if b'missed' in line:
            print("Ping missed")
            missed_pings += 1

        # Checking for errors.
        failed = is_failing(line)
        if failed:
            choice = ''
            while choice != 'y' and choice != 'n':
                # choice = raw_input('TDoA failed, continue? (y/n) ')
                print('TDoA failed, continuing')
                choice = 'y'
            i -= 1

        # Wait.
        sleep(samp_delay)
# End of main loop.
#-----------------------------------------------------------------------------#
#########################
# Writing data to file. #
#########################
    i = 0
    output1.write("\nDist, Pings missed")
    for val in missed_pings_list[1:]:
        output1.write("\n" + str(dist_list[i]) + "," + str(val))
        i += 1

    print("Data gathered.")

    print("Writing to file...")
    output1.close()
    print("File written!")
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
