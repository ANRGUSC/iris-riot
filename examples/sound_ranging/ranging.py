+#pylint: disable=C0103, C0111, C0301, C0330, W0603, W0611, W0612, W0613
+from __future__ import print_function
+import sys
+import re
+# import socket
+from time import sleep, time
+from struct import pack
+from serial import Serial
+# from subprocess import run
+# from subprocess import call
+
+# Default port that openmote connects to.
+# port_usb = '0'
+
+#Boolean flag for testing quickly.
+quick = True
+
+#Distance being tested - ~1-10 ft.
+dist = 2
+if not quick:
+    dist = input('Enter distance: ')
+
+
+#Number of samples - ~200.
+samp = 20
+if not quick:
+    samp = input('Enter number of samples: ')
+
+
+#Threshold - ~45.
+thresh = 45
+if not quick:
+    thresh = input('Enter threshold: ')
+
+
+#Delay
+# delay = 1
+# if not quick:
+#     delay = input('Enter delay: ')
+
+
+#Need sudo to make term for ttyUSB - call the following command in terminal:
+#sudo adduser 'Your username here' dialout
+#Adds you to the dialout group - no need to call sudo to flash/access ttyUSB.
+# call(['ls'])
+# call(['sudo make term PORT=/dev/ttyUSB' + port_usb])
+# call(['sudo make term'])
+# run(["ls"])
+# run("sudo make term", shell=True)
+
+# call(['range_rx ' + thresh])
+# run('range_rx ' + thresh, shell=True)
+
+def script(port):
+
+    # Outputs are labeled r_out[distance].txt
+    # Example: r_out3.txt = ranging output for 3 ft.
+    output1 = open("r_out" + str(dist) + ".txt", 'w')
+
+    # Holds the data, will then write this to output1 at the end.
+    data = []
+
+    # Rebooting node for safety.
+    port.write(b'reboot\n')
+
+    choice = '1'
+    #start of loop
+    while choice is '1':
+
+        # Wait until setup completes before entering command.
+        line = b' '
+        while line is not b'':
+            line = port.readline()
+            print(line)
+
+        # Runs the range_rx command.
+        port.write(('range_rx %d\n' % (thresh)).encode())
+
+        # Checks for errors, if none, adds the final TDoA to data[].
+        line = port.readline()
+        print(line)
+        while b'Error' or b'Timed' b'Unknown' not in line:
+            line = port.readline()
+            print(line)
+
+            if b'TDoA' in line:
+                words = line.split()
+                data.append(int(words[1]))
+
+            print('Command run successfully.')
+
+        # If an error has occurred.
+        if b'Error' or b'Timed' b'Unknown' in line:
+            print('Run failed.')
+            print(line)
+
+        choice = str(input('Press 1 to continue, or 2 to finish.'))
+
+    output1.write('Distance: ' + str(dist) + '\n')
+    output1.write('Samples: ' + str(samp) + '\n')
+    output1.write('Threshold: ' + str(thresh) + '\n')
+    # output1.write('Delay: ' + str(delay) + '\n')
+    output1.write('Data: \n')
+
+    for datum in data:
+        output1.write(str(datum) + ', ')
+        # output1.write(datum + '\n')
+
+    output1.close()
+
+def configure_interface(port, channel):
+    line = ""
+    iface = 0
+    port.write(b'ifconfig\n')
+    while True:
+        line = port.readline()
+        if line == '':
+            print("Application has no network interface defined",
+                  file=sys.stderr)
+            sys.exit(2)
+        match = re.search(r'^Iface +(\d+)', line.decode())
+        if match is not None:
+            iface = int(match.group(1))
+            break
+
+
+# def connect(argv):
+def connect():
+    # connType = argv[1]
+    connType = "serial"
+
+    conn = None
+    if connType == "serial":
+        # open serial port
+        try:
+            # conn = Serial(argv[2], argv[3], dsrdtr=0, rtscts=0,
+                        #   timeout=1)
+            conn = Serial('/dev/ttyUSB1', '115200', dsrdtr=0, rtscts=0,
+                          timeout=1)
+        except IOError:
+            print("error opening serial port", file=sys.stderr)
+            sys.exit(2)
+    else:
+        print("error: unsupported connection type. Use \"serial\" or \"socket\"")
+        sys.exit(2)
+
+    return conn
+
+
+# def main(argv):
+def main():
+    # if len(argv) < 5:
+    #     print("Usage: %s serial tty baudrate channel [outfile]\n" % (argv[0]),
+    #     file=sys.stderr)
+    #     print("       channel = 11-26", file=sys.stderr)
+    #     sys.exit(2)
+
+    # conn = connect(argv)
+    print('Connecting...')
+    conn = connect()
+    print('Connected!')
+
+    sleep(1)
+    print('Configured!')
+
+    print('Running script...')
+    script(conn)
+    print('Script complete!')
+    sleep(3)
+
+if __name__ == "__main__":
+    # main(sys.argv)
+    main()
+
