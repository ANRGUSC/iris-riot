# udp_server.py
# Creates a server to handle udp interactions with motes or client scripts

import socket
import sys

<<<<<<< HEAD
devices = []
=======
# Dictionary of Address 4tuples by mote
# Can change key names later
devices = {'A':('2001:db8::212:4b00:433:ed4c', 8888, 0, 0),
			'B':('2001:db8::212:4b00:433:ed4e', 8888, 0, 0),
			'C':('2001:db8::212:4b00:433:ecf5', 8888, 0, 0),
			'D':('2001:db8::212:4b00:433:ed19', 8888, 0, 0),
			'E':('2001:db8::212:4b00:433:ed02', 8888, 0, 0),
			'F':('2001:db8::212:4b00:613:66d', 8888, 0, 0),
			'G':('2001:db8::212:4b00:613:1556', 8888, 0, 0),
			'H':('2001:db8::212:4b00:433:ed2a', 8888, 0, 0)}
# Address of border router
# Change based on mote being used
bdr_addr = devices['B']
bdr_key = 'B'
# Devices actually in use
registered = {}
>>>>>>> develop

port = int(input("Choose a port to use for UDP server: "))

#Creating socket
try:
	s = socket.socket(socket.AF_INET6, socket.SOCK_DGRAM)
	print("Socket created")
except socket.error as msg :
	print("Failed to create socket. Error Code: " + str(msg[0]) \
	+ " Message: " + msg[1])
	sys.exit()

#Bind socket to local host and port
try:
	s.bind(("::", port))
except socket.error as msg :
	print("Bind Failed. Error code: " + str(msg[0]) + "Message: " + msg[1])
	sys.exit()

print('Socket bind complete')

<<<<<<< HEAD
while 1:
	#receive data from client (data, addr)
	d = s.recvfrom(1024)
	data = d[0]
	addr = d[1]
	
	#If given start up signal, add address to list of connected devices
	if data == b'$$START$$':
		for device in devices:
			if addr == device:
				break
		else:
			devices.append(addr)

	if not data:
		break

	#Send acknowledge signal to sender
	s.sendto(b'$$ACK$$', addr)
	#Send data to rest of connected devices
	for device in devices:
		if addr != device:
			s.sendto(data, device)
=======
#Sets Timeout for recvfrom function to 1s
s.settimeout(1)

#Register the border router
while 1:
	#Send start signal to border router
	try:
		s.sendto(b'$$START$$\0', bdr_addr)
	except OSError as msg:
		continue
	#print('Message sent')

	#receive data from client (data, addr)
	try:
		d = s.recvfrom(1024)
	except socket.timeout as msg:
		print('System timeout')
		continue
	data = d[0]
	addr = d[1]
	#print('Message received')
	
	#If given ack signal, add address to list of registered devices
	if data == b'$$ACK$$' \
	and addr[0:2] == ('fe80::202:f8ff:fe70:f121%tap0', 8888):
		registered[bdr_key] = addr
		print("Border router registered.")
		break
	if not data:
		break

# Register all other motes
num_connected = 0
print("\nNot including the border router,")
num_devices = int(input("enter how many motes will connect to server: "))

while num_connected < num_devices:
	for key in devices.keys():
		if key not in registered:
			#Send start signal to motes
			try:
				s.sendto(b'$$START$$\0', devices[key])
			except OSError as msg:
				continue

			#receive data from client (data, addr)
			try:
				d = s.recvfrom(1024)
			except socket.timeout as msg:
				print("Mote {} is not currently available".format(key))
				continue
			data = d[0]
			addr = d[1]

			#If given ack signal, add address to list of registered devices
			if data == b'$$ACK$$':
				registered[key] = addr
				print("Successfully connected to mote {}".format(key))
				num_connected += 1
				if num_connected >= num_devices:
					break
			else:
				print("Received something other than $$ACK$$")

#Make all socket functions blocking again, no more timeouts
s.setblocking(True)
#Server functionality
while 1:

	d = s.recvfrom(1024)
	data = d[0]
	addr = d[1]
	#Send acknowledge signal to sender
	s.sendto(b'$$ACK$$', addr)
	#Send data to rest of connected devices
	for key in registered.keys():
		if addr != registered[key]:
			s.sendto(data, registered[key])
>>>>>>> develop
	#Output message to terminal
	print("Message[" + addr[0] + ":" + str(addr[1]) + "] - " 
		+ str(data.strip()))

s.close()

