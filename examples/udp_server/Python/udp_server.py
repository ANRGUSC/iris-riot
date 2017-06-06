# udp_server.py
# Creates a server to handle udp interactions with motes or client scripts

import socket
import sys

devices = []

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
	#Output message to terminal
	print("Message[" + addr[0] + ":" + str(addr[1]) + "] - " 
		+ str(data.strip()))

s.close()

