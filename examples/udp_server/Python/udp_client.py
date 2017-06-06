# udp_client
# A simple test client to send data to the UDP server

import socket
import sys

host = '::1' #Since server is on local host
port = int(input("Choose UDP Server port: "))

#Create socket
try:
	s = socket.socket(socket.AF_INET6, socket.SOCK_DGRAM)
	print("Socket created")
except socket.error:
	print("Failed to create socket")
	sys.exit()

while(1):
	#data received in bytes data type, so must convert or compare with bytes
	reply = b'0'
	msg = input("Enter message to send: ")
	msg_bytes = msg.encode('utf-8')
	try:
		while reply != b'$$ACK$$':
			#Send message to server
			s.sendto(msg_bytes, (host, port))
			#Wait for acknowledgement
			d = s.recvfrom(1024)
			reply = d[0]
	except socket.error as err:
		print("Error Code: " + str(err[0]) + " Message: " + str(err[1]))
		sys.exit()

