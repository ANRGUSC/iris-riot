from __future__ import print_function
import paho.mqtt.client as mqtt


#initializing the variables
#Change the following according to your system 
broker_address="fd00:dead:beef::1" 
port = 1886
#path_to_certificate="/etc/mosquitto/certs/ca.crt"
topic_sub = "init_info"
topic_client_con = "$SYS/broker/clients/connected"
topic_client_id = "$SYS/broker/clients/ID"
global topic_pub 
topic_pub = ""
server_mqtt = {'0': 'ACK', '1': 'Do something'}
hwaddr=[]
global total_num_clients_con_broker
total_num_clients_con_broker=0
global connected_clients
connected_clients=0
client_ID=[]

#Creating the callback functions 

# The callback for when the client receives a CONNACK response from the server.
# The callback function when client is connected to the broker.
	
def usr_input(client):
	c=0
	exit=0
	while(c!=3):
		print("0 - send a normal message to mbed")
		print("1 - send a sub message to the mbed")
		print("2 - send a pub message to the mbed so it publishes to a topic")
		print("3 - exit")
		c=int(input("Enter your choice\t"))
		if c==0:
				usrtopic=str(input("Enter the Client ID of the openmote you want to send a normal message to\t"))
				usrmessage="0"
				usrmessage=usrmessage+str(input("Enter your message\t"))
				break;
		elif c==1:
				usrtopic=input("Enter the Client ID of the openmote that will sub to a topic\t")
				usrmessage="1"
				usrmessage=usrmessage+str(input("Enter the topic you want the openmote to sub to\t"))
				break;
		elif c==2:
				print("This will pub a normal message to the other message")
				usrtopic=input("Enter the Client ID of the openmote that will send the pub message\t")
				usrmessage="2"
				temp=str(input("Enter the topic that the openmote will pub to\t"))
				usrmessage=usrmessage+str(len(temp))+temp
				message_to_be_pubbed=str(input("Enter the message that you want the openmote to publish\t"))
				usrmessage=usrmessage+"0"+message_to_be_pubbed
				break;
		elif c==3:
				exit=1
				break;
		else:
			print("invalid choice")
	if exit==0:
		client.publish(usrtopic,usrmessage)



def on_connect(client, userdata, rc):
	print ("Connected to broker")

# Subscribing in on_connect() ensures that if we lose the connection and
# reconnect then subscriptions will be renewed.
# Reestablishing the subscription if connection is lost and 
# then reconnected 

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
	global topic_pub
	global total_num_clients_con_broker
	global connected_clients
	print ("Data received")
	message = str(msg.payload.decode())
	if msg.topic==topic_sub:		
			if message[0] in server_mqtt:
				topic_pub=message[1:]
				hwaddr.append(topic_pub)
				#when a message is received, the message is published to another topic
				client.publish(topic_pub,server_mqtt[message[0]])     
	elif msg.topic==topic_client_con:
		connected_clients=connected_clients+1
		total_num_clients_con_broker=int(message)
	elif msg.topic==topic_client_id:
		if str(msg.payload.decode()) in client_ID:
				connected_clients=connected_clients-1
		else:
			client_ID.append(str(msg.payload.decode()))
	  
def on_subscribe(mosq, obj, mid, granted_qos):
	print("Subscribed to all topics ")

def on_publish(client,userdata,result): 
	global connected_clients
	print("Data published to topic", topic_pub)
	print("The number of clients connected is",connected_clients)
	print("The total number of clients that have connected to broker", total_num_clients_con_broker)
	print("The clients are", client_ID) 
	#check to ensure that all clients have connected
	#For this test it assumes there are only 2 clients
	if connected_clients==2:
		usr_input(client)
 
#Creating an instance and setting up the callbacks 

client = mqtt.Client() #creating an instance 
client.on_connect = on_connect	#on_connect callback
client.on_message = on_message	#on_message callback 
client.on_publish = on_publish	#on_publish callback	
client.on_subscribe =on_subscribe
#client.tls_set(path_to_certificate)	#establishing the SSL certificate
client.connect(broker_address, port)	#connecting to broker

#Setting up the subscribe 

client.subscribe([(topic_sub, 0), (topic_client_con, 0), (topic_client_id,0)])
client.loop_forever()	#looping forever so it doesn't terminate 