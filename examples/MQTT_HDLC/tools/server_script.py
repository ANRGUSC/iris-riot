import paho.mqtt.client as mqtt

#initializing the variables
#Change the following according to your system 
broker_address="fd00:dead:beef::1" 
port = 1886
#path_to_certificate="/etc/mosquitto/certs/ca.crt"
topic_sub = "init_info"
global topic_pub 
topic_pub = ""
server_mqtt = {'0': 'ACK', '1': 'Do something'}
hwaddr=[]
#Creating the callback functions 

# The callback for when the client receives a CONNACK response from the server.
# The callback function when client is connected to the broker.
def on_connect(client, userdata, rc):
	print ("Connected to topic", topic_sub)

# Subscribing in on_connect() ensures that if we lose the connection and
# reconnect then subscriptions will be renewed.
# Reestablishing the subscription if connection is lost and 
# then reconnected 

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
	print ("Data received")
	print ("Topic: ", msg.topic, "\n" + "Message:" ,(msg.payload.decode()))
	message = str(msg.payload.decode())
	if message[0] in server_mqtt:
		topic_pub=message[1:]
		hwaddr.append(topic_pub)
		#when a message is received, the message is published to another topic
		client.publish(topic_pub,server_mqtt[message[0]])    

def on_publish(client,userdata,result):     #create function for callback
    print("Data published to topic")
 
#Creating an instance and setting up the callbacks 

client = mqtt.Client() #creating an instance 
client.on_connect = on_connect	#on_connect callback
client.on_message = on_message	#on_message callback 
client.on_publish = on_publish	#on_publish callback	
#client.tls_set(path_to_certificate)	#establishing the SSL certificate
client.connect(broker_address, port)	#connecting to broker

#Setting up the subscribe 

client.subscribe(topic_sub)	#subscribing to test topic
client.loop_forever()	#looping forever so it doesn't terminate 
