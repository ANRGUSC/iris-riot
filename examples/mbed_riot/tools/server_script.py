from __future__ import print_function
from threading import Thread
import paho.mqtt.client as mqtt


#initializing the variables
#Change the following according to your system 
broker_address="fd00:dead:beef::1" 
port = 1886
#path_to_certificate="/etc/mosquitto/certs/ca.crt"
topic_sub = "init_info"
range_sub = "range_info"
range_req_msg = "INIT_RANGE"
node_data_flag = '0'
node_disc_flag = '1'
global topic_pub 
topic_pub = ""
server_mqtt = {'0': 'ACK', '1': 'Do something'}
hwaddr=[]
client_ID=[]

def parse_msg(msg):
    origin = msg[:9]
    msgtype = msg[9]
    print("Message origin: "+ origin)
    if(msgtype == node_data_flag):
        print("Message type: node data")
        return parse_node_data(msg[10:])
    elif(msgtype == node_disc_flag):
        print("Message type: node discovery")
        return parse_node_disc(msg[10:])
    else:
        return -1;
    

def parse_node_disc(msg):
    temp = []
    nodelist= []
    i = 0

    temp = msg.split(",")
    for data in temp[:-1]:
        nodelist.append(int(data))
        i= i+1

    return nodelist


def parse_node_data(msg):
    temp = []
    nodedict = {}
    i = 0

    splitmsg = msg.split(";")
    for data in splitmsg:
        data = data.strip();
        if data != "":
            temp = data.split(",")
            if int(temp[0]) != 0:
                nodedict[int(temp[1])]=int(temp[0])
            else:
                print("No data available; ping missed")
        i = i+1


    return nodedict

#Creating the callback functions 

# The callback for when the client receives a CONNACK response from the server.
# The callback function when client is connected to the broker.
    
def usr_input(client):
    
    c=0
    exit=0
    clientnum=0
    while(c!=5):
        usrtopic = ""
        usrmessage= ""
        print("0 - send a normal message to mbed")
        print("1 - send a sub message to the mbed")
        print("2 - send a pub message to the mbed so it publishes to a topic")
        print("3 - send a ranging request message to the mbed")
        print("4 - show clients")
        print("5 - exit")
        c=int(raw_input("Enter your choice\t"))
        if c==0:
            try:
                clientnum = int(raw_input("Enter the number of the openmote you want to send a normal message to\t"))
                usrtopic=hwaddr[clientnum]
                usrmessage="0"
                usrmessage=usrmessage+raw_input("Enter your message\t")
            except:
                print("Input error")
                continue
        elif c==1:
            try:
                clientnum = int(raw_input("Enter the number of the openmote that will sub to a topic\t"))
                usrtopic=hwaddr[clientnum]
                usrmessage="1"
                usrmessage=usrmessage+raw_input("Enter the topic you want the openmote to sub to\t")
            except:
                print("Input error")
                continue
        elif c==2:
            try:
                print("This will pub a normal message to the other message")
                clientnum = int(raw_input("Enter the number of the openmote that will send the pub message\t"))
                usrtopic=hwaddr[clientnum]
                usrmessage="2"
                temp=raw_input("Enter the topic that the openmote will pub to\t")
                usrmessage=usrmessage+str(unichr(len(temp)))+temp
                message_to_be_pubbed=raw_input("Enter the message that you want the openmote to publish\t")
                usrmessage=usrmessage+message_to_be_pubbed
            except:
                print("Input error")
                continue
        elif c==3:
            try:
                clientnum = int(raw_input("Enter the number of the openmote you want to range\t"))
                node_id = int(raw_input("Enter the node_id of the node you with to range (-1 for discovery mode)\t"))
                node_id += ord('0')
                print("Select ranging mode:")
                ranging_mode = int(raw_input("\n0: ONE_SENSOR_MODE\n1: TWO_SENSOR_MODE\n2: XOR_SENSOR_MODE\n3: OMNI_SENSOR_MODE\nInput:\t"))
                ranging_mode += 96 #this is the offset for ranging_mode values
                usrtopic=hwaddr[clientnum]
                usrmessage="0"
                usrmessage= usrmessage + str(chr(node_id)) + str(chr(ranging_mode)) + range_req_msg
            except:
                print("Input error")
                continue
        elif c==4:
            print("***********************************")
            print("Clients:")
            i = 0
            for addr in hwaddr:
                print(str(i)+": "+addr)
            print("***********************************")
        elif c==5:
            exit=1
            break;
        else:
            print("invalid choice")
            continue;
        if c!=4:
            client.publish(usrtopic,usrmessage)
            print("publishing "+usrmessage+" to "+usrtopic)

def on_connect(client, userdata, flag, rc):
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
    print ("\nData received")
    message = str(msg.payload.decode())
    print(message)
    if msg.topic==topic_sub:        
        if message[0] in server_mqtt:
            topic_pub=message[1:]
            if topic_pub not in hwaddr:
                print(topic_pub + " added to clients list")
                hwaddr.append(topic_pub)
            else:
                print(topic_pub + " already in clients list")
            #when a message is received, the message is published to another topic
            client.publish(topic_pub,server_mqtt[message[0]])   
    elif msg.topic==range_sub:
        data = parse_msg(message);
        print(data)
      
def on_subscribe(mosq, obj, mid, granted_qos):
    print("Subscribed to all topics ")

def on_publish(client,userdata,result): 
    global connected_clients
    print("Data published")
    #check to ensure that all clients have connected
    #For this test it assumes there are only 2 clients
 
#Creating an instance and setting up the callbacks 

client = mqtt.Client("serv") #creating an instance 
client.on_connect = on_connect    #on_connect callback
client.on_message = on_message    #on_message callback 
client.on_publish = on_publish    #on_publish callback    
client.on_subscribe =on_subscribe
#client.tls_set(path_to_certificate)    #establishing the SSL certificate
client.connect(broker_address, port)    #connecting to broker

t = Thread(target=usr_input, args=(client,))
t.start()   

#Setting up the subscribe 

client.subscribe([(topic_sub, 0), (range_sub, 0)])
client.loop_forever()    #looping forever so it doesn't terminate 