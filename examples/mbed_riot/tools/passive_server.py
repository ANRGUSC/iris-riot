from __future__ import print_function
from threading import Thread
import paho.mqtt.client as mqtt
from struct import *
import time
import json
from triangulate import *

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
topic_pub = ""
server_mqtt = {'0': 'ACK', '1': 'Do something'}
hwaddr=[]
message_rcvd=False
save_to_file=False
file_name = ""
rcvd_data = {}
valid_nodelist = False
a_nodelist = {}

def parse_msg(msg):
    data_info = msg[:11]
    rcvd_data = msg[11:]
    origin, msgtype, size = unpack("=9s1sB", data_info)
    origin = origin.decode()
    msgtype = msgtype.decode()
    print(msgtype)
    if(msgtype == node_data_flag):
        return {"origin": origin, "data": parse_node_data(rcvd_data, size), "typeflag": node_data_flag}
    elif(msgtype == node_disc_flag):
        return {"origin": origin, "data": parse_node_disc(rcvd_data, size), "typeflag": node_data_flag}
    else:
        return -1;
    

def parse_node_disc(msg, size):
    nodelist= []

    fmt = "=" + "b" * size
    
    nodelist = unpack(fmt, msg)

    return nodelist


def parse_node_data(msg, size):
    temp = []
    nodedict = {}
    fmt = "=" + "bH" * size
    
    temp = unpack(fmt, msg)
    
    for i in range(size):
        nodedict[temp[i*2]] = temp[i*2 + 1]; 

    return nodedict

#Creating the callback functions 


def on_connect(client, userdata, flag, rc):
    print ("Connected to broker")

# Subscribing in on_connect() ensures that if we lose the connection and
# reconnect then subscriptions will be renewed.
# Reestablishing the subscription if connection is lost and 
# then reconnected 

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    global message_rcvd
    global topic_pub
    global total_num_clients_con_broker
    global connected_clients
    global rcvd_data

    print ("\nData received")
    #message = str(msg.payload.decode())
    #print(message)
    print(msg.payload)
    #print(msg.payload.decode())
    #print(":".join("{:02x}".format(ord(c)) for c in msg.payload))
    payload = msg.payload

    if msg.topic==topic_sub:
        message=payload.decode()        
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
        rcvd_data = parse_msg(payload)
        if rcvd_data["typeflag"] == node_data_flag:
            print("Origin: " + str(rcvd_data["origin"]))
            print("Type: Node Data")
            print("Data: " + str(rcvd_data["data"]))
            position = []

            if len(rcvd_data["data"]) >= 3:
                position = triangulate(rcvd_data["data"], a_nodelist)
                print(list(position))
                mean = (np.mean(position, axis = 0))
                print(mean)

        elif rcvd_data["typeflag"] == node_data_flag:
            print("Origin: " + str(rcvd_data["origin"]))
            print("Type: Node ID")
            print("Data: " + str(rcvd_data["data"]))
        else:
            print("Unexpected type found")

    message_rcvd = True
        
def on_subscribe(mosq, obj, mid, granted_qos):
    print("Subscribed to all topics ")

def on_publish(client,userdata,result): 
    global connected_clients
    print("Data published")
    #check to ensure that all clients have connected
    #For this test it assumes there are only 2 clients
 
#Creating an instance and setting up the callbacks 

def main():
    global a_nodelist
    global valid_nodelist

    client = mqtt.Client("serv") #creating an instance 
    client.on_connect = on_connect    #on_connect callback
    client.on_message = on_message    #on_message callback 
    client.on_publish = on_publish    #on_publish callback    
    client.on_subscribe =on_subscribe
    #client.tls_set(path_to_certificate)    #establishing the SSL certificate

    try:
        json_data=open('anchor_nodelist.json')
        a_nodelist = json.load(json_data)
        a_nodelist = a_nodelist['list'][0]
        valid_nodelist = True
    except:
        print('An error occured while reading anchor nodelist')
        valid_nodelist = False

    client.connect(broker_address, port)    #connecting to broker

    #Setting up the subscribe 

    client.subscribe([(topic_sub, 0), (range_sub, 0)])
    client.loop_forever()    #looping forever so it doesn't terminate 
    
    # x = triangulate({'4': 0.707, '3': 0.707, '2': 0.707, '1': 0.707}, a_nodelist)
    # print("Estimates:")
    # i=0
    # for n in x:
    #     i=i+1
    #     print(str(i)+": "+str(n))
    # print("Mean:")
    # print(np.mean(x, axis = 0))

if(__name__=="__main__"):
    main()
