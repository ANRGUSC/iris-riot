from __future__ import print_function
from threading import Thread
import paho.mqtt.client as mqtt
from struct import *
import time
import json

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
node_anchor_map = {}
message_rcvd=False
save_to_file=False
file_name = ""
rcvd_data = {}
valid_nodelist = False
a_nodelist = {}

def triangulate(node_data):
    #do something eventually
    return [0,0]

def parse_msg(msg):
    data_info = msg[:11]
    rcvd_data = msg[11:]
    origin, msgtype, size = unpack("=9s1sB", data_info)
    origin = origin.decode()
    msgtype = msgtype.decode()
    print("Message origin: "+ origin)
    print("Size: " + str(size))
    print(msgtype)
    if(msgtype == node_data_flag):
        print("Message type: node data")
        return parse_node_data(rcvd_data, size)
    elif(msgtype == node_disc_flag):
        print("Message type: node discovery")
        return parse_node_disc(rcvd_data, size)
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

    # splitmsg = msg.split(";")
    # for rcvd_data in splitmsg:
    #     rcvd_data = rcvd_data.strip();
    #     if rcvd_data != "":
    #         temp = rcvd_data.split(",")
    #         if int(temp[0]) != 0:
    #             nodedict[int(temp[1])]=int(temp[0])
    #         else:
    #             print("No data available; ping missed")
    #     i = i+1
    
    temp = unpack(fmt, msg)
    
    for i in range(size):
        nodedict[temp[i*2]] = temp[i*2 + 1]; 

    return nodedict

#Creating the callback functions 

# The callback for when the client receives a CONNACK response from the server.
# The callback function when client is connected to the broker.

def usr_input(client):
    global message_rcvd
    global file_name
    global save_to_file
    global rcvd_data

    c=0
    exit=0
    clientnum=0
    while(c!=6):
        usrtopic = ""
        usrmessage= ""
        
        print("0 - exit")
        print("1 - show clients")
        print("2 - show node data")
        print("3 - send a series of ranging request messages and save to csv")
        print("4 - send a ranging request message to the mbed")
        print("5 - trangulate on client")
        
        c=int(input("Enter your choice\t"))

        save_to_file = False;
        message_rcvd = False;

        if c==0:
            exit=1
        elif c==1:
            print("***********************************")
            print("Clients:")
            i = 0
            for addr in hwaddr:
                print(str(i)+": "+addr)
            print("***********************************")
        
            break;
        elif c==2:
            print("***********************************")
            print("Clients:")
            i = 0
            for addr in node_anchor_map.keys():
                print(addr+":")
                for anchor in node_anchor_map[addr].keys():
                    print("\t"+anchor+": "+str(node_anchor_map[addr][anchor]))
            print("***********************************")
        
            break;
        elif c==3:
            save_to_file = True
            node_id = 0
            try:
                file_name = input("File name:")
                file_name = file_name + ".csv"

                clientnum = int(input("Enter the number of the openmote you want to range\t"))
                usrtopic=hwaddr[clientnum]

                node_id = int(input("Enter the node_id of the node you with to range"))
                if(node_id < 0):
                    print("Input error")
                    save_to_file = False;
                    continue
                node_id += ord('0')

                print("Select ranging mode:")
                ranging_mode = int(input("\n0: ONE_SENSOR_MODE\n1: TWO_SENSOR_MODE\n2: XOR_SENSOR_MODE\n3: OMNI_SENSOR_MODE\nInput:\t"))
                ranging_mode += 96 #this is the offset for ranging_mode values
                                   #
                num_samples = int(input("Number of samples to take: "))
                
                usrmessage="0"
                usrmessage= usrmessage + str(chr(node_id)) + str(chr(ranging_mode)) + range_req_msg
            except:
                print("Input error")
                save_to_file = False;
                continue
            file = open(file_name, 'w')
            resend = False
            quit = False
            j = 0
            node_id_num = node_id-ord('0')
            for i in range(num_samples):
                time.sleep(0.005)
                client.publish(usrtopic,usrmessage)
                print(str(i)+": publishing "+usrmessage+" to "+usrtopic)
                timeout = time.time() + 3
                while message_rcvd == False:
                    if(time.time() > timeout):
                        j = j+1
                        print("Timed out.. resending "+ str(j))
                        resend = True
                        if(j>5):
                            print("Resending failed.. quitting")
                            quit = True
                        break;

                message_rcvd = False

                if quit:
                    break;
                if resend:
                    resend = False
                    continue  
                j = 0 
                if(node_id_num in rcvd_data.keys()):
                    file.write(str(rcvd_data[node_id_num])+'\n')
                else:
                    file.write('-1')
                

            file.close()
        elif c==4:
            try:
                clientnum = int(input("Enter the number of the openmote you want to range\t"))
                usrtopic=hwaddr[clientnum]

                node_id = int(input("Enter the node_id of the node you with to range (-1 for discovery mode)\t"))
                node_id += ord('0')

                print("Select ranging mode:")
                ranging_mode = int(input("\n0: ONE_SENSOR_MODE\n1: TWO_SENSOR_MODE\n2: XOR_SENSOR_MODE\n3: OMNI_SENSOR_MODE\nInput:\t"))
                ranging_mode += 96 #this is the offset for ranging_mode values
                
                usrmessage="0"
                usrmessage= usrmessage + str(chr(node_id)) + str(chr(ranging_mode)) + range_req_msg
            except:
                print("Input error")
                continue
        elif c==5:
            quit = False
            if not valid_nodelist:
                print("No valid anchor nodelist found")
                continue

            try:
                clientnum = int(input("Enter the number of the openmote you want to range\t"))
                usrtopic=hwaddr[clientnum]
                
                print("Select ranging mode:")
                ranging_mode = int(input("\n0: ONE_SENSOR_MODE\n1: TWO_SENSOR_MODE\n2: XOR_SENSOR_MODE\n3: OMNI_SENSOR_MODE\nInput:\t"))
                ranging_mode += 96 #this is the offset for ranging_mode values

            except:
                print("Input error")
                continue

            #discovery mode
            node_id = -1 + ord('0')
            usrmessage="0"
            usrmessage= usrmessage + str(chr(node_id)) + str(chr(ranging_mode)) + range_req_msg

            client.publish(usrtopic,usrmessage)
            timeout = time.time() + 3
            while message_rcvd == False:
                if(time.time() > timeout):
                    print("Timed out.."+ str(j))
                    break;
            if message_rcvd == False:
                continue;
            else:
                message_rcvd = False
                for anchors in rcvd_data:
                    node_anchor_map[usrtopic][str(anchors)] = -1; #store updated list of local anchors

            i = 0;        
            for anchors in node_anchor_map[usrtopic].keys():
                if anchors in anchor_nodelist.keys(): #check to see if we have an absolute position for this anchor
                    #range individual anchors (eventually we want to allow for ranging multiple anchors in one transmit)
                    node_id = anchors + ord('0')
                    usrmessage="0"
                    usrmessage= usrmessage + str(chr(node_id)) + str(chr(ranging_mode)) + range_req_msg

                    client.publish(usrtopic,usrmessage)
                    timeout = time.time() + 3
                    while message_rcvd == False:
                        if(time.time() > timeout):
                            print("Timed out.."+ str(j))
                            break;
                    if message_rcvd == False:
                        continue;
                    else:
                        i = i+1;
                        message_rcvd = False
                        node_anchor_map[usrtopic][anchors] = rcvd_data[anchors] #store updated list of local anchors
                if i == 3:
                    break

            returnval = triangulate(node_anchor_map[usrtopic])
            print(returnval)
            print(node_anchor_map)


        else:
            print("invalid choice")
            continue;
        if c!=0 and c!=1 and c!=2:
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
    global message_rcvd
    global topic_pub
    global total_num_clients_con_broker
    global connected_clients
    global rcvd_data
    global node_anchor_map

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
                node_anchor_map[topic_pub] = {}
            else:
                print(topic_pub + " already in clients list")
            #when a message is received, the message is published to another topic
            client.publish(topic_pub,server_mqtt[message[0]])   
    elif msg.topic==range_sub:
        rcvd_data = parse_msg(payload)
        print(rcvd_data)
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
    client = mqtt.Client("serv") #creating an instance 
    client.on_connect = on_connect    #on_connect callback
    client.on_message = on_message    #on_message callback 
    client.on_publish = on_publish    #on_publish callback    
    client.on_subscribe =on_subscribe
    #client.tls_set(path_to_certificate)    #establishing the SSL certificate

    try:
        json_data=open('anchor_nodelist.json')
        a_nodelist = json.load(json_data)
        a_nodelist = a_nodelist['list']
        valid_nodelist = True
    except:
        print('An error occured while reading anchor nodelist')
        valid_nodelist = False

    client.connect(broker_address, port)    #connecting to broker

    t = Thread(target=usr_input, args=(client,))
    t.start()   

    #Setting up the subscribe 

    client.subscribe([(topic_sub, 0), (range_sub, 0)])
    client.loop_forever()    #looping forever so it doesn't terminate 

if(__name__=="__main__"):
    main()
