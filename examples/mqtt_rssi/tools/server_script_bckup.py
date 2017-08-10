from __future__ import print_function
import paho.mqtt.client as mqtt
import time


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
server_mqtt = {'0': 'ACK', '1': 'REQUEST', '2': 'SEND_RSSI'}
hwaddr=[]
global total_num_clients_con_broker
total_num_clients_con_broker=0
global client_req
global rssi_sender_topic
client_req=0
global connected_clients
connected_clients=0
global rssi_sender_topic
global send_rssi
send_rssi=0
global published_clients
published_clients=0

#Creating the callback functions 


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
    global client_req
    global rssi_sender_topic
    global send_rssi
    print ("Data received")
    message = str(msg.payload.decode())
    if msg.topic==topic_sub:        
            if message[0] == '0':
                topic_pub=message[1:]
                if topic_pub in hwaddr:
                    print("already registered")
                else:
                    hwaddr.append(topic_pub)
                #when a message is received, the message is published to another topic
                client.publish(topic_pub,server_mqtt[message[0]]) 
            elif message[0] == '1':
                client_req=client_req+1 
                print("clients requested",client_req)
            elif message[0] == '2':
                print("RSSI_PUB received")              
                rssi_sender_topic=message[1:]
                print("***********",rssi_sender_topic,"**********")
                send_rssi=1
                for i in range(1000000):
                    pass

    print("The clients connected", hwaddr)
    connected_clients=len(hwaddr)

      
def on_subscribe(mosq, obj, mid, granted_qos):
    print("Subscribed to all topics ")

def on_publish(client,userdata,result): 
    global connected_clients
    print("Data published to topic", topic_pub) 
    print("The number of connected clients:", connected_clients)
    #check to ensure that all clients have connected
    #For this test it assumes there are only 2 clients
    


        
 
#Creating an instance and setting up the callbacks 

client = mqtt.Client() #creating an instance 
client.on_connect = on_connect  #on_connect callback
client.on_message = on_message  #on_message callback 
client.on_publish = on_publish  #on_publish callback    
client.on_subscribe =on_subscribe
#client.tls_set(path_to_certificate)    #establishing the SSL certificate
client.connect(broker_address, port)
client.subscribe([(topic_sub, 0), (topic_client_con, 0), (topic_client_id,0)])  #connecting to broker
client.loop_start()
count=0
while (1):
    if client_req==2:
        for i in range(2):
            len_data="3"+str(client_req)
            infoc=client.publish(hwaddr[i],len_data)
            infoc.wait_for_publish()
        time.sleep(1)
        for i in range(2):
            for j in range(2):
                data = "4"+hwaddr[j]
                info=client.publish(hwaddr[i], data)
                info.wait_for_publish()
                for c in range(10000000):
                    pass
            for a in range(10000000):
                    pass
            print ("Sent to hardware address", hwaddr[i])
        published_clients=1
        client_req=0
    if send_rssi==1 and published_clients:
        print("send the send_rssi command")
        for i in range(1000000):
            pass
        for i in range(connected_clients):
            data_pub=str(hwaddr[i])
            if hwaddr[i]==rssi_sender_topic:
                print("hwaddr is the rssi sender topic")
            else:
                print("The topic to pub to is",data_pub)
                topic_pub=data_pub
                infop=client.publish(data_pub,"5")
                infop.wait_for_publish()
            for j in range(1000000):
                pass
        for j in range(1000000):
                pass
        count=count+1
        print("the count is", count)
        send_rssi=0


client.loop_forever()   #looping forever so it doesn't terminate 
