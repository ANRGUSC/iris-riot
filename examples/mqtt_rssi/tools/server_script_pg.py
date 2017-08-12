import paho.mqtt.client as mqtt
import time

#initializing the variables
#Change the following according to your system 
broker_address="fd00:dead:beef::1" 
port = 1886

global clients_connected
clients_connected = 0

global req_clients
req_clients = 0

global topic_pub 
topic_pub = ""

global rssi_sender_topic
rssi_sender_topic = ""

global send_rssi
send_rssi = 0
client_ID=[]
topic_sub = "init_info"
msg_type= { 'HW_ADDR' : '0', 'REQUEST' : '1', 'SEND_RSSI' : '2', 'LEN_CLIENTS_LIST' : '3', 'GET_CLIENTS' : '4', 'UDP_SEND' : '5'}

server_mqtt = {'0': 'ACK', '1': 'Do something'}

#Creating the callback functions

# The callback for when the client receives a CONNACK response from the server.
# The callback function when client is connected to the broker.
def time_wait(count):
    #delay for 1 second * count
    start = time.time()
    for i in range (count):
        for j in range (53631579):
            pass
    end = time.time()
    return (end - start)

def time_wait_ds(count):
    #delay for 1 decisecond * count
    start = time.time()
    for i in range (count):
        for j in range (5363157):
            pass
    end = time.time()
    return (end - start)

def on_connect(client, userdata, rc):
    print ("Connected to broker")


# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    global clients_connected
    global req_clients
    global send_rssi
    global rssi_sender_topic
    print ("Data received")
    message = str(msg.payload.decode())
    if message[0] == msg_type['HW_ADDR']:
        topic_pub = message[1:]
        print("The topic is \n", topic_pub)
        if topic_pub not in client_ID:
            client_ID.append(topic_pub)
            clients_connected += 1
        # elif topic_pub in client_ID:
        #     if clients_connected != len(client_ID):
        #         clients_connected += 1

        #when a message is received, the message is published to another topic
        client.publish(topic_pub,server_mqtt[message[0]])                
        print("The number of clients connected is", clients_connected)
        print("The clients are", client_ID)

    elif message[0] == msg_type['REQUEST']:
        print ("client requested received")
        req_clients += 1

    elif message[0] == msg_type['SEND_RSSI']:
        print ("*******RSSI_SEND*******")
        rssi_sender_topic = message[1:]
        send_rssi = 1

    else:
        print ("wrong format %d\n" % int(message[0]))
                                        

def on_subscribe(mosq, obj, mid, granted_qos):
   print("Subscribed to all topics ")


def on_publish(client,userdata,result):     #create function for callback
   #print("Data published to topic", topic_pub)
   pass

#Creating an instance and setting up the callbacks

client = mqtt.Client() #creating an instance 
client.on_connect = on_connect    #on_connect callback
client.on_message = on_message    #on_message callback 
client.on_publish = on_publish    #on_publish callback    
client.on_subscribe = on_subscribe
#client.tls_set(path_to_certificate)    #establishing the SSL certificate
client.connect(broker_address, port)    #connecting to broker

#Setting up the subscribe 
client.subscribe(topic_sub)
data_pub = ""
client.loop_start()
while 1:
    '''
    if req_clients == 2:
        print(time_wait_ds(1))
    '''
    if req_clients == 2:
        for i in range(2):
            len_data = msg_type['LEN_CLIENTS_LIST'] + str(req_clients)
            topic_pub = client_ID[i]
            infoc = client.publish(client_ID[i] , len_data)
            infoc.wait_for_publish()
        time_wait(2)
        for i in range(req_clients):
            for j in range(req_clients):
                data_pub = msg_type['GET_CLIENTS'] + client_ID[j]
                print("the data to be sent is data_pub", data_pub)
                print("the topic is", client_ID[i])
                infop = client.publish(client_ID[i], data_pub)
                infop.wait_for_publish()
                time_wait_ds(5)
            time_wait_ds(5)
        req_clients = 0

    if send_rssi == 1:
        print ("the rssi topic to send is not", rssi_sender_topic)
        for i in range(clients_connected):
            if client_ID[i] != rssi_sender_topic:
                info = client.publish(client_ID[i], msg_type['UDP_SEND'])
                info.wait_for_publish()
                break;
            time_wait_ds(3)
        send_rssi = 0