import paho.mqtt.client as mqtt
import time
from threading import Thread

#initializing the variables
#Change the following according to your system 
broker_address="fd00:dead:beef::1" 
port = 1886
max_nodes = 10

global count

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
msg_type= { 'HW_ADDR' : '0', 'REQUEST' : '1', 'SEND_RSSI' : '2', 'LEN_CLIENTS_LIST' : '3', 'GET_CLIENTS' : '4', 'UDP_SEND' : '5', 'RSSI_DATA' : '9' }

server_mqtt = {'0': 'ACK', '1': 'Do something'}

global message_queue
message_queue = []

global alpha
alpha = 0.9


#Creating the callback functions


def l_search(item):
    c = 0
    for i in client_ID:
        if i == item:
            return c
        c += 1



def data_table():
    global alpha
    global rssi_data
    global w_avg
    rssi_data = [[0]*max_nodes for i in range(max_nodes)]
    w_avg = [[0]*max_nodes for i in range(max_nodes)]
    count = 0  
    overall_count = 0  
    start = False
    l = len(client_ID)
    while(1):  
        if len(message_queue) != 0:
            count += 1
            data = message_queue.pop(0)
            rssi = ord(data[0])
            rcv_addr = data[1:9]
            src_addr = data[9:]
            r_index = l_search(rcv_addr)
            s_index = l_search(src_addr)
            # print("*********************")
            # print("Receiver address",rcv_addr)
            # print("Sender address",src_addr)
            # print("RSSI:",rssi)
            # print(r_index)
            # print(s_index)
            # print("*********************")
            w_avg[r_index][s_index] = (w_avg[r_index][s_index])*(alpha) + (1-alpha)*(rssi - 73)
            rssi_data[r_index][s_index] = rssi - 73
        if count%((l*l)-l) == 0:
            print ("Weighted Table")
            for i in w_avg:
                print(i)
            print(" ")
            print("RSSI values")
            for i in rssi_data:
                print(i)  
            count = 1







def rssi_ping_pong(client):
    print ("Starting RSSI ping pong broadcast")
    count = 0
    while(1):
        #implement round robin system
        if len(client_ID) >=2 :
            for i in range(len(client_ID)):
                t0 = time.time()
                while(1):
                    t1 = time.time()       

                    if ((t1-t0) >= 1 ):
                        print ("RSSI Broadcast message sent")
                        info = client.publish(client_ID[i], msg_type['UDP_SEND'])
                        info.wait_for_publish()
                        count += 1
                        break 
                    

                # time.sleep(0.5)       
            print("count: ",count)
        



def on_connect(client, userdata, rc):
    print ("Connected to broker")


# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    global clients_connected
    global req_clients
    global send_rssi
    global rssi_sender_topic
    global message_queue
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

    elif message[0] == msg_type['RSSI_DATA']:
        message_queue.append(message[1:])
        # print("*********")
        # print(message_queue)
        # print("*********")

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
turn = 0;

t = Thread(target=rssi_ping_pong, args=(client,))
t.start()   

t1 = Thread(target=data_table)
start = True

while 1:
    while(start):
        if len(client_ID) == 2:
            t1.start()
            start = False

    

client.loop_forever() #loop forever