import paho.mqtt.client as mqtt
import time
from threading import Thread

#initializing the variables
#Change the following according to your system 
broker_address="fd00:dead:beef::1" 
port = 1886

global max_nodes
max_nodes = 10

global alpha
alpha = 0.9

nodes_to_start_rssi_ping_pong = 2

topic_sub = "init_info"

#Initializing the global variables that are used in different threads
global clients_connected
clients_connected = 0

global topic_pub 
topic_pub = ""

global client_ID
client_ID=[]

msg_type= { 'HW_ADDR' : '0', 'REQUEST' : '1', 'SEND_RSSI' : '2', 'LEN_CLIENTS_LIST' : '3', 'GET_CLIENTS' : '4', 'UDP_SEND' : '5', 'RSSI_DATA' : '9' }

server_mqtt = {'0': 'ACK', '1': 'Do something'}

global rssi_data_queue
rssi_data_queue = []

global mqtt_queue
mqtt_queue = []


# Linear search to help organize the data

def l_search(item):
    c = 0
    for i in client_ID:
        if i == item:
            return c
        c += 1


#thread to handle the organization of the data
# def data_table():
#     global alpha
#     global rssi_data
#     global w_avg
#     rssi_data = [[0]*max_nodes for i in range(max_nodes)]
#     w_avg = [[0]*max_nodes for i in range(max_nodes)]
#     count = 0  
#     overall_count = 0  
#     check = False
#     l = len(client_ID)
#     while(1):  
#         if len(rssi_data_queue) != 0:
#             count += 1
#             overall_count +=1
#             check = True
#             data = rssi_data_queue.pop(0)
#             rssi = ord(data[0])
#             rcv_addr = data[1:9]
#             src_addr = data[9:]
#             r_index = l_search(rcv_addr)
#             s_index = l_search(src_addr)

#             if (overall_count == 1 or overall_count == 2):
#                 w_avg[r_index][s_index] = (rssi - 73)
#             else:
#                 w_avg[r_index][s_index] = (w_avg[r_index][s_index])*(alpha) + (1-alpha)*(rssi - 73)
#             rssi_data[r_index][s_index] = rssi - 73
#         if count%((l*l)-l) == 0 and check:
#             print ("Weighted Table")
#             for i in w_avg:
#                 print(i)
#             print(" ")
#             print("RSSI values")
#             for i in rssi_data:
#                 print(i)  
#             #count = 0
#             check = False

def data_table():
    global alpha
    global rssi_data
    global w_avg
    l = len(client_ID)
    rssi_data = [[0]*l for i in range(l)]
    w_avg = [[0]*l for i in range(l)]
    count = 0   
    check = False    
    while(1):  
        if len(rssi_data_queue) != 0:
            count += 1
            check = True
            data = rssi_data_queue.pop(0)
            rssi = ord(data[0])
            rcv_addr = data[1:9]
            src_addr = data[9:]
            r_index = l_search(rcv_addr)
            s_index = l_search(src_addr)

            if (count == 1 or count == 2):
                w_avg[r_index][s_index] = (rssi - 73)
            else:
                w_avg[r_index][s_index] = (w_avg[r_index][s_index])*(alpha) + (1-alpha)*(rssi - 73)
            rssi_data[r_index][s_index] = rssi - 73
        if count%((l*l)-l) == 0 and check:
            l = len(client_ID)
            # print(len(rssi_data))
            w_avg_temp = [[0]*l for i in range(l)]
            for row in range(len(w_avg)):
                for col in range(len(w_avg[row])):
                    w_avg_temp[row][col] = w_avg[row][col]
            w_avg = w_avg_temp
            print ("Weighted Table")
            for i in w_avg:
                print(i)
            print(" ")
            print("RSSI values")
            for i in rssi_data:
                print(i)  
            check = False




def rssi_ping_pong(client):
    print ("Starting RSSI ping pong broadcast")
    count = 0    
    while(1):
        #implement round robin system
        l = len(client_ID)
        for i in range(l):
            # t0 = time.time()
            time.sleep(1)
            print ("RSSI Broadcast message sent")
            info = client.publish(client_ID[i], msg_type['UDP_SEND'])
            info.wait_for_publish()
            count += 1
       
        print("count: ",count)
        



def on_connect(client, userdata, rc):
    print ("Connected to broker")


# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    global mqtt_queue
    print ("Data received")
    try :     
        message = str(msg.payload.decode())
        mqtt_queue.append(message)
        
    except UnicodeDecodeError:
        print ("invalid packet")
# Processes the mqtt messages  
def handle_mqtt_message(client):
    global mqtt_queue
    global client_ID
    global topic_pub
    global clients_connected
    if (len(mqtt_queue) != 0):
        message = mqtt_queue.pop(0)
        if message[0] == msg_type['HW_ADDR']:
            topic_pub = message[1:]
            print("The topic is \n", topic_pub)
            if topic_pub not in client_ID:
                client_ID.append(topic_pub)
                clients_connected += 1
            client.publish(topic_pub,server_mqtt[message[0]])                
            print("The number of clients connected is", clients_connected)
            print("The clients are", client_ID)

        elif message[0] == msg_type['REQUEST']:
            print ("client requested received")

        elif message[0] == msg_type['RSSI_DATA']:
            rssi_data_queue.append(message[1:])

        else:
            print ("wrong format %d\n" % int(message[0]))

#callback function to handle subscribing to the broker
def on_subscribe(mosq, obj, mid, granted_qos):
   print("Subscribed to all topics ")

#function called when a message is published
def on_publish(client,userdata,result):     
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


client.loop_start()

#Handling the rssi round robin in a separate thread
rssi_ping_pong_thread = Thread(target=rssi_ping_pong, args=(client,))

#Handling the rssi data in a separate thread
rssi_data_thread = Thread(target=data_table)

check = True

#infinite loop that starts the rssi and calls 
#handle_mqtt_message when a message is received
while (1):
    if (len(client_ID) == nodes_to_start_rssi_ping_pong and check):        
        print(client_ID)
        time.sleep(2)
        rssi_ping_pong_thread.start() 
        rssi_data_thread.start()
        check = False

    if len(mqtt_queue) != 0:
        handle_mqtt_message(client)

            

    

client.loop_forever() #loop forever