import paho.mqtt.client as mqtt
import time
from threading import Thread
import curses
#initializing the variables
#Change the following according to your system 
broker_address="fd00:dead:beef::1" 
port = 1886

#Handles the number of nodes that will be connected before the remote control

global count

global clients_connected
clients_connected = 0

global topic_pub 
topic_pub = ""

global data_pub
data_pub = "1test/trial"

global rmt_go
rmt_go = 0

global rmt_ctrl_start
rmt_ctrl_start = True

global exit
exit = True

client_ID = []
topic_sub = "init_info"
topic_rmt = "test/trial"

msg_type= { 'HW_ADDR' : '0', 'REQUEST' : '1', 'SEND_RSSI' : '2', 'LEN_CLIENTS_LIST' : '3', 'GET_CLIENTS' : '4', 'UDP_SEND' : '5'}

server_mqtt = {'0': 'ACK', '1': 'Do something'}

def rmt_ctrl():
    while 1:        
        if rmt_go == 1:        
            if __name__ == '__main__':
                    curses.wrapper(main)
                    break

def main(stdscr):
    # use 'q' to quit the application 
    # do not wait for input when calling getch
    global client
    global exit
    stdscr.nodelay(1)
    stdscr.clear()
    stdscr.addstr("Press \'q\' to quit")
    while True:
        # get keyboard input, returns -1 if none available        
        c = stdscr.getch()


        if c != -1:
            stdscr.clear()
            stdscr.addstr("Press \'q\' to quit")
            stdscr.addstr("\n")            

            if str(c) == "97": 
                client.publish(topic_rmt,"8a")  
                stdscr.addstr("left")        

            elif str(c) =="115":              
                client.publish(topic_rmt,"8s")   
                stdscr.addstr("backward")             

            elif str(c) =="100":  
                client.publish(topic_rmt,"8d") 
                stdscr.addstr("right")               

            elif str(c) == "119":                 
                client.publish(topic_rmt,"8w")
                stdscr.addstr("forward")

            elif str(c) == "113":
                break;

            else:
                continue
            stdscr.refresh()

            # return curser to start position
            stdscr.move(0, 0)
    exit = False
    client.loop_stop();



#Creating the callback functions

# The callback for when the client receives a CONNACK response from the server.
# The callback function when client is connected to the broker.

def on_connect(client, userdata, rc):
    print ("Connected to broker")


# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    global clients_connected
    global data_pub
    print ("Data received")
    message = str(msg.payload.decode())
    if message[0] == msg_type['HW_ADDR']:
        topic_pub = message[1:]
        print("The topic is \n", topic_pub)
        if topic_pub not in client_ID:
            client_ID.append(topic_pub)
            clients_connected += 1

        #when a message is received, the message is published to another topic
        client.publish(topic_pub,server_mqtt[message[0]])  
        if (rmt_go == 1):
            client.publish(client_ID[(len(client_ID)-1)], data_pub)              
        print("The number of clients connected is", clients_connected)
        print("The clients are", client_ID)
    else:
        print ("wrong format %d\n" % int(message[0]))
                                        

def on_subscribe(mosq, obj, mid, granted_qos):
   print("Subscribed to all topics ")


def on_publish(client,userdata,result):     #create function for callback
   #print("Data published to topic", topic_pub)
   pass

#Creating an instance and setting up the callbacks
global client
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

#starting thread for remote control 
t = Thread(target=rmt_ctrl)
t.start()  


while (exit):

    if (rmt_ctrl_start and len(client_ID) == 1):
        time.sleep(1)
        print ("Ready to start")        
        info = client.publish(client_ID[0],data_pub)
        info.wait_for_publish()
        print ("Use \'q\' to exit the curses screen")
        time.sleep(2)
        print("execute ncurses main")
        rmt_ctrl_start = False   
        rmt_go = 1
        

