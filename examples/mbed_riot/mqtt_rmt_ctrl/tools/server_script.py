import paho.mqtt.client as mqtt
import time
from threading import Thread
import curses
#initializing the variables
#Change the following according to your system 
broker_address="fd00:dead:beef::1" 
port = 1886

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

client_ID = []
topic_sub = "init_info"
topic_rmt = "test/trial"

toggle_client = []
toggle_client_value = [True] * 1

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
    stdscr.nodelay(1)
    stdscr.clear()
    while True:
        # get keyboard input, returns -1 if none available        
        c = stdscr.getch()

        #if a key has been pressed
        if c != -1:
            stdscr.clear()
            # print numeric value
            if str(c) == "97": #value of a
                #if all nodes are active send to shared topic
                if (toggle_client_value == [True] * 1):
                    client.publish(topic_rmt,"8a")
                #sending to a subset of nodes(active)
                else :
                    for i in range (len(toggle_client_value)):
                        if toggle_client_value[i] == True:
                            client.publish(toggle_client[i],"8a")
                stdscr.addstr("left")

            elif str(c) =="115": #value of s
                #if all nodes are active send to shared topic
                if (toggle_client_value == [True] * 1):
                    client.publish(topic_rmt,"8s")
                #sending to a subset of nodes(active)
                else :
                    for i in range (len(toggle_client_value)):
                        if toggle_client_value[i] == True:
                            client.publish(toggle_client[i],"8s")
                stdscr.addstr("backward")

            elif str(c) =="100": #value of d
                #if all nodes are active send to shared topic
                if (toggle_client_value == [True] * 1):
                    client.publish(topic_rmt,"8d")
                #sending to a subset of nodes(active)
                else :
                    for i in range (len(toggle_client_value)):
                        if toggle_client_value[i] == True:
                            client.publish(toggle_client[i],"8d")
                stdscr.addstr("right")

            elif str(c) == "119": #value of w
                #if all nodes are active send to shared topic
                if (toggle_client_value == [True] * 1):
                    client.publish(topic_rmt,"8w")
                #sending to a subset of nodes(active) 
                else :
                    for i in range (len(toggle_client_value)):
                        if toggle_client_value[i] == True:
                            client.publish(toggle_client[i],"8w")
                stdscr.addstr("forward")  

            #handles toggling nodes on or off 
            elif str(c) == "49":  #use '1' on the keyboard to toggle the first node
                toggle_client_value[((int(str(c)))-49)] = not toggle_client_value[((int(str(c)))-49)]
                stdscr.addstr("toggle 1st node")
            elif str(c) == "50": #use '2' on the keyboard to toggle the second node
                toggle_client_value[((int(str(c)))-49)] = not toggle_client_value[((int(str(c)))-49)]
                stdscr.addstr("toggle 2nd node")
            elif str(c) == "51": #use '3' on the keyboard to toggle the third node
                toggle_client_value[((int(str(c)))-49)] = not toggle_client_value[((int(str(c)))-49)]
                stdscr.addstr("toggle 3rd node")
            elif str(c) == "52": #use '4' on the keyboard to toggle the fourth node
                toggle_client_value[((int(str(c)))-49)] = not toggle_client_value[((int(str(c)))-49)]
                stdscr.addstr("toggle 4th node")    
            elif str(c) == "53": #use '5' on the keyboard to toggle the fifth node
                toggle_client_value[((int(str(c)))-49)] = not toggle_client_value[((int(str(c)))-49)]
                stdscr.addstr("toggle 5th node")
            #handles exitting the curses screen
            elif str(c) == "113": #value of 'q'
                curses.endwin()
                break

            else:
                continue
            stdscr.refresh()

            # return curser to start position
            stdscr.move(0, 0)


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


while 1:

    if (clients_connected == 1 and rmt_ctrl_start):
        time.sleep(2)
        print ("Ready to start")
        for i in range(clients_connected):
            client.publish(client_ID[i],data_pub)
            time.sleep(0.5)
        print ("Use \'q\' to exit the curses screen")
        time.sleep(5)
        toggle_client = client_ID
        print("execute ncurses main")
        rmt_ctrl_start = False   
        rmt_go = 1
        


client.loop_forever() #loop forever