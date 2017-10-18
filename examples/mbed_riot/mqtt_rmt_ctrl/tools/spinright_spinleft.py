import paho.mqtt.client as mqtt  #import the client1
import time

#Tap interface address
broker_address="fd00:dead:beef::1"
#port for MQTT
port = 1886
#topic robot movement 
topic = "test/trial"

def on_connect(client, userdata, flags, rc):  
    #When the python script connects to the mosquitto broker, execute this callback
    print ("Connected to Broker")

def on_message(client, userdata, message):
    #When a message has been received, execute this callback
    pass


def on_publish(client,userdata,result):     
    #When the python script publishes a message, execute this callback
    pass

#rotation function
def rotate(direction):
    if direction:   
        info = client.publish(topic, "8a") #rotates left
        info.wait_for_publish()
    else:
        info = client.publish(topic, "8d") #rotates right 
        info.wait_for_publish()


client = mqtt.Client()    #create new instance
client.on_connect= on_connect        #attach function to callback
client.on_message=on_message        #attach function to callback
client.connect(broker_address,port)      #connect to broker
client.on_publish = on_publish
client.loop_start()    #start the loop
     
for i in range(20): #Number of rotations left
    rotate(1)
    time.sleep(0.05) #Delay between the messages


time.sleep(1)
for j in range(20): #Number of rotations right
    rotate(0)
    time.sleep(0.05) #delay between messages
    
client.loop_stop()