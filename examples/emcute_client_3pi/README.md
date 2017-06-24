## About
This demonstrates how to integrate MQTT-SN with pololu through openmote-cc2538.
**NOTE: This tutorial uses the openmote set to channel 21, please adjust accordingly**

## Server Setup
For using this example, two prerequisites have to be fullfilled:

1. You need a running MQTT broker that supports MQTT-SN or a running MQTT-SN
   gateway that is connected to a running MQTT broker
2. Setting up the border-router. 


### Setting up a broker
This tutorial assumes you have installed MQTT and have it up and running
You can refer to http://www.steves-internet-guide.com/install-mosquitto-broker/ for more
information. 
In the 'conf.d' of '/etc/mosquitto', make sure you comment out the 'default.conf' file.

In general, any MQTT-SN capable broker or broker/gateway setup will do.
Following a quick instruction on how-to setup the Mosquitto Real Simple Message
Broker:

1. Get the RSMB here: https://github.com/eclipse/mosquitto.rsmb:
```
git clone https://github.com/eclipse/mosquitto.rsmb.git
```

2. Go into the source folder and build the RSMB
```
cd mosquitto.rsmb/rsmb/src
make
```

3. Create a config file. In this case we run the RSMB as MQTT and MQTT-SN
   capable broker, using port 8888 for MQTT-SN and 1886 for MQTT and enabling
   IPv6, so save the following to `config.conf`:
```
# add some debug output
trace_output protocol

# listen for MQTT-SN traffic on UDP port 8888
listener 8888 INADDR_ANY mqtts
  ipv6 true

# listen to MQTT connections on tcp port 1886
listener 1886 INADDR_ANY
  ipv6 true
```

4. Start the broker:
```
./broker_mqtts config.conf
```

You can refer to
https://rawgit.com/MichalFoksa/rsmb/master/rsmb/doc/gettingstarted.htm for more
configuration options.

### Setting up the border router

1. Flash the code in `../emcute_border_router` to an openmote 
2. The make script automatically creates a tap interface with address `fd00:dead:beef::1`
3. The openmote automatically connects to the tap interface at `fd00:dead:beef::1`, unless changed
3. To test the code, run `ping6 fd00:dead:beef::1` from the openmote terminal and you should receive a response 

## Client Setup

1. The code automatically connects to `fd00:dead:beef::1` and 
subscribes to the topic `test/trial` by using two functions 
`auto_con()` and `auto_sub()`
2. The code also adds a function `on_pub_mbed()`, which receives the incoming 
packets and pushes it through the UART
**NOTE: All the information received when subscribed to "test/trial" is 
sent through the UART to the pololu**
3. The code also automatically sets up a unique client ID based on the hardware address of the openmote.
This ID is used by the MQTT-SN broker to identify individual motes.

### Setting up the openmote

1. Flash the code on the openmote.
2. Take the openmote and connect it to the pololu robots.
3. The code should be up and running
4. Download the code at 'https://github.com/ANRGUSC/leader-m3pi/tree/ec65ac91f84b22a4bac05ab3b9dab2d41dd4c32b' and flash it to the pololu robot
5. In order to test the functionality of the set up, open up terminal on the linux 
system that you have the MQTT-SN Broker running on and enter the following code:
```
mosquitto_pub -h fd00:dead:beef::1 -t "test/trial" -p 1886 -m "wwwwwww"
```
6. If the pololu robot was configured to move, the robot should move forward for 1-2secs
