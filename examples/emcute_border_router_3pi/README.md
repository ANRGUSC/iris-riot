# Emcute Border Router

This example includes a modified version of the gnrc_border_router that can
communicate via UDP. 
To setup the system just load the program and run it.

1. Flash the code to an openmote 
    `make TARGET=openmote-cc2538 flash term`
2. The make script automatically creates a tap interface with address `fd00:dead:beef::1`
3. The openmote automatically connects to the tap interface at `fd00:dead:beef::1`, unless changed
3. To test the code, run `ping6 fd00:dead:beef::1` from the openmote terminal and you should receive a response 
