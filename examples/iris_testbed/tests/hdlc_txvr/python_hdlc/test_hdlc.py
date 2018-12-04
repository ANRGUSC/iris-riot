import os
from run_app import *
import threading
#from hdlc_nooob import hdlc_tx, hdlc_rx
from hdlc import hdlc
MAIN_THR_PORT =  1234
THREAD2_PORT  =  5678
PKT_FROM_MAIN_THR =  0
PKT_FROM_THREAD2  =  1

thr1_port = MAIN_THR_PORT
thr2_port = THREAD2_PORT 
appnum1 = PKT_FROM_MAIN_THR
appnum2 = PKT_FROM_THREAD2

thr1_frame_no = 0
thr2_frame_no = 0
#stdoutmutex = threading.Lock() # for printing to stdout only
#hdlcmutex = threading.Lock()

#hdlc_reg = []

#thr1 = AppThread(appnum1, thr1_port, thr1_frame_no, stdoutmutex)
#thr2 = AppThread(appnum2, thr2_port, thr2_frame_no, stdoutmutex)
thr1 = AppThread(appnum1, thr1_port, thr1_frame_no)
thr2 = AppThread(appnum2, thr2_port, thr2_frame_no)

#entry1 = hdlc_entry(thr1_port, thr1.get_rbox(), thr1.get_tbox())
#entry2 = hdlc_entry(thr2_port, thr2.get_rbox(), thr2.get_tbox())

# can done it here
# tmp for pipe 12/1 # hdlctxthr = threading.Thread(target=hdlc.hdlc_tx)		
# tmp for pipe 12/1 # hdlcrxthr = threading.Thread(target=hdlc.hdlc_rx)		

p1pipein, p1pipeout = os.pipe()
p2pipein, p2pipeout = os.pipe()

childid = os.fork()
if childid != 0 :
# parent proc1
	print ("Parent process %d running..." % os.getpid())
	os.close(p1pipein)  # proc1 will write to p1pipeout
	os.close(p2pipeout) # proc1 will read from p2pipein
	p1pipeout = os.fdopen(p1pipeout, 'w')
	p2pipein  = os.fdopen(p2pipein, 'r')
	#nooob#hdlctxthr1 = threading.Thread(target=hdlc_tx, args=(p1pipeout,))
	#nooob#hdlcrxthr1 = threading.Thread(target=hdlc_rx, args=(p2pipein,))

	#nooob#hdlctxthr1.start()
	#nooob#hdlcrxthr1.start()
	
	hdlcthr = hdlc(0, 0, p2pipein, p1pipeout)
	#hdlcthr.hdlc_register(entry1)	
	#hdlcthr.hdlc_register(entry2)	
	hdlcthr.hdlc_run()
	thr1.apprun(hdlcthr)
	thr2.apprun(hdlcthr)
	#nooob#hdlctxthr1.join()
	#nooob#hdlcrxthr1.join()
	#thr1.join()
	#thr2.join()
#	while thr1.isRunning or thr2.isRunning:
#		pass
else:
	print ("Child process %d running..." % os.getpid())
	os.close(p2pipein)  # proc2 will read from p1pipein
	os.close(p1pipeout) # proc2 will write to p2pipeout
	p2pipeout = os.fdopen(p2pipeout, 'w')
	p1pipein  = os.fdopen(p1pipein, 'r')
	#nooob#hdlctxthr2 = threading.Thread(target=hdlc_tx, args=(p2pipeout,))
	#nooob#hdlcrxthr2 = threading.Thread(target=hdlc_rx, args=(p1pipein,))

	#nooob#hdlctxthr2.start()
	#nooob#hdlcrxthr2.start()
	
	hdlcthr = hdlc(0, 0, p1pipein, p2pipeout)
	#hdlcthr.hdlc_register(entry1)	
	#hdlcthr.hdlc_register(entry2)	
	hdlcthr.hdlc_run()
	thr1.apprun(hdlcthr)
	thr2.apprun(hdlcthr)
	#nooob#hdlctxthr2.join()
	#nooob#hdlcrxthr2.join()
	#thr1.join()
	#thr2.join()


