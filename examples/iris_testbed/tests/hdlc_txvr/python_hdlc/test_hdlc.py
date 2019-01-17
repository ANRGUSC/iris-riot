import os
from run_app import *
import threading
from HDLC import HDLC
MAIN_THR_PORT =  1234
THREAD2_PORT  =  5678
PKT_FROM_MAIN_THR =  0
PKT_FROM_THREAD2  =  1

thr1_port = MAIN_THR_PORT
thr2_port = THREAD2_PORT 
app_num1 = PKT_FROM_MAIN_THR
app_num2 = PKT_FROM_THREAD2

thr1_frame_no = 0
thr2_frame_no = 0
thr1 = AppThread(app_num1, thr1_port, thr1_frame_no)
thr2 = AppThread(app_num2, thr2_port, thr2_frame_no)

# sender for parent
sertx = serial.Serial()
sertx.port = '/dev/pts/19'
sertx.baudrate = 9600
sertx.timeout = 0 # non blocking

# receive for parent
serrx = serial.Serial()
serrx.port = '/dev/pts/20'
serrx.baudrate = 9600
serrx.timeout = 0 # non blocking
try:
	sertx.open()
except serial.serialutil.SerialException as e:
	print('[x] Serial tx connection problem : {0}\n'.format(e))
	exit(1)

try:
	serrx.open()
except serial.serialutil.SerialException as e:
	print('[x] Serial rx connection problem : {0}\n'.format(e))
	exit(1)

childid = os.fork()
if childid != 0 :
# parent process
	print ("Parent process %d running..." % os.getpid())
	hdlc_thr = HDLC(0, 0, sertx, serrx)
	hdlc_thr.hdlc_run()
	thr1.apprun(hdlc_thr)
	thr2.apprun(hdlc_thr)
else:
# child process
	print ("Child process %d running..." % os.getpid())
	hdlc_thr = HDLC(0, 0, serrx, sertx)
	hdlc_thr.hdlc_run()
	thr1.apprun(hdlc_thr)
	thr2.apprun(hdlc_thr)


