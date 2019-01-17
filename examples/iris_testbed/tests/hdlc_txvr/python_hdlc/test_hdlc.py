""" 
Copyright (c) 2019, Autonomous Networks Research Group. All rights reserved.

Developed by:
Autonomous Networks Research Group (ANRG)
University of Southern California
http://anrg.usc.edu/

Contributors:
Dennis Wang
Jason A. Tran

Permission is hereby granted, free of charge, to any person obtaining a copy of 
this software and associated documentation files (the "Software"), to deal with 
the Software without restriction, including without limitation the rights to 
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so, 
subject to the following conditions:

- Redistributions of source code must retain the above copyright notice, this 
  list of conditions and the following disclaimers.
- Redistributions in binary form must reproduce the above copyright notice, this
  list of conditions and the following disclaimers in the documentation and/or 
  other materials provided with the distribution.
- Neither the names of Autonomous Networks Research Group, nor University of 
  Southern California, nor the names of its contributors may be used to endorse 
  or promote products derived from this Software without specific prior written 
  permission.
- A citation to the Autonomous Networks Research Group must be included in any 
  publications benefiting from the use of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE CONTRIBUTORS
OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, 
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS WITH THE SOFTWARE.

Test script for pre-alpha HDLC implementation.
"""

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


