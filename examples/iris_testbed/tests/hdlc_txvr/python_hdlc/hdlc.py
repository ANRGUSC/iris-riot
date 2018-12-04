#import _thread 
import queue
import threading # can join thread
#from _thread import Queue
#from Queue import *
from enum import Enum
from yahdlc import *
import serial
from sys import stdout, stderr
import struct

# for pipe testing
import os

# used to find out subscriber
#typedef struct hdlc_entry {
#    struct hdlc_entry *next;
#    uint16_t port;
#    Mail<msg_t, HDLC_MAILBOX_SIZE> *mailbox;
#} hdlc_entry_t;

class hdlc_entry:
	def __init__(self, port=0, rbox=None, tbox=None):
		self.port = port
		self.rbox = rbox
		self.tbox = tbox

# used to get port number from uart packet
#typedef struct __attribute__((packed)) {
#    uint16_t    src_port;      
#    uint16_t    dst_port;      
#    uint8_t     pkt_type; // this is for app use - help to determine which thread to communicate
#} uart_pkt_hdr_t;

class uart_pkt_hdr:
	def __init__(self, src_port=0, des_port=0, ptype=""):
		self.src_port = src_port
		self.des_port = des_port
		self.ptype = ptype 
#typedef struct {
#    osThreadId sender_pid;    
#    void *source_mailbox;
#    uint16_t type;              /**< Type field. */
#    union {
#        void *ptr;              /**< Pointer content field. */
#        uint32_t value;         /**< Value content field. */
#        float line;
#    } content;                  /**< Content of the message. */
#} msg_t;

# pipe setup 
# file descriptor
#pipein = 0
#pipeout = 0
# pipe setup end

# Serial port configuration
# for real test
# tmp for pipe 12/1 #ser = serial.Serial(
# tmp for pipe 12/1 #port = '/dev/ttyUSB0', # TODO: check how to split it into TX, RX?
# tmp for pipe 12/1 #baudrate = 115200, # TODO: check HDLC_BAUDRATE in mbed github
# tmp for pipe 12/1 #timeout = 0, #None, # blocking read # TODO: does it mean TX would be blocked as well? yes, so change to nonblocking (we can't keep holding uart since others may have data to send
# tmp for pipe 12/1 #write_timeout = 0) # nonblocking write  
# tmp for pipe 12/1 #try:
# tmp for pipe 12/1 #	ser.open()
# tmp for pipe 12/1 #except serial.SerialException as e:
# tmp for pipe 12/1 #	stderr.write('[x] Serial connection problem : {0}\n'.format(e))
# tmp for pipe 12/1 #	exit(1)

# global vars
HDLC_RTRY_TIMEO_USEC  = 200000

#class#send_seq_no = 0
#class#recv_seq_no = 0
#class#hdlc_reg = [] # will be accessed by rx thread only
#not used#sender_inbox = None
#not used#sender_id = 0
# mutex is used to access global variable send_seq_no, recv_seq_no, mailboxes and uart
#mutex = thread.allocate_lock() # import _thread as thread?
#class#mutex = threading.Lock()
#class#hdlcInbox = queue.Queue()
#class#hdlcOutbox = queue.Queue()

#list = []
#enumerate()
class HDLCMT(Enum):
	HDLC_MSG_RECV = 0
	HDLC_MSG_SND = 1
	HDLC_MSG_SND_ACK = 2
	HDLC_MSG_RESEND = 3
	HDLC_RESP_SND_SUCC = 4
	HDLC_PKT_RDY = 5
	HDLC_RESP_RETRY_W_TIMEO = 6
	HDLC_NONE = 7

# type, content, sender id, sender mailbox, 
class hdlcmsg:
	def __init__(self, mailtype=HDLCMT.HDLC_NONE, msg=b'', srcid=0, srcbox=None): # srcbox will point to corresponding queue
		self.type = mailtype
		self.msg = msg
		self.srcid = srcid
		self.srcbox = srcbox

class hdlc:
	def __init__(self, send_seq_no=0, recv_seq_no=0, ifobj=None, ofobj=None):
		self.send_seq_no = send_seq_no
		self.recv_seq_no = recv_seq_no
		self.hdlc_reg = []
#		self.mutex = mutex
		self.mutex = threading.Lock() # 
		self.hdlcInbox = queue.Queue()
		self.hdlcOutbox = queue.Queue()
		self.txthr = threading.Thread(target=self.hdlc_tx, args=(ofobj,)) 
		self.rxthr = threading.Thread(target=self.hdlc_rx, args=(ifobj,)) 
		self.isRunning = 0

	def hdlc_run(self):
		self.isRunning = 1
		self.txthr.start()
		self.rxthr.start()
		
	# tmp # def hdlc_tx(self): # hdlc version
	def hdlc_tx(self, ser):
		#with self.mutex:
		print ("hdlc thread %d tx starts running\n" % threading.get_ident())
	# in general, if serial is busy, create a RESEND msg and put it back to hdlc outbox
	#
	# . 
	# . get message from Outbox (can be blocked)
	# . once got a msg, check type
	# . - MSG_SND: if serial not busy, send (reset timer); otherwise, create a RETRY msg in sender's mailbox
	# . - MSG_SND_ACK: send ACK if serial not busy; (added: otherwise, create a RESEND msg in hdlc mailbox)
	# . - MSG_RESEND: send the msg again if serial not busy (reset timer); (added: otherwise, create a RESEND msg in hdlc mailbox)
		# tmp for pipe 12/1 #global send_seq_no, hdlcOutbox, ser, mutex
		# global send_seq_no, hdlcOutbox, mutex
	
		while True:
			mail = self.hdlcOutbox.get(block=True) # here we can use blocking to see if we have anything to send (will be removed automatically from the queue)
# here we use blocking call so that if nothing in mailbox, we don't need to wake up to send unlike existing C implementation
# TODO NOTE: if we use nonblocking, will need to create a HDLC_MSG_RESEND, and continue to next iteration
	
			#debug 
			print ("hdlc tx: processing type %s msg %s from %d\n" % (mail.type, mail.msg, mail.srcid))
			#debug end

			if mail.type == HDLCMT.HDLC_MSG_SND:
				#nonblocking#try: 
				#nonblocking#	ser.write(mail)
				#nonblocking#except serial.SerialTimeoutException as e:
	# can use blocking write & timeout! follow C implementation for now
					sender_pid = mail.srcid
					print ("hdlc tx: waiting for mutex before sending\n") #debug
					#with self.mutex: # for uart and seq_no
					self.mutex.acquire()
					seq_no = self.send_seq_no % 8
					print ("hdlc: request to send %s from pid %d\n" % (mail.msg, sender_pid))
					if ser.write(frame_data(mail.msg, FRAME_DATA, seq_no)) == 0: 
					#if ser.write(mail.msg) == 0: # pipe
						self.mutex.release()
						print ("hdlc: uart locked, telling thr %d to retry\n" % sender_pid)
						# notify the sender
						#bytesobj = pack('I', HDLC_RTRY_TIMEO_USEC)
						newMail = hdlcmsg(HDLCMT.HDLC_RESP_RETRY_W_TIMEO, HDLC_RTRY_TIMEO_USEC, mail.srcid, mail.srcbox)
						try:
							mail.srcbox.put(newMail, block=False) 
						except queue.Full as e:
							print ("hdlc: no space in source thread (%d) outbox. ERROR!!\n" % sender_pid)
							ser.close()
							exit(0)
					else:
						self.mutex.release()
						print ("hdlc: sender_pid set to %d\n" % sender_pid)
						print ("hdlc: sending frame seq no %d\n" % (seq_no)) #, len(mail.msg))
			elif mail.type == HDLCMT.HDLC_MSG_SND_ACK:
				#with self.mutex: # for uart
					self.mutex.acquire()
					if ser.write(frame_data(b'', FRAME_DATA_ACK, mail.msg)) == 0: # this mail is from rx thread who put the seq no in msg
					#if ser.write("ACK") == 0: # pipe
						self.mutex.release()
						print ("hdlc: uart locked, telling hdlc tx thread to retry\n") 
						try:
							self.hdlcOutbox.put(mail, block=False)
						except queue.Full as e:
							print ("hdlc: no space in hdlc thread outbox. ERROR!!\n")
							ser.close()
							exit(0)
					else:
						self.mutex.release()
						print ("hdlc: sending ack w/ seq no %d\n" % mail.msg) 
			elif mail.type == HDLCMT.HDLC_MSG_RESEND:
				#with self.mutex: # for uart
					self.mutex.acquire()
					if ser.write(frame_data(mail.msg, FRAME_DATA, self.send_seq_no) == 0):
					#if ser.write(mail.msg) == 0:
						print ("hdlc: uart locked, telling hdlc tx thread to retry\n") 
						#newMail = hdlcmsg(HDLCMT.HDLC_RESP_RETRY_W_TIMEO, mail.msg, mail.srcid, mail.srcbox)
						self.mutex.release()
						try:
							self.hdlcOutbox.put(mail, block=False)
						except queue.Full as e:
							print ("hdlc: no space in hdlc thread outbox. ERROR!!\n")
							ser.close()
							exit(0)
					else:
						self.mutex.release()
						print ("hdlc tx: Resending frame w/ seq no %d\n" % self.send_seq_no)
			else:
			#	with self.mutex:
				print ("hdlc tx: INVALID HDLC MSG\n")
	
	# tmp for pipe 12/1 #def hdlc_rx(self):
	def hdlc_rx(self, ser):
		#with self.mutex:
		print ("hdlc thread %d rx starts running\n" % threading.get_ident())
		# tmp for pipe 12/1 #global ser, mutex, recv_seq_no, send_seq_no, hdlcOutbox, hdlcInbox, hdlc_reg #sender_inbox, sender_id
		# global mutex, recv_seq_no, send_seq_no, hdlcOutbox, hdlcInbox, hdlc_reg #sender_inbox, sender_id
		while True:
			#with self.mutex:
				self.mutex.acquire()
				seq_no = 0 # init
				#debug 
				print ("hdlc rx: reading from ser\n")
				#debug end
				try:
					data, ftype, seq_no = get_data(ser.read()) # ser.read is changed to nonblocking
				except MessageError:
					pass
				except FCSError:
					print ('[x] Bad FCS\n')
					ser.close()
					exit(0)
				#except TimeoutError as e:
				#	stderr.write(str(e) + '\n')
				#	stdout.write('[*] Done\n')
				#	ser.close()
				#	exit(0)
				except KeyboardInterrupt:
					print ('[*] Bye !\n')
					ser.close()
					exit(0)
				#else:
	# error will not get beyond this line
			#with mutex: # need to access global var
				if len(data) > 0 and (self.recv_seq_no % 8 == seq_no or (self.recv_seq_no-1) % 8 == seq_no): 
	# valid data frame
					self.mutex.release()
					newMail = hdlcmsg(HDLCMT.HDLC_MSG_SND_ACK, seq_no, threading.get_ident(), self.hdlcInbox) #_thread.get_ident()
					# notify hdlc tx thread
					try:  
						self.hdlcOutbox.put(newMail, block=False)
					except queue.Full as e:
						print ("hdlc: ACK no space available in hdlc tx thread outbox. ERROR!!\n")
						ser.close()
						exit(0)
					print ("hdlc: received data frame w/ seq_no: %d\n" % seq_no)
	
					# pass the data to corresponding thread
					#with mutex:
					self.mutex.acquire()
					if (self.recv_seq_no % 8 == seq_no):
						print("hdlc: received data frame w/ seq_no: %d\n" % seq_no)
					# TODO; need to pass to the thread register to this port
					# check the entry (type uart pkt hdr):
					# notify the app thread data is ready to be received					 
						print("hdlc: received packet for port %d\n" % data) 
						self.recv_seq_no += 1
						self.mutex.release()
						src_port, dst_port, pkt_type, msgstr = unpack('HHN59s', data)
						#entry = getClientEntry(port) 
						entry = next((entry for entry in self.hdlc_reg if entry.port == dst_port), None)
						if entry != None: # found a valid subscriber
							# simply pass data and don't unpack
							newMail = hdlcmsg(HDLCMT.HDLC_MSG_RDY, data, threading.get_ident(), self.hdlcInbox) # from rx to app threads # data will include header just like C implementation!
							# notify app thread
							try: 
								entry.rbox.put(newMail, block=False) # for app's rx thr to process
							except queue.Full as e:
								print ("hdlc: entry's mailbox is full!\n")
								ser.close()
								exit(0)
						else:
							print ("hdlc: no thread subscribed to port!\n")
					else:
						self.mutex.release()

				elif len(data) == 0 and (ftype == FRAME_ACK or  ftype == FRAME_NACK): # NACK shouldn't occur
				# receive ACK so need to notify sender msg was received properly
					self.mutex.release()
					print ("hdlc: received ACK/NACK w/ seq_no: %d\n" % seq_no)
					self.mutex.acquire()
					if (seq_no == (self.send_seq_no % 8)):
						print ("hdlc: received ACK/NACK w/ seq_no: %d\n", seq_no)
						if (seq_no == (self.send_seq_no % 8)):
							self.send_seq_no += 1 # we can move on to next send seq num
							self.mutex.release()
							newMail = hdlcmsg(HDLCMT.HDLC_RESP_SND_SUCC, b'', threading.get_ident(), self.hdlcInbox) # empty string since this is just to notify sender # TODO: review
							# find sender
							src_port, dst_port, pkt_type, msgstr = unpack('HHN59s', data) 
							#entry = getClientEntry(port) 
							entry = next((entry for entry in self.hdlc_reg if entry.port == dst_port), None) # still use dst_port since this is from other side to sender on this side
							try:
								entry.tbox.put(newMail, block=False) # notify the app tx thread so tx can work on next #TODO check ibox or obox
							except queue.Full as e:
								print ("hdlc: no space available in sender thread inbox. ERROR!!\n")
								ser.close()
								exit(0)
							print ("hdlc: sender port is %d!\n" % port)
						else:
							self.mutex.release()
							print ("hdlc: no sender thread on this port!\n")
					else:
						self.mutex.release()

				else:
					self.mutex.release()
					print ("hdlc rx: got nothing to read") #debug

	
	
	# can only send one seq number until it is being acked. 
	# Question: what if different thread want to send? there is no way to prevent it enter the queue?
	# 
	# Quesion there is only one global seq, but what if multiple app send data one after another, how to keep track?
	
	def hdlc_register(self, entry):
		#global hdlc_reg
		self.hdlc_reg.append(entry)
	
	def hdlc_unregister(self, entry):
		#global hdlc_reg
		try:
			self.hdlc_reg.remove(entry)
		except ValueError:
			pass

	def get_outbox(self):
		return self.hdlcOutbox

	def get_inbox(self):
		return self.hdlcInbox

	def __del__(self):
		self.isRunning = 0
