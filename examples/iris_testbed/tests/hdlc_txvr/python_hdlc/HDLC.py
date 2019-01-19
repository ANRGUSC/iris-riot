#import _thread 
import queue
import threading # can join thread
#from _thread import Queue
#from Queue import *
from enum import Enum
from yahdlc import *
import serial
from sys import stdout, stderr
from struct import *
# for pipe testing
import os
from time import time

# used to find out subscriber
#typedef struct hdlc_entry {
#    struct hdlc_entry *next;
#    uint16_t port;
#    Mail<msg_t, HDLC_MAILBOX_SIZE> *mailbox;
#} hdlc_entry_t;

class HDLCEntry:
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

class UARTPktHdr:
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

# global vars
# python 3.7 has nano sec resolution
HDLC_RTRY_TIMEO_USEC  = 200000
HDLC_RETRANS_TIMEO_USEC = 50000

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
class HDLCMsg:
	def __init__(self, mailtype=HDLCMT.HDLC_NONE, msg=None, srcid=0, srcbox=None): # srcbox will point to corresponding queue
		self.type = mailtype
		self.msg = msg
		self.srcid = srcid
		self.srcbox = srcbox

class HDLC:
	def __init__(self, send_seq_no=0, recv_seq_no=0, if_obj=None, of_obj=None):
		self.send_seq_no = send_seq_no
		self.recv_seq_no = recv_seq_no
		self.hdlc_reg = []
		self.mutex = threading.Lock() 
		self.uart_lock = False
		self.hdlc_inbox = queue.Queue()
		self.hdlc_outbox = queue.Queue()
		self.tx_thr = threading.Thread(target=self.hdlc_tx, args=(of_obj,)) 
		self.rx_thr = threading.Thread(target=self.hdlc_rx, args=(if_obj,)) 
		self.is_running = 0
		self.sender_mbox = None
		self.sender_id = 0
		self.sender_msg = None

	def hdlc_run(self):
		self.is_running = 1
		self.tx_thr.start()
		self.rx_thr.start()
		
	def hdlc_tx(self, ser):
		global HDLC_RTRY_TIMEO_USEC, HDLC_RETRANS_TIMEO_USEC 
		timeout = 0
		last_sent = 0
		#with self.mutex:
		print ("hdlc thread %d tx starts running\n" % threading.get_ident())
	# in general, if serial is busy, create a RESEND msg and put it back to hdlc outbox
	#
	# . 
	# . get message from Outbox (can be blocked)
	# . once got a msg, check type
	# . - MSG_SND: if serial not busy (or not waiting for ACK), send (reset timer); otherwise, create a RETRY msg in sender's mailbox
	# . - MSG_SND_ACK: send ACK if serial not busy; (added: otherwise, create a RESEND msg in hdlc mailbox)
	# . - MSG_RESEND: send the msg again if serial not busy (reset timer); (added: otherwise, create a RESEND msg in hdlc mailbox)
	
		while True:
			with self.mutex:
				if self.uart_lock == True:
					timeout = last_sent + HDLC_RETRANS_TIMEO_USEC/1000000 - time() 
					if timeout < 0: # if we haven't got ACK back
						newMail = HDLCMsg(HDLCMT.HDLC_MSG_RESEND, self.sender_msg, self.sender_id, self.sender_mbox)	
						try:
							self.hdlc_outbox.put(newMail, block=False)
						except queue.Full as e:
							print ("hdlc: no space in hdlc thread outbox. ERROR!!\n")
							ser.close()
							exit(0)

						mail = self.hdlc_outbox.get(block=True) # here we can use blocking to see if we have anything to send (will be removed automatically from the queue)
					else: # since we are still waiting for ACK & no timeout yet, we don't move on to next one yet
						continue
				else:
					mail = self.hdlc_outbox.get(block=True) # here we can use blocking to see if we have anything to send (will be removed automatically from the queue)
	
			#debug 
			print ("hdlc tx: processing type %s msg %s from %d\n" % (mail.type, mail.msg, mail.srcid))
			#debug end

			if mail.type == HDLCMT.HDLC_MSG_SND:
				self.mutex.acquire()
				if self.uart_lock == True: # we are processing current packet and wait for ACK, so tell other threads to retry
					self.mutex.release()
					newMail = HDLCMsg(HDLCMT.HDLC_RESP_RETRY_W_TIMEO, mail.msg, mail.srcid, mail.srcbox)
					try:
						mail.srcbox.put(newMail, block=False) 
					except queue.Full as e:
						print ("hdlc: no space in source thread (%d) outbox. ERROR!!\n" % sender_pid)
						ser.close()
						exit(0)
				else:
					self.uart_lock = True

					sender_pid = mail.srcid
					# pipe for now #self.mutex.acquire() # no shared descriptor
					seq_no = self.send_seq_no % 8
					self.mutex.release()
					print ("hdlc: request to send %s from pid %d\n" % (mail.msg, sender_pid))
					# need to send one char at a time
					if ser.write(frame_data(mail.msg, FRAME_DATA, seq_no)) == 0: 
						# pipe for now #self.mutex.release() # no shared descriptor
						print ("hdlc tx: unable to send through serial line, telling thr %d to retry\n" % sender_pid)
						# notify the sender
						newMail = HDLCMsg(HDLCMT.HDLC_RESP_RETRY_W_TIMEO, mail.msg, mail.srcid, mail.srcbox)
						try:
							mail.srcbox.put(newMail, block=False) 
						except queue.Full as e:
							print ("hdlc: no space in source thread (%d) outbox. ERROR!!\n" % sender_pid)
							ser.close()
							exit(0)
					else:
						# pipe for now #self.mutex.release()# no shared descriptor
						last_sent = time()
						with self.mutex:
							self.sender_mbox = mail.srcbox
							self.sender_id = mail.srcid
							self.sender_msg = mail.msg
						print ("hdlc: sender_pid set to %d\n" % sender_pid)
						print ("hdlc: sending frame seq no %d\n" % (seq_no)) #, len(mail.msg))
			elif mail.type == HDLCMT.HDLC_MSG_SND_ACK:
					# pipe for now self.mutex.acquire()# no shared descriptor
					if ser.write(frame_data('', FRAME_ACK, mail.msg)) == 0: # this mail is from rx thread who put the seq no in msg
						# pipe for now #self.mutex.release()# no shared descriptor
						print ("hdlc: uart locked, telling hdlc tx thread to retry\n") 
						try:
							self.hdlc_outbox.put(mail, block=False)
						except queue.Full as e:
							print ("hdlc: no space in hdlc thread outbox. ERROR!!\n")
							ser.close()
							exit(0)
					else:
						# pipe for now #self.mutex.release()# no shared descriptor
						print ("hdlc: sending ack w/ seq no %d\n" % mail.msg) 
			elif mail.type == HDLCMT.HDLC_MSG_RESEND:
					# pipe for now #self.mutex.acquire()# no shared descriptor
					self.mutex.acquire() # need to use mutex to access seq_no
					if ser.write(frame_data(mail.msg, FRAME_DATA, (self.send_seq_no % 8))) == 0:
						print ("hdlc: uart locked, telling hdlc tx thread to retry\n") 
						# pipe for now #self.mutex.release()# no shared descriptor
						try:
							self.hdlc_outbox.put(mail, block=False)
						except queue.Full as e:
							print ("hdlc: no space in hdlc thread outbox. ERROR!!\n")
							ser.close()
							exit(0)
					else:
						# pipe for now #self.mutex.release()# no shared descriptor
						last_sent = time()
						print ("hdlc tx: Resending frame w/ seq no %d\n" % (self.send_seq_no % 8))
					self.mutex.release()
			else:
				print ("hdlc tx: INVALID HDLC MSG\n")
	
	def hdlc_rx(self, ser):
		print ("hdlc thread %d rx starts running\n" % threading.get_ident())
		while True:
				#pipe for now# self.mutex.acquire() # not shared descriptor
				seq_no = 0 # init
				# reset begin
				data = b''
				ftype = 0
				seq_no = 0
				# reset end
				while True:
					try:
						data, ftype, seq_no = get_data(ser.read(ser.in_waiting)) # ser.read is changed to nonblocking
						break
					except MessageError:
						pass
					except FCSError:
						print ('[x] Bad FCS\n')
						ser.close()
						exit(0)
					except KeyboardInterrupt:
						print ('[*] Bye !\n')
						ser.close()
						exit(0)
				# comment out len(data) > 0 since we just need to check if it is a data frame
				if ftype == FRAME_DATA and (self.recv_seq_no % 8 == seq_no or (self.recv_seq_no-1) % 8 == seq_no): # if it is data frame from current seq no or previous seq no hasn't been acked 
					# valid data frame
					# pipe for now #self.mutex.release()# not shared descriptor
					newMail = HDLCMsg(HDLCMT.HDLC_MSG_SND_ACK, seq_no, threading.get_ident(), self.hdlc_inbox) #_thread.get_ident()
					# notify hdlc tx thread
					try:  
						self.hdlc_outbox.put(newMail, block=False)
					except queue.Full as e:
						print ("hdlc: ACK no space available in hdlc tx thread outbox. ERROR!!\n")
						ser.close()
						exit(0)
					print ("hdlc: received data frame w/ seq_no: %d\n" % seq_no)
	
					# pass the data to corresponding thread
					self.mutex.acquire()
					if (self.recv_seq_no % 8 == seq_no):
						print("hdlc: received data frame w/ seq_no: %d\n" % seq_no)
					# Need to pass to the thread register to this port
					# check the entry (type uart pkt hdr):
					# notify the app thread data is ready to be received					 
						self.recv_seq_no += 1
						self.mutex.release()
						# need to find valid subscriber
						src_port, dst_port, pkt_type, msgstrenc = unpack('HHB59s', data)
						msgstr = msgstrenc.decode('ascii')
						print("hdlc: received packet %s for port %d\n" % (msgstr, dst_port)) 
						entry = next((entry for entry in self.hdlc_reg if entry.port == dst_port), None)
						if entry != None: # found a valid subscriber
							# simply pass data and don't unpack
							newMail = HDLCMsg(HDLCMT.HDLC_PKT_RDY, data, threading.get_ident(), self.hdlc_inbox) # from rx to app threads # data will include header just like C implementation!
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

				elif  (ftype == FRAME_ACK or  ftype == FRAME_NACK): # NACK shouldn't occur # comment out len(data) == 0 since FRAME_ACK will be the ack frame
				# receive ACK so need to notify sender msg was received properly
					# pipe for now# self.mutex.release()# not shared descriptor
					self.mutex.acquire()
					self.uart_lock = False
					if (seq_no == (self.send_seq_no % 8)):
						print ("hdlc: received ACK/NACK w/ seq_no: %d\n" % seq_no)
						if (seq_no == (self.send_seq_no % 8)):
							self.send_seq_no += 1 # we can move on to next send seq num
							self.mutex.release()
							newMail = HDLCMsg(HDLCMT.HDLC_RESP_SND_SUCC, b'', threading.get_ident(), self.hdlc_inbox) # empty string since this is just to notify sender 
						   # need to find valid subscriber
							with self.mutex:
								try:
									self.sender_mbox.put(newMail, block=False) # notify the app tx thread so tx can work on next 
								except queue.Full as e:
									print ("hdlc: no space available in sender thread inbox. ERROR!!\n")
									ser.close()
									exit(0)
								print ("hdlc: passed ACK to sender %d!\n" % self.sender_id)
						else:
							self.mutex.release()
							print ("hdlc: no sender thread on this port!\n")
					else:
						self.mutex.release()

				#else:
				#	#pipe for now#self.mutex.release()# not shared descriptor

	
	def hdlc_register(self, entry):
		self.hdlc_reg.append(entry)
	
	def hdlc_unregister(self, entry):
		try:
			self.hdlc_reg.remove(entry)
		except ValueError:
			pass

	def get_outbox(self):
		return self.hdlc_outbox

	def get_inbox(self):
		return self.hdlc_inbox

	def __del__(self):
		self.is_running = 0
