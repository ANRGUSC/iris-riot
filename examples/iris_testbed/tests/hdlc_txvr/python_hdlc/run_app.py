#import _thread, time
import threading # for oob
from enum import Enum
from yahdlc import *
import serial
import queue
from sys import stdout, stderr
#nooob#from hdlc_nooob import hdlc_entry, hdlc_register, hdlc_unregister, hdlcmsg, HDLCMT
from hdlc import hdlc, hdlc_entry, hdlcmsg, HDLCMT, HDLC_RTRY_TIMEO_USEC

from struct import *
# random generating letters
import string, random
# global vars
#MAIN_THR_PORT =  1234
#THREAD2_PORT  =  5678
#PKT_FROM_MAIN_THR =  0
#PKT_FROM_THREAD2  =  1
#  hdlc_pkt_t contains a pointer to char and length
# 			copy send_hdr to the beginning of pkt.data
#        uart_pkt_insert_hdr(pkt.data, HDLC_MAX_PKT_SIZE, &send_hdr); 
#
#        pkt.data[UART_PKT_DATA_FIELD] = frame_no; # put seq no after the header
#        for(int i = UART_PKT_DATA_FIELD + 1; i < HDLC_MAX_PKT_SIZE; i++) { # from there on, write data one byte at a time 
#            pkt.data[i] = (char) ( rand() % 0x7E);
#        }
#
#        pkt.length = HDLC_MAX_PKT_SIZE; # HDLC_MAX_PKT_SIZE = 64, UART_PKT_DATA_FIELD = 5 

# value of TIMEO #define HDLC_RTRY_TIMEO_USEC        200000
# check typo # 287 - 288 in main.cpp - seems to be ok  

#  hdlc_pkt_t contains a pointer to char and length
#  hdlc_pkt_t pkt; // dw comment: for other threads to pass to hdlc thread via IPC

#  pkt.data = send_data;
#  pkt.length = 0; # init

#  hdlc_buf_t contains control, pointer to char, length
#  hdlc_buf_t *buf; // dw comment: for the hdlc packets
#	hdlc_entry_t thread2 = { NULL, THREAD2_PORT, &thread2_mailbox }; // dw comment: linked list of port mapping for a thread (a thread can register to multiple port, but not vice versa)

# copy header portion in buf->data to a recv_hdr (header structure) first. if it is the right port, copy data (header + data), but only display data (w/o header) portion
#                        uart_pkt_parse_hdr(&recv_hdr, buf->data, buf->length);
#                        if (recv_hdr.pkt_type == PKT_FROM_MAIN_THR) {
#                            memcpy(recv_data, buf->data, buf->length);
#                            printf("main_thr: received pkt %d ; dst_port %d\n", 
#                            recv_data[UART_PKT_DATA_FIELD], recv_hdr.dst_port);
#                        }

class AppThread: #(threading.Thread): use thread obj instead
#	APPCOUNT = 0
	#nooob#def __init__(self, appnum=0, port=0, frame_no=0, mutex=None):
	def __init__(self, appnum=0, port=0, frame_no=0, hdlcthr=None): # since mutex is shared between thr, so we pass it in
		self.appnum = appnum
		self.port = port
		self.frame_no = frame_no
		self.rbox = queue.Queue()
		self.tbox = queue.Queue()
		self.mutex = threading.Lock() # 
		#self.mutex = mutex 
		#self.isRunning = 1
#		APPCOUNT += 1
		self.txthr = threading.Thread(target=self.txrun) 
		self.rxthr = threading.Thread(target=self.rxrun)
		# create an entry to register to hdlc 
		#entry = hdlc_entry(port, self.rbox, self.tbox)
		#self.hdlc_reg = hdlc_reg
		#self.hdlc_reg.hdlc_register(entry)
		#nooob#hdlc_register(entry)
		# generate data?
		#self.txthr.start()
		#self.rxthr.start()
		#self.txthr.join()
		#self.rxthr.join()
		self.isRunning = 0
		self.hdlcthr = hdlcthr

	def apprun(self, hdlcthr):
		self.isRunning = 1
		if self.hdlcthr == None:
			self.hdlcthr = hdlcthr
		entry = hdlc_entry(self.port, self.rbox, self.tbox)
		self.hdlcthr.hdlc_register(entry)
		self.txthr.start()
		self.rxthr.start()
		#self.txthr.join()
		#self.rxthr.join()

	def txrun(self):
		#with self.mutex:
		print ("App thread %d tx starts running\n" % threading.get_ident())
		while True:
			# can call other function/method to generate data including header to hdlc mailbox
			randstr = ''.join(random.sample(list(string.printable), 59))
			#print ("App tx thread %d gens %s\n" % (threading.get_ident(), randstr)) # debug
			bytesobj = pack('HHN59s', self.port, self.port, self.appnum, randstr.encode('ascii'))
			mail = hdlcmsg(HDLCMT.HDLC_MSG_SND, bytesobj, threading.get_ident(), self.tbox)
			try:
				self.hdlcthr.get_outbox().put(mail, block=False)
			except queue.Full as e:
				print ("hdlc's tx mailbox is full!\n")
				exit(0)
			#debug
			if self.hdlcthr.get_outbox().empty():
				print ("hdlc's tx mailbox is still empty!")
			
			#print ("App tx thread %d gens %s\n" % (threading.get_ident(), randstr)) # debug
			# get message
			print ("App tx thread %d checking tbox\n" % (threading.get_ident())) # debug
			#debug end
			mail = self.tbox.get(block=True) # if we have nothing to send, we wait
			if mail.type == HDLCMT.HDLC_RESP_SND_SUCC:
				with self.mutex:
					self.frame_no += 1
				print ("app thread id %d: frame_no %d was successfully sent & acked!\n" % (threading.get_ident(), frame_no))

			elif mail.type == HDLCMT.HDLC_RESP_RETRY_W_TIMEO:
					time.sleep(HDLC_RTRY_TIMEO_USEC/1000000) 
				#with self.mutex: Queue will take care of the synchronization
					print("app thread id %d: retry frame_no %d\n" % (threading.get_ident(), frame_no))
					# retry message will be put in both tx thr mailbox & hdlc tx mailbox
					#bytesobj = pack('I', HDLC_RTRY_TIMEO_USEC) # since we identify the mail by time, don't pack it
					newMail4app = hdlcmsg(HDLCMT.HDLC_RESP_RETRY_W_TIMEO, HDLC_RTRY_TIMEO_USEC, threading.get_ident(), self.tbox) 
					try: 
						self.tbox.put(newMail4app, block=False) 
					except queue.Full as e:
						print ("app thread id %d's tx mailbox is full!\n" % threading.get_ident())
						exit(0)
					# mail was already in tbox, so no additional encoding required
					newMail4hdlc = hdlcmsg(HDLCMT.HDLC_MSG_SND, mail.msg, threading.get_ident(), self.tbox) # TODO: review timeout
					try: 
						self.hdlcthr.get_outbox().put(newMail4hdlc, block=False) 
					except queue.Full as e:
						print ("hdlc's tx mailbox is full!\n")
						exit(0)
					with self.mutex:						
						self.frame_no += 1
			else:
				print ("Unknown mail type in app thread id %d tx mailbox - skipped\n" % threading.get_ident())

	def rxrun(self):
#		with self.mutex:
		print ("App thread %d rx starts running\n" % threading.get_ident())
		while True:
			print ("App rx thread %d checking rbox\n" % (threading.get_ident())) # debug
			mail = self.rbox.get(block=True) # if we have nothing to recv, we wait
			if mail.type == HDLCMT.HDLC_PKT_RDY:
# extract header # in C, header data structure is copied which contains type info
				ptk_type = mail.msg
				src_port, dst_port, pkt_type, msgstr = unpack('HHN59s', mail.msg)
				if ptk_type == self.appnum:
					with self.mutex:
						self.frame_no += 1
					print ("App thread id %d receives pkt %s\n" % (threading.get_ident(), msgstr))

#	def send2hdlc(self, data): # data will need to include header info yet		

	def get_tbox(self):
		return self.tbox

	def get_rbox(self):
		return self.rbox

	def __del__(self):
		if (self.hdlcthr != None):
			entry = hdlc_entry(self.port, self.rbox, self.tbox)
			self.hdlcthr.hdlc_unregister(entry)
		#nooob#hdlc_unregister(entry)
# python doesn't like it		
#		if self.txthr != None:
#			self.txthr.join()
#		if self.rxthr != None:
#			self.rxthr.join()
		self.isRunning = 0

					


