import threading # for oob
from enum import Enum
from yahdlc import *
import serial
import queue
from sys import stdout, stderr
from HDLC import HDLC, HDLCEntry, HDLCMsg, HDLCMT, HDLC_RTRY_TIMEO_USEC
from time import time, sleep
from struct import *
# random generating letters
import string, random

#  A reminder of C implementation hdlc packet
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

class AppThread: 
	def __init__(self, app_num=0, port=0, frame_no=0, hdlc_thr=None): 
		self.app_num = app_num
		self.port = port
		self.frame_no = frame_no # frame_no just means the numbers of frame sent/received by this thread (diff from seq no)
		self.rbox = queue.Queue()
		self.tbox = queue.Queue()
		self.mutex = threading.Lock() # 
		self.tx_thr = threading.Thread(target=self.txrun) 
		self.rx_thr = threading.Thread(target=self.rxrun)
		self.is_running = 0
		self.hdlc_thr = hdlc_thr

	def apprun(self, hdlc_thr):
		self.is_running = 1
		if self.hdlc_thr == None:
			self.hdlc_thr = hdlc_thr
		entry = HDLCEntry(self.port, self.rbox, self.tbox)
		self.hdlc_thr.hdlc_register(entry)
		self.tx_thr.start()
		self.rx_thr.start()

	def txrun(self):
		print ("App thread %d tx starts running\n" % threading.get_ident())
		while True:
			# Here is just an ex. Can call other function/method to generate data including header to hdlc mailbox
			randstr = ''.join(random.sample(list(string.printable), 59))
			print ("App tx thread %d gens %s\n" % (threading.get_ident(), randstr)) # debug
			bytesobj = pack('HHB59s', self.port, self.port, self.app_num, randstr.encode('ascii'))
			mail = HDLCMsg(HDLCMT.HDLC_MSG_SND, bytesobj, threading.get_ident(), self.tbox)
			try:
				self.hdlc_thr.get_outbox().put(mail, block=False)
			except queue.Full as e:
				print ("hdlc's tx mailbox is full!\n")
				exit(0)
			#debug
			if self.hdlc_thr.get_outbox().empty():
				print ("hdlc's tx mailbox is still empty!")
			
			# get message
			print ("App tx thread %d checking tbox\n" % (threading.get_ident())) # debug
			#debug end
			mail = self.tbox.get(block=True) # if we have nothing to send, we wait
			if mail.type == HDLCMT.HDLC_RESP_SND_SUCC:
				with self.mutex:
					self.frame_no += 1

			elif mail.type == HDLCMT.HDLC_RESP_RETRY_W_TIMEO:
					sleep(HDLC_RTRY_TIMEO_USEC/1000000) 
					with self.mutex: 
						print("app thread id %d: retry frame_no %d\n" % (threading.get_ident(), self.frame_no))
					# retry message will be put in both tx thr mailbox & hdlc tx mailbox
					newMail4app = HDLCMsg(HDLCMT.HDLC_RESP_RETRY_W_TIMEO, mail.msg, threading.get_ident(), self.tbox) 
					try: 
						self.tbox.put(newMail4app, block=False) 
					except queue.Full as e:
						print ("app thread id %d's tx mailbox is full!\n" % threading.get_ident())
						exit(0)
					# mail was already in tbox, so no additional encoding required
					newMail4hdlc = HDLCMsg(HDLCMT.HDLC_MSG_SND, mail.msg, threading.get_ident(), self.tbox) 
					try: 
						self.hdlc_thr.get_outbox().put(newMail4hdlc, block=False) 
					except queue.Full as e:
						print ("hdlc's tx mailbox is full!\n")
						exit(0)
					with self.mutex:						
						self.frame_no += 1
			else:
				print ("Unknown mail type in app thread id %d tx mailbox - skipped\n" % threading.get_ident())
				with self.mutex:						
					self.frame_no += 1
			sleep(0.1) # sleep for 0.1s

	def rxrun(self):
		print ("App thread %d rx starts running\n" % threading.get_ident())
		while True:
			print ("App rx thread %d checking rbox\n" % (threading.get_ident())) # debug
			mail = self.rbox.get(block=True) # if we have nothing to recv, we wait
			if mail.type == HDLCMT.HDLC_PKT_RDY:
# extract header # in C, header data structure is copied which contains type info
				src_port, dst_port, pkt_type, msgstrenc = unpack('HHB59s', mail.msg)
				msgstr = msgstrenc.decode('ascii')

				if pkt_type == self.app_num:
					with self.mutex:
						self.frame_no += 1
					print ("App thread id %d receives pkt %s\n" % (threading.get_ident(), msgstr))


	def get_tbox(self):
		return self.tbox

	def get_rbox(self):
		return self.rbox

	def __del__(self):
		if (self.hdlc_thr != None):
			entry = HDLCEntry(self.port, self.rbox, self.tbox)
			self.hdlc_thr.hdlc_unregister(entry)
		self.is_running = 0

					


