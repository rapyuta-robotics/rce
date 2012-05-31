#!/usr/bin/env python
from twisted.internet import reactor
from autobahn.websocket import WebSocketServerFactory, WebSocketServerProtocol, listenWS

import json

class EchoServerProtocol(WebSocketServerProtocol):

	incoming_msg_count = 0 

	def onConnect(self, _):
		print 'Connection established'
    
	def onMessage(self, msg, binary):
		print 'received Message'
		self.incoming_msg_count += 1
		self.sendMessage('received Message # '+str(self.incoming_msg_count))
		print('received Message # '+str(self.incoming_msg_count))
           
if __name__ == '__main__':
	factory = WebSocketServerFactory("ws://localhost:9000")
	factory.protocol = EchoServerProtocol
	listenWS(factory)
	reactor.run()
