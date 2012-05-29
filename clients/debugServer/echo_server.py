#!/usr/bin/env python
from twisted.internet import reactor
from autobahn.websocket import WebSocketServerFactory, WebSocketServerProtocol, listenWS


class EchoServerProtocol(WebSocketServerProtocol):
	def onMessage(self, msg, binary):
		print "received message from client"
		f = open('chillie_copy.png','w')
		f.write(msg)
		f.close()
		self.sendMessage("Message Arrived", binary)

if __name__ == '__main__':

	factory = WebSocketServerFactory("ws://localhost:9000")
	factory.protocol = EchoServerProtocol
	listenWS(factory)
	reactor.run()
