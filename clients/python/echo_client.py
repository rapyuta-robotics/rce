#!/usr/bin/env python
from twisted.internet import reactor
from autobahn.websocket import WebSocketClientFactory, WebSocketClientProtocol, connectWS

class EchoClientProtocol(WebSocketClientProtocol):

	def sendHello(self):
 		f = open('chillie.png','r')
		data = f.read()
		f.close()
		self.sendMessage(data,binary=True)

   	def onOpen(self):
        self.sendHello()

	def onMessage(self, msg, binary):
        print "Got echo: " + msg + " Type: " + str(binary)
        reactor.callLater(1, self.sendHello)


if __name__ == '__main__':
	factory = WebSocketClientFactory("ws://localhost:9000")
	factory.protocol = EchoClientProtocol
	connectWS(factory)
	reactor.run()
