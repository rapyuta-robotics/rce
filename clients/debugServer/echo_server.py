#!/usr/bin/env python
from twisted.internet import reactor
from autobahn.websocket import WebSocketServerFactory, WebSocketServerProtocol, listenWS

import json

class EchoServerProtocol(WebSocketServerProtocol):
    def onMessage(self, msg, binary):
        if not binary:
            print "Received a non binary image"
            msg_pydict = json.loads(msg)
            print( msg_pydict)
            
if __name__ == '__main__':

	factory = WebSocketServerFactory("ws://localhost:9000")
	factory.protocol = EchoServerProtocol
	listenWS(factory)
	reactor.run()
