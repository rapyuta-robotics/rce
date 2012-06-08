#!/usr/bin/env python
from twisted.internet import reactor
from autobahn.websocket import WebSocketServerFactory, WebSocketServerProtocol, listenWS

class CloudEngineProtocol(WebSocketServerProtocol):
    def onMessage(self, msg, binary):
        self.sendMessage(msg, binary)

if __name__ == '__main__':
    factory = WebSocketServerFactory("ws://localhost:9000")
    factory.protocol = CloudEngineProtocol
    listenWS(factory)
    reactor.run()
