#!/usr/bin/env python
from twisted.internet import reactor
from autobahn.websocket import WebSocketServerFactory, WebSocketServerProtocol, listenWS
import uuid

class CloudEngineProtocol(WebSocketServerProtocol):
    def onMessage(self, msg, binary):
        if binary:
            self.sendMessage('received binary message')
            #f = open('apple_copy.png','w')
            #f.write(msg[33:])
            #f.close()
            
            print 'UUID -> '+ msg[:32]
            #self.sendMessage(msg[32:], binary=True)
            tmpUUID = uuid.uuid4().hex
            print 'UUID <- '+ tmpUUID
            self.sendMessage(tmpUUID+msg[32:], binary=True)
            #self.sendMessage(msg, binary=True)
        else:
            self.sendMessage('received non binary message')
            
    def onOpen(self):
        print 'connection made.'
        

if __name__ == '__main__':
    factory = WebSocketServerFactory("ws://localhost:9000")
    factory.protocol = CloudEngineProtocol
    listenWS(factory)
    reactor.run()
