#!/usr/bin/env python
from twisted.internet import reactor
from autobahn.websocket import WebSocketClientFactory, WebSocketClientProtocol, connectWS



class EchoClientProtocol(WebSocketClientProtocol):

    def sendHello(self):
        #f = open('chillie.png','r')
        #data = f.read()
        #f.close()
        msg_pydict = {'type':'type', 'dest':'destination','orig':'origin','msg':'Hello world !!' }
        self.sendMessage(json.dumps(msg_pydict),binary=False)

    def onOpen(self):
        self.sendHello()

    def onMessage(self, msg, binary):
        print "received message from server!"
        

if __name__ == '__main__':
    factory = WebSocketClientFactory("ws://localhost:9000")
    factory.protocol = EchoClientProtocol
    connectWS(factory)
    print 'calling reactor.run() ..'
    reactor.run()
    print 'Done'
