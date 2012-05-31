#!/usr/bin/env python
from threading import Thread
from Queue import Queue
from autobahn.websocket import WebSocketClientFactory, WebSocketClientProtocol, connectWS

class Container(object):

    def __init__(self, connection):
        self.connection = connection
        self.containerID = None
        
    def registerContainerID(self, containerID):
        # TODO: Avoid multiple calls
        self.containerID = containerID
    
    def change(self):
        #TODO
        pass
    
    def sendMsg(self, msg):
        #TODO
        pass
    
    def __del__(self):
        cmd_CH = {
            "type":"CH",
            "dest":"$$$$$$",
            "orig":"robotUniqueID",  
            "data":{"containerID":self.containerID}
            }
        
        self.connection.send(json.dumps(cmd_CH))

class RoboEarthCloudEngine(object):
    
    def processMessage(self, msg):
        msg = json.loads(msg)
        
        if msg['type'] == 'CSR':
            self.pendingContainer.pop().registerContainerID(msg['data']['containerID'])
        else:
            pass
    
    def processBinary(self, msg):
        pass
    
    def __init__(self, reactor, main, wsURL="ws://localhost:9000"):
        self.reactor = reactor
        self.main = main
        self.wsURL = wsURL
        self.connection = None
        self.pendingContainer = Queue()
    
    def send(self, msg):
        # Should not be called directly
        if self.connection:
            self.reactor.callFromThread(self.connection.send, msg)
        else:
            raise RuntimeError()
    
    def createContainer(self): 
        cmd_CS = {
            "type":"CS",
            "dest":"$$$$$$",
            "orig":"robotUniqueID",
            "data":None
            }
        cmd_CS_js = json.dumps(cmd_CS)
        container = Container(self)
        self.pendingContainer.append(container)
        self.send(cmd_CS_js)
        return container
        
    def registerConnection(self,connection):
        #TODO: Chek if connection already exists
        self.connection = connection
        
    def unregisterConnection(self):
        self.connection = None

    def spin(self):
        # Function never returns: Has to be run from the main thread
        factory = Factory(self.wsURL)
        connectWS(factory)
        Thread(target=self.main).start()
        self.reactor.run()
        
class Factory(WebSocketClientFactory):
    def __init__(self, url):
        WebSocketClientFactory.__init__(self, url)
        self.engine = engine
    
    def buildProtocol(self, addr):
        p = Protocol(self.engine)
        p.factory = self
        return p
    
class Protocol(WebSocketClientProtocol):
    def __init__(self, engine):
        self.engine = engine
    
    def send(self, msg):
        self.sendMessage("Hello from client")

    def onOpen(self):
        self.engine.registerConnection(self)
    
    def onClose(self):
        self.engine.unregisterConnection()

    def onMessage(self, msg, binary):
        if binary:
            self.engine.processBinary(msg)
        else:
            self.engine.processMessage(msg)
        
        print "received message from server!"
        

def main():
    print 'I was here'

if __name__ == '__main__':
    from twisted.internet import reactor
    engine = RoboEarthCloudEngine(reactor, main)
    engine.spin()
