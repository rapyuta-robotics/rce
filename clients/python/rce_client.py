#!/usr/bin/env python
from threading import Thread
from Queue import Queue

from autobahn.websocket import WebSocketClientFactory, WebSocketClientProtocol, connectWS
import json
import time

class RCEClient(object):
    def processMessage(self, msg):
        data = json.loads(msg)
        
        if data['type'] == 'CSR':
            (cTag, status) = data['data'].items()[0]
            if cTag in self.container.keys():
                if status:
                    self.container[cTag] = 'running'
                else:
                    self.container[cTag] = 'failedBooting'
                print '== Container Status =='
                for k,v in self.container.items():
                    print k + ' - ' + v
            else:
                print 'Error: [processMessage] received CSR with tag ['+cTag\
                    +']. But a container with this tag does not exist.'
        else:
            pass
    
    def processBinary(self, msg):
        pass
    
    def __init__(self, id, reactor, main, wsURL='ws://localhost:9000'):
        self.uniqueID = id
        self.reactor = reactor
        self.main = main
        self.wsURL = wsURL
        self.connection = None
        self.container = {}
    
    def sendConfigMsg(self, data):
        self.send(json.dumps(data))
        
    def sendDataMsg(self, data):
        self.send(json.dumps(data))
    
    def send(self, msg):
        # Should not be called directly
        # self.reactor.callFromThread(self.connection.sendMessage, msg)
        if self.connection:
            #self.connection.send(msg)
            self.reactor.callFromThread(self.connection.send, msg)
        else:
            raise RuntimeError()
    
    def createContainer(self, tag): 
        cmd_CS = {
            'type':'CS',
            'dest':'$$$$$$',
            'orig':'robotUniqueID',
            'data':{'containerTag':tag}
            }
        cmd_CS_js = json.dumps(cmd_CS)
        self.container[tag] = 'booting';
        self.send(cmd_CS_js)
        
    def removeContainer(self, tag): 
        print 'Info: [removeContainer] container remove request'
        if tag in self.container.keys():
            if self.container[tag]=='running':
                del self.container[tag]
                cmd_CH = {
                    'type':'CH',
                    'dest':'$$$$$$',
                    'orig':'robotUniqueID',
                    'data':{'containerTag':tag}
                }
                cmd_CH_js = json.dumps(cmd_CH)
                self.send(cmd_CH_js)
            else:
                print 'Warn: [removeContainer] Container with tag:['+tag\
                +'] is [booting] status. Can not be removed.'
        else:
            print 'Error: [removeContainer] No container found with tag:['+tag+']'
    
    def registerConnection(self,connection):
        #TODO: Chek if connection already exists
        print '[reactor] connection made'
        self.connection = connection
        
    def unregisterConnection(self):
        self.connection = None

    def spin(self):
        print 'I am starting to spin. '
        # Function never returns: Has to be run from the main thread
        factory = RCEClientFactory(self.wsURL,self)
        connectWS(factory)
        Thread(target=self.main).start()
        self.reactor.run()
        
class RCEClientFactory(WebSocketClientFactory):
    def __init__(self, url, client):
        WebSocketClientFactory.__init__(self, url)
        self.client = client
    
    def buildProtocol(self, addr):
        p = RCEClientProtocol(self.client)
        p.factory = self
        return p
    
class RCEClientProtocol(WebSocketClientProtocol):
    def __init__(self, client):
        self.client = client
    
    def send(self, msg):
        print 'sending message to server'
        self.sendMessage(msg)

    def onOpen(self):
        print 'Info: [protocol] registerConnection called'
        self.client.registerConnection(self)
        
    def connectionMade(self):
        WebSocketClientProtocol.connectionMade(self)
        print 'Info: [protocol] connection Made'
    
    def onMessage(self, msg, binary):
        if binary:
            self.client.processBinary(msg)
        else:
            self.client.processMessage(msg)
    print 'received message from server!'

def main():
    print 'I was here'
    print(client)
        
    while not client.connection:
        time.sleep(1)
        print 'waiting for connection'
    
    print '[main] connection made'
    client.createContainer('tag01')
    client.createContainer('tag02')
    
    time.sleep(2)
    
    client.removeContainer('tag01')
    
    time.sleep(2)
    
    client.createContainer('tag03')

if __name__ == '__main__':
    from twisted.internet import reactor
    client = RCEClient(reactor, main)
    client.spin()