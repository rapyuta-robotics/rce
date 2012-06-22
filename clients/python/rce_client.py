#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       rce_client.py
#       
#       This file is part of the RoboEarth Cloud Engine clients.
#       
#       This file was originally created for RoboEearth
#       http://www.roboearth.org/
#       
#       The research leading to these results has received funding from
#       the European Union Seventh Framework Programme FP7/2007-2013 under
#       grant agreement no248942 RoboEarth.
#       
#       Copyright 2012 RoboEarth
#       
#       Licensed under the Apache License, Version 2.0 (the "License");
#       you may not use this file except in compliance with the License.
#       You may obtain a copy of the License at
#       
#       http://www.apache.org/licenses/LICENSE-2.0
#       
#       Unless required by applicable law or agreed to in writing, software
#       distributed under the License is distributed on an "AS IS" BASIS,
#       WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#       See the License for the specific language governing permissions and
#       limitations under the License.
#       
#       \author/s: Gajamohan Mohanarajah
#       
#       

from threading import Thread
from Queue import Queue

from autobahn.websocket import WebSocketClientFactory, WebSocketClientProtocol, connectWS
import json
import time

import ClientDebugMsgs

class RCEClient(object):
    def processMessage(self, msg):
        data = json.loads(msg)
        
        if data['type'] == 'CS':
            for cTag, status in data['data'].items():
                if cTag in self.containers:
                    if status:
                        self.containers[cTag] = 'running'
                    else:
                        self.containers[cTag] = 'failedBooting'
                    print '== Container Status Update =='
                    for k,v in self.containers.items():
                        print k + ' - ' + v
                else:
                    print 'Error: [processMessage] received CS with tag ['+cTag\
                        +']. But a container with this tag does not exist.'
                    print 'Existing containers are '+str(self.containers.keys())
        else:
            pass
    
    def processBinary(self, msg):
        pass
    
    def __init__(self, id, reactor, main, wsURL):
        self.uniqueID = id
        self.reactor = reactor
        self.main = main
        self.wsURL = wsURL
        self.connection = None
        self.containers = {}
    
    def sendConfigMsg(self, data):
        self.send(json.dumps(data))
        
    def sendDataMsg(self, data):
        self.send(json.dumps(data))
    
    def send(self, msg):
        # Should not be called directly
        # self.reactor.callFromThread(self.connection.sendMessage, msg)
        if self.connection:
            # self.connection.send(msg)
            self.reactor.callFromThread(self.connection.send, msg)
        else:
            raise RuntimeError()
    
    def createContainer(self, tag): 
        cmd_CC = ClientDebugMsgs.cmd_CC_debug.cmd;
        cmd_CC["data"]["containerTag"] = tag
        cmd_CC_js = json.dumps(cmd_CC)
        self.containers[tag] = 'booting';
        self.send(cmd_CC_js)
        
    def removeContainer(self, tag): 
        print 'Info: [removeContainer] container remove request'
        if tag in self.containers.keys():
            if self.containers[tag]=='running':
                del self.containers[tag]
                cmd_DC_js = json.dumps(ClientDebugMsgs.cmd_DC_debug.cmd)
                self.send(cmd_DC_js)
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
    client = RCEClient('robotUniqueID', reactor, main, 'ws://localhost:9000')
    client.spin()
