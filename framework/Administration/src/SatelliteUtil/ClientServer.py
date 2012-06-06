#!/usr/bin/env python
from twisted.internet import reactor
from autobahn.websocket import WebSocketServerFactory, WebSocketServerProtocol, listenWS
from Exceptions import InvalidRequest, InternalError
from Queue import Queue
from time import time
import cStringIO
import json
import uuid

from Robot import Robot

class FrontEndMsgHandler(object):
    incompleteMsgs = [] # List of dictionaries
                        # keys: incompleteMsg, incompleteURIList, arrivalTime
    
    def __init__(self, protocol):
        self.protocol = protocol
        
    def binaryMsgHandle(self, msg):
        pass 
        
    def recursiveURISearch(self, multiddict):
        valueList = []
        for k,v in multiddict.items():
            if isinstance(v, dict):
                valueList += self.recursiveURIKeySearch(v)
            elif k[-1:]=='*':
                valueList.append(v)
        return valueList
    
    def strMsgHandle(self, msg):
        if msg['type']=='CS':
            if not  self.protocol._robot:
                self.protocol._robot = Robot(msg['origin'], self.protocol, self.protocol._manager)
            self.protocol._robot.createContainer(msg['data']['containerTag'])
        
        elif msg['type'] == 'CH':
            self.protocol._robot.destroyContainer(msg['data']['containerTag'])
        
        elif msg['type'] == 'CC':
            if 'addNodes' in msg['data']:
                for nodeConfig in msg['data']['addNodes']:
                    self.protocol._robot.addNode(msg['dest'],nodeConfig)
                
            if 'removeNodes' in msg['data']:
                for nodesID in msg['data']['removeNodes']:
                    self.protocol._robot.removeNode(msg['dest'],nodeID)
                
            if 'addInterfaces' in msg['data']:
                for interfaceConfig in msg['data']['addInterfaces']:
                    self.protocol._robot.addInterface(msg['dest'],interfaceConfig['name'],interfaceConfig['interfaceType'],interfaceConfig['className'])

            if 'removeInterfaces' in msg['data']:
                for interfaceID in msg['data']['removeInterfaces']:
                    self.protocol._robot.removeInterface(msg['dest'],interfaceID)

            if 'setParam' in msg['data']:
                for k,v in  msg['data']['setParam'].items():
                    self.protocol._robot.setParam(msg['dest'],k,v)
                    
            if 'deleteParam' in msg['data']:
                for paramName in msg['data']['deleteParam']:
                    self.protocol._robot.deleteParam(msg['dest'],paramName)
            
        elif msg['type'] == 'CM':
            URIList = self.recursiveURIKeySearch(msg['data']['msg'])
            if URIList:
                # keys: incompleteMsg, incompleteURIList, arrivalTime
                self.incompleteMsgs.append({"incompleteMsg":msg, 
                                            "incompleteURIList":URIList,
                                            "arrivalTime":time()})
            else:
                self.protocol._robot.sendROSMsgToContainer(msg['dest'],msg['data']['interfaceName'],msg['data']['msg'])
        else:
            raise InvalidRequest('This type is not supproted')
    
    def searchAndReplaceMultiDDict(self, multiddict, searchItem, replaceItem):
        for k,v in multiddict.items():
            if isinstance(v, dict):
                self.searchAndReplaceMultiDict(v, searchItem, replaceItem)
            elif v == searchItem:
                multiddict[k] = replaceItem
    
    def binaryMsgHandle(self, msg):
        URI = msg[:32]
        binaryData = cStringIO.StringIO().write(msg[33:])
        
        # Find and replace URI with data
        for incompleteMsg in self.incompleteMsgs:
            if URI in incompleteMsg['incompleteURIList']:
                self.searchAndReplaceMultiDDict(incompleteMsg['incompleteMsg'],URI,binaryData)
                incompleteMsg['incompleteURIList'].remove(URI)
                if not incompleteMsg['incompleteURIList']:
                    self.protocol._robot.sendROSMsgToContainer(msg['dest'],msg['data']['interfaceName'],msg['data']['msg'])
            
        # remove old incomplete msgs
        for incompleteMsg in self.incompleteMsgs:
            if time() - incompleteMsg['arrivalTime'] > 60: # Magic number alert -> has to go into a confing file
                self.incompleteMsgs.remove(incompleteMsg)

class WebSocketCloudEngineProtocol(WebSocketServerProtocol):
    
    incoming_msg_count = 0 
    
    def __init__(self, manager):
        self._manager = manager
        self._robot = None
        self._frontEndMsgHandler = FrontEndMsgHandler(self)
    
    def onConnect(self, _):
        pass
    
    def connectionMade(self):
        self.connectionManager = connectionManager(self)
    
    def onMessage(self, msg, binary):          
        if binary:
            self._frontEndMsgHandler.binaryMsgHandle(msg)
        else:
            msgPy = json.dumps(msg)
            # TODO: Handle Errors caused by invalid JSON strings
            self._frontEndMsgHandler.strMsgHandle(msgPy)
            
            
    def connectionLost(self, reason):
        self._robot = None

class WebSocketCloudEngineFactory(WebSocketServerFactory):
    def __init__(self, manager, url):
        WebSocketServerFactory.__init__(self, url)
        
        self.manager = manager
    
    def buildProtocol(self, addr):
        protocol = WebSocketCloudEngineProtocol(self.manager)
        protocol.factory = self
        return protocol
