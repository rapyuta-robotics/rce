#!/usr/bin/env python
# -*- coding: utf-8 -*-

# twisted specific imports
from twisted.internet.task import LoopingCall
from autobahn.websocket import WebSocketServerFactory, WebSocketServerProtocol

# Python specific imports
from datetime import datetime, timedelta
import json
import uuid

try:
    from cStringIO import StringIO
except ImportError:
    from StringIO import StringIO

# Custom imports
import settings
from Exceptions import InvalidRequest
from Robot import Robot

class WebSocketCloudEngineProtocol(WebSocketServerProtocol):
    """ Protocol which is used for the connections from the robots to the reCloudEngine.
    """
    _CREATE_CONTAINER = 'CS'
    _DESTROY_CONTAINER = 'CH'
    _CHANGE_COMPONENT = 'CC'
    _MESSAGE = 'CM'
    
    def __init__(self, commManager, satelliteManager):
        """ Initialize the Protocol.
        """
        self._commManager = commManager
        self._satelliteManager = satelliteManager
        self._robot = None
        self._incompleteMsgs = []   # List of tuples (incompleteMsg, incompleteURIList, arrivalTime)
        
        # Setup repeated calling of the clean up method
        LoopingCall(self._cleanUp).start(settings.MSG_QUQUE_TIMEOUT/4)
    
    def _recursiveURISearch(self, multidict):
        """ Internally used method to find binary data in incoming messages.
        """
        valueList = []
        
        for k,v in multidict.iteritems():
            if isinstance(v, dict):
                valueList += self._recursiveURISearch(v)
            elif k[-1] == '*':
                valueList.append((v, multidict, k))
        
        return valueList
    
    def _strMsgHandle(self, msg):
        """ Internally used method to handle incoming string messages.
        """
        data = msg['data']
        
        if msg['type']==self._CREATE_CONTAINER:
            if not self._robot:
                self._robot = Robot(self._commManager, self._satelliteManager, self, msg['orig'])
            self._robot.createContainer(data['containerTag'])
        
        elif msg['type'] == self._DESTROY_CONTAINER:
            self._robot.destroyContainer(data['containerTag'])
        
        elif msg['type'] == self._CHANGE_COMPONENT:
            if 'addNodes' in data:
                for nodeConfig in data['addNodes']:
                    self._robot.addNode(msg['dest'], nodeConfig)
                
            if 'removeNodes' in data:
                for nodeID in data['removeNodes']:
                    self._robot.removeNode(msg['dest'], nodeID)
                
            if 'addInterfaces' in data:
                for interfaceConfig in data['addInterfaces']:
                    self._robot.addInterface( msg['dest'],
                                              interfaceConfig['name'],
                                              interfaceConfig['interfaceType'],
                                              interfaceConfig['className'] )

            if 'removeInterfaces' in data:
                for interfaceID in data['removeInterfaces']:
                    self._robot.removeInterface(msg['dest'], interfaceID)

            if 'setParam' in data:
                for k,v in  data['setParam'].items():
                    self._robot.setParam(msg['dest'], k, v)
                    
            if 'deleteParam' in data:
                for paramName in data['deleteParam']:
                    self._robot.deleteParam(msg['dest'],paramName)
            
        elif msg['type'] == self._MESSAGE:
            uriList = self._recursiveURISearch(data['msg'])
            if uriList:
                self._incompleteMsgs.append((msg, uriList, datetime.now()))
            else:
                self._robot.sendROSMsgToContainer( msg['dest'],
                                                   data['interfaceName'],
                                                   data['msg'] )
        else:
            raise InvalidRequest('This type is not supported')
    
    def _binaryMsgHandle(self, msg):
        """ Internally used method to handle incoming binary messages.
        """
        uri = msg[:32]
        binaryData = StringIO().write(msg[33:])
        
        # Find and replace URI with data
        for incompleteMsg in self._incompleteMsgs:
            msg, incompleteURIList, _ = incompleteMsg
            
            for incompleteUri in incompleteURIList:
                missingUri, msgDict, key = incompleteUri
                
                if uri == missingUri:
                    del msgDict[key]
                    msgDict[key[:-1]] = binaryData
                    incompleteURIList.remove(incompleteUri)
                    
                    if not incompleteURIList:
                        self._robot.sendROSMsgToContainer( msg['dest'],
                                                           msg['data']['interfaceName'],
                                                           msg['data']['msg'] )
                        self._incompleteMsgs.remove(incompleteMsg)
                    
                    return
    
    def onMessage(self, msg, binary):    
        """ Method is called by the Autobahn engine when a message has been received
            from the client.
        """
        if binary:
            self._binaryMsgHandle(msg)
        else:
            try:
                self._strMsgHandle(json.loads(msg))
            except Exception:   # TODO: Refine Error handling
                import traceback
                super(WebSocketCloudEngineProtocol, self).sendMessage(
                    'Error: {0}'.format(traceback.format_exc())
                )
    
    def _recursiveBinarySearch(self, multidict):
        """ Internally used method to find binary data in outgoing messages.
        """
        uriBinary = []
        keys = []
        
        for k,v in multidict.iteritems():
            if isinstance(v, dict):
                uriBinaryPart, multidictPart = self._out_recursiveBinarySearch(v)
                uriBinary += uriBinaryPart
                multidict[k] = multidictPart 
            elif isinstance(v, StringIO):
                keys.append(k)
        
        for k in keys:
            tmpURI = uuid.uuid4().hex
            uriBinary.append((tmpURI, v))
            del multidict[k]
            multidict['{0}*'.format(k)] = tmpURI
        
        return uriBinary, multidict
    
    def sendMessage(self, msg):
        """ This method is called by the Robot instance to send a message to the robot.
            
            @param msg:     Message which should be sent
        """
        URIBinary, msgURI = self._out_recursiveBinaryDataSearch(msg)
        
        super(WebSocketCloudEngineProtocol, self).sendMessage(json.dumps(msgURI))
        
        for binData in URIBinary:
            super(WebSocketCloudEngineProtocol, self).sendMessage( binData[0] + binData[1].getvalue(),
                                                                   binary=True )
            
    def connectionLost(self, reason):
        """ Method is called by the twisted engine when the connection has been lost.
        """
        self._robot = None
    
    def _cleanUp(self):
        """ Internally used method to remove old incomplete messages.
        """
        limit = datetime.now() - timedelta(seconds=settings.MSG_QUQUE_TIMEOUT)
        
        for rm in [incompleteMsg for incompleteMsg in self._incompleteMsgs if incompleteMsg[2] < limit]:
            self._incompleteMsgs.remove(rm)

class WebSocketCloudEngineFactory(WebSocketServerFactory):
    """ Factory which is used for the connections from the robots to the reCloudEngine.
    """
    def __init__(self, commManager, satelliteManager, url):
        """ Initialize the Factory.
        """
        WebSocketServerFactory.__init__(self, url)
        
        self._commManager = commManager
        self._satelliteManager = satelliteManager
    
    def buildProtocol(self, addr):
        """ Method is called by the twisted reactor when a new connection attempt is made.
        """
        p = WebSocketCloudEngineProtocol(self._commManager, self._satelliteManager)
        p.factory = self
        return p
