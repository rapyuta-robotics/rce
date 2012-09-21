#!/usr/bin/env python
# -*- coding: utf-8 -*-
#     
#     client.py
#     
#     This file is part of the RoboEarth Cloud Engine framework.
#     
#     This file was originally created for RoboEearth
#     http://www.roboearth.org/
#     
#     The research leading to these results has received funding from
#     the European Union Seventh Framework Programme FP7/2007-2013 under
#     grant agreement no248942 RoboEarth.
#     
#     Copyright 2012 RoboEarth
#     
#     Licensed under the Apache License, Version 2.0 (the "License");
#     you may not use this file except in compliance with the License.
#     You may obtain a copy of the License at
#     
#     http://www.apache.org/licenses/LICENSE-2.0
#     
#     Unless required by applicable law or agreed to in writing, software
#     distributed under the License is distributed on an "AS IS" BASIS,
#     WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#     See the License for the specific language governing permissions and
#     limitations under the License.
#     
#     \author/s: Dominique Hunziker 
#     
#     

import json
from uuid import uuid4

try:
    from cStringIO import StringIO, InputType, OutputType
    from StringIO import StringIO as pyStringIO
    
    def _checkIsStringIO(obj):
        return isinstance(obj, (InputType, OutputType, pyStringIO))
except ImportError:
    from StringIO import StringIO
    
    def _checkIsStringIO(obj):
        return isinstance(obj, StringIO)

from twisted.internet.defer import Deferred
from autobahn.websocket import WebSocketClientFactory, \
    WebSocketClientProtocol, connectWS

import msgTypes as types
from assembler import recursiveBinarySearch, MessageAssembler


class _RCERobotProtocol(WebSocketClientProtocol):
    def __init__(self, connection):
        self._connection = connection
        self._assembler = MessageAssembler(self, 30)
    
    def onConnect(self, _):
        self._assembler.start()
        self._connection.registerConnection(self)
    
    def onMessage(self, msg, binary):
        self._assembler.processMessage(msg, binary)
    
    def processCompleteMessage(self, msg):
        self._connection.receivedMessage(msg)
    
    def onClose(self, *_):
        self._connection.unregisterConnection(self)
        self._assembler.stop()


class _RCERobotFactory(WebSocketClientFactory):
    def __init__(self, url, connection):
        WebSocketClientFactory.__init__(self, url)
        self._connection = connection
    
    def buildProtocol(self, addr):
        p = _RCERobotProtocol(self._connection)
        p.factory = self
        return p


class _RCEMasterProtocol(WebSocketClientProtocol):
    def __init__(self, userID, robotID, deferred):
        self._userID = userID
        self._robotID = robotID
        self._deferred = deferred
    
    def onConnect(self, _):
        self.sendMessage(json.dumps({'userID' : self._userID,
                                     'robotID' : self._robotID}))
    
    def onMessage(self, msg, binary):
        if binary:
            print('Received message should not be binary.')
            return
        
        try:
            msg = json.loads(msg)
        except ValueError:
            print('Received message has an invalid format.')
            return
        
        try:
            msgType = msg['type']
            data = msg['data']
        except KeyError as e:
            print('Could not authenticate user/robot with master.\n'
                  'Response is missing "{0}".'.format(e))
            return
        
        if msgType == types.ERROR:
            print('Received error message from master: {0}'.format(data))
        elif msgType == types.STATUS:
            try:
                self._deferred.callback((data['key'], data['url']))
            except KeyError as e:
                print('Could not authenticate user/robot with master.\n'
                      'Status message is missing "{0}".'.format(e))
        else:
            print('Received message has invalid type.')


class _RCEMasterFactory(WebSocketClientFactory):
    def __init__(self, url, userID, robotID, deferred):
        WebSocketClientFactory.__init__(self, url)
        self._userID = userID
        self._robotID = robotID
        self._deferred = deferred
    
    def buildProtocol(self, addr):
        p = _RCEMasterProtocol(self._userID, self._robotID, self._deferred)
        p.factory = self
        return p


class Connection(object):
    class _Subscriber(object):
        def __init__(self, conn, iTag, msgType, cb):
            self._conn = conn
            self._iTag = iTag
            self._msgType = msgType
            self._cb = cb
        
        def unsubscribe(self):
            self._conn._unsubscribe(self._iTag, self)
        
        def callback(self, msgType, msg):
            if not msgType == self._msgType:
                raise TypeError('Received unexpected message type.')
            
            self._cb(msg)
        
        def __eq__(self, other):
            return self._msgType == other._msgType and self._cb == other._cb
        
        def __ne__(self, other):
            return not self.__eq__(other)
        
        def __hash__(self):
            return hash(self._msgType)^hash(self._cb)
    
    def __init__(self, userID, robotID, reactor):
        self._init = False
        self._userID = userID
        self._robotID = robotID
        self._reactor = reactor
        
        self._conn = None
        self._connectedDeferred = None
        
        self._subscribers = {}
        self._responses = {}
    
    def registerConnection(self, conn):
        if self._conn:
            raise AttributeError('There is already a connection registered.')
        
        self._conn = conn
        self._sendMessage({'type':types.INIT,
                           'data':{'userID':self._userID,
                                   'robotID':self._robotID,
                                   'key':self._key}})
    
    def unregisterConnection(self, conn):
        self._init = False
        
        if not self._conn:
            raise AttributeError('There is no connection registered.')
        
        if self._conn != conn:
            raise ValueError('This connection is not registered.')
        
        self._conn = None
    
    def connect(self, masterUrl, connectedDeferred):
        self._connectedDeferred = connectedDeferred
        deferred = Deferred()
        
        def authenticate(resp):
            self._key, url = resp
            
            lb = 2+url.find('//')
            rb = url.rfind(':')
            
            if url[lb:rb] == '127.0.0.1':
                mlb = 2+masterUrl.find('//')
                mrb = masterUrl.rfind(':')
                url = '{0}{1}{2}'.format(url[:lb], masterUrl[mlb:mrb],
                                         url[rb:])
            
            factory = _RCERobotFactory(url, self)
            self._reactor.callLater(1, connectWS, factory)
        
        deferred.addCallbacks(authenticate)
        
        factory = _RCEMasterFactory(masterUrl, self._userID, self._robotID,
                                    deferred)
        connectWS(factory)
    
    def close(self):
        if self._conn:
            self._conn.dropConnection()
        
        self._subscribers = {}
    
    def _sendMessage(self, msg):
        if not self._conn:
            raise ValueError('No connection registered.')
        
        uriBinary, msgURI = recursiveBinarySearch(msg)
        
        self._conn.sendMessage(json.dumps(msgURI))
        
        for data in uriBinary:
            self._conn.sendMessage(data[0] + data[1].getvalue(), binary=True)
    
    def createContainer(self, cTag):
        self._sendMessage({'type':types.CREATE_CONTAINER,
                           'data':{'containerTag':cTag}})
    
    def destroyContainer(self, cTag):
        self._sendMessage({'type':types.DESTROY_CONTAINER,
                           'data':{'containerTag':cTag}})
    
    def addNode(self, cTag, nTag, pkg, exe, namespace):
        self._sendMessage({'type':types.CONFIGURE_COMPONENT,
                           'data':{'addNodes':[{'containerTag':cTag,
                                                'nodeTag':nTag,
                                                'pkg':pkg,
                                                'exe':exe,
                                                'namespace':namespace}]}})
    
    def removeNode(self, cTag, nTag):
        self._sendMessage({'type':types.CONFIGURE_COMPONENT,
                           'data':{'removeNodes':[{'containerTag':cTag,
                                                   'nodeTag':nTag}]}})
    
    def addParameter(self, cTag, name, paramType, value):
        if paramType not in ['int', 'str', 'float', 'bool', 'file']:
            raise TypeError('Parameter type is not valid.')
        
        self._sendMessage({'type':types.CONFIGURE_COMPONENT,
                           'data':{'setParam':[{'containerTag':cTag,
                                                'name':name,
                                                'value':value,
                                                'paramType':paramType}]}})
    
    def removeParameter(self, cTag, name):
        self._sendMessage({'type':types.CONFIGURE_COMPONENT,
                           'data':{'deleteParam':[{'containerTag':cTag,
                                                   'name':name}]}})
    
    def addInterface(self, eTag, iTag, iType, iCls, addr):
        if iType not in ['ServiceInterface', 'ServiceProviderInterface',
                         'PublisherInterface', 'SubscriberInterface',
                         'ServiceConverter', 'ServiceProviderConverter',
                         'PublisherConverter', 'SubscriberConverter']:
            raise TypeError('Interface type is not valid.')
        
        self._sendMessage({'type':types.CONFIGURE_COMPONENT,
                           'data':{'addInterfaces':[{'endpointTag':eTag,
                                                     'interfaceTag':iTag,
                                                     'interfaceType':iType,
                                                     'className':iCls,
                                                     'addr':addr}]}})
    
    def removeInterface(self, iTag):
        self._sendMessage({'type':types.CONFIGURE_COMPONENT,
                           'data':{'removeInterfaces':[iTag]}})
    
    def addConnection(self, iTag1, iTag2):
        self._sendMessage({'type':types.CONFIGURE_CONNECTION,
                           'data':{'connect':[{'tagA':iTag1,
                                               'tagB':iTag2}]}})
    
    def removeConnection(self, iTag1, iTag2):
        self._sendMessage({'type':types.CONFIGURE_CONNECTION,
                           'data':{'disconnect':[{'tagA':iTag1,
                                                  'tagB':iTag2}]}})
    
    def publish(self, iTag, msgType, msg):
        self._sendMessage({'type':types.DATA_MESSAGE,
                           'data':{'dest':iTag,
                                   'orig':self._robotID,
                                   'type':msgType,
                                   'msgID':'nil',
                                   'msg':msg}})
    
    def subscribe(self, iTag, msgType, cb):
        if not callable(cb):
            raise TypeError('Callback has to be callable.')
        
        subscriber = Connection._Subscriber(self, iTag, msgType, cb)
        
        if iTag not in self._subscribers:
            self._subscribers[iTag] = set()
        
        subscribers = self._subscribers[iTag]
        
        if subscriber in subscribers:
            raise ValueError('Same subscription already registered.')
        
        subscribers.add(subscriber)
        return subscriber
    
    def callService(self, iTag, srvType, req, cb):
        if not callable(cb):
            raise TypeError('Callback has to be callable.')
        
        uid = uuid4().hex
        deferred = Deferred()
        deferred.addCallback(cb)
        
        self._responses[uid] = (deferred, srvType)
        
        self._sendMessage({'type':types.DATA_MESSAGE,
                           'data':{'dest':iTag,
                                   'orig':self._robotID,
                                   'type':srvType,
                                   'msgID':uid,
                                   'msg':req}})
    
    def _unsubscribe(self, iTag, subscriber):
        if iTag not in self._subscribers:
            raise ValueError('No subscribers for interface tag.')
        
        subscribers = self._subscribers[iTag]
        subscribers.remove(subscriber)
        
        if not subscribers:
            del self._subscribers[iTag]
    
    def _processDataMessage(self, dataMsg):
        msgType = dataMsg['type']
        msg = dataMsg['msg']
        
        if dataMsg['dest'] != self._robotID:
            print('Received message which was not for this connection.')
            return
        
        response = self._responses.pop(dataMsg['msgID'], None)
        
        if response:
            deferred, responseMsgType = response
            
            if msgType != responseMsgType:
                raise TypeError('Received unexpected message type.')
            
            deferred.callback(msg)
        else:
            for subscriber in self._subscribers.get(dataMsg['orig'],
                                                    set()).copy():
                subscriber.callback(msgType, msg)
    
    def receivedMessage(self, msg):
        try:
            msgType = msg['type']
            data = msg['data']
        except KeyError as e:
            raise ValueError('Received message from robot manager is missing '
                             'the key {0}.'.format(e))
        
        if msgType == types.ERROR:
            print('Received error message: {0}'.format(data))
        elif msgType == types.STATUS:
            print('Received status message: {0}'.format(data))
            self._init = True
            
            if self._connectedDeferred:
                self._connectedDeferred.callback(self)
        elif self._init and msgType == types.DATA_MESSAGE:
            self._processDataMessage(data)
