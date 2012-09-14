#!/usr/bin/env python
# -*- coding: utf-8 -*-
#     
#     machine.py
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

# Python specific imports
from datetime import datetime, timedelta

# zope specific imports
from zope.interface import implements

# twisted specific imports
from twisted.internet.error import ConnectionDone
from twisted.internet.protocol import ClientFactory, ServerFactory
from twisted.protocols.basic import LineReceiver
from twisted.internet.task import LoopingCall

# Custom imports
from errors import InternalError, InvalidRequest
from core.interfaces import ILoadBalancer
from comm import definition
from comm import types as msgTypes
from comm.message import Message
from util.ssl import createKey, createCertReq, dumpCertReq, parseCertReqStr, \
    createCert, dumpCert


class _UIDClientProtocol(LineReceiver):
    """ Protocol which is used by a client to retrieve a new UID for a machine.
    """
    def __init__(self, deferred):
        """ Initialize UIDClientProtocol.
            
            @param deferred:    Deferred which should be called with the
                                received UID.
            @type  deferred:    Deferred
        """
        self._deferred = deferred
        self._uid = None
    
    def lineReceived(self, line):
        """ Callback which is called by twisted when a line has been received.
        """
        if not self._uid:
            if not definition.validateSuffix(line):
                raise ValueError('Received address suffix is not valid.')
            
            self._uid = line
    
    def connectionLost(self, reason):
        """ Callback which is called by twisted when the connection is lost.
        """
        if reason.check(ConnectionDone):
            self._deferred.callback(self._uid)
        else:
            self._deferred.errback(reason.getErrorMessage())


class _UIDServerProtocol(LineReceiver):
    """ Protocol which is used by the server to send a new UID for a machine.
    """
    def __init__(self, uidServer):
        """ Initialize the UIDServerProtocol.
            
            @param uidServer:   Instance which is used to provide the new UIDs.
            @type  uidServer:   core.machine.UIDServer /
                                core.machine.SSLServer
        """
        self._uidServer = uidServer
    
    def connectionMade(self):
        """ Called by twisted when connection is established.
        """
        self.transport.write('{0}{1}'.format(self._uidServer.getUID(),
                                             self.delimiter))
        self.transport.loseConnection()
    
    def lineReceived(self, line):
        """ Called by twisted when a line has been received.
        """
        pass


class UIDClient(ClientFactory):
    """ Factory which is used by a client to retrieve a new UID for a machine.
    """
    def __init__(self, deferred):
        """ Initialize UIDClientFactory.
            
            @param deferred:    Deferred which should be called with the
                                received UID.
            @type  deferred:    Deferred
        """
        self._deferred = deferred
    
    def buildProtocol(self, addr):
        """ Callback which is called by twisted when a new connection should
            be established.
        """
        return _UIDClientProtocol(self._deferred)


class UIDServer(ServerFactory):
    """ Factory which is used by the server to distribute new UID for machines.
    """
    def __init__(self, loadBalancer, timeout):
        """ # TODO: Add description
        """
        self._loadBalancer = loadBalancer
        self._timeout = timeout
        
        self._pendingRelay = {}
        self._pendingContainer = {}
        
        self._usedUID = set()
        self._reservedUID = {}
        
        self._cleaner = LoopingCall(self._clean)
        self._cleaner.start(timeout/2)
    
    def buildProtocol(self, addr):
        """ Called by twisted framework when a new connection to the UIDServer
            is attempted.
        """
        return _UIDServerProtocol(self)
    
    def addRelay(self, ip, commID):
        """ # TODO: Add description
        """
        uid = commID[definition.PREFIX_LENGTH:]
        
        if not self._checkUID(uid):
            raise InvalidRequest('The used commID is not valid.')
        
        if ip in self._pendingRelay:
            raise InternalError('There is already a relay manager registered '
                                'for the machine "{0}".'.format(ip))
        
        if ip in self._pendingContainer:
            self._usedUID.add(uid)
            del self._reservedUID[uid]
            
            self._loadBalancer.registerMachine(
                Machine(ip, self._pendingContainer[ip], commID))
        else:
            self._reservedUID[uid] = datetime.now()
            self._pendingRelay[ip] = commID
    
    def removeRelay(self, ip, commID):
        """ # TODO: Add description
        """
        if ip in self._pendingRelay:
            del self._pendingRelay[ip]
        else:
            self._loadBalancer.unregisterMachine(ip)
            self._reservedUID[commID[definition.PREFIX_LENGTH:]] = (
                datetime.now())
    
    def addContainer(self, ip, commID):
        """ # TODO: Add description
        """
        uid = commID[definition.PREFIX_LENGTH:]
        
        if not self._checkUID(uid):
            raise InvalidRequest('The used commID is not valid.')
        
        if ip in self._pendingContainer:
            raise InternalError('There is already a container manager '
                                'registered for the machine "{0}".'.format(ip))
        
        if ip in self._pendingRelay:
            self._usedUID.add(uid)
            del self._reservedUID[uid]
            
            self._loadBalancer.registerMachine(
                Machine(ip, commID, self._pendingRelay[ip]))
        else:
            self._reservedUID[uid] = datetime.now()
            self._pendingContainer[ip] = commID
    
    def removeContainer(self, ip, commID):
        """ # TODO: Add description
        """
        if ip in self._pendingContainer:
            del self._pendingContainer[ip]
        else:
            self._loadBalancer.unregisterMachine(ip)
            self._reservedUID[commID[definition.PREFIX_LENGTH:]] = (
                datetime.now())
    
    def getUID(self):
        """ # TODO: Add description
        """
        while 1:
            uid = definition.generateSuffix()
            
            if uid not in self._usedUID and uid not in self._reservedUID:
                self._reservedUID[uid] = datetime.now()
                return uid
    
    def _checkUID(self, uid):
        """ Check if the given UID is in the list of not yet confirmed UIDs.
            
            @param uid:     UID which should be verified.
            @type  uid:     str
            
            @return:        True if the UID is valid; False otherwise
            @rtype:         bool
        """
        return uid in self._reservedUID
    
    def shutdown(self):
        """
        """
        self._cleaner.stop()
    
    def _clean(self):
        """ Method is regularly called to free no longer claimed UIDs.
        """
        limit = datetime.now() - timedelta(seconds=self._timeout)
        
        for uid in [uid for uid, timestamp in self._reservedUID.iteritems()
                    if timestamp < limit]:
            del self._reservedUID[uid]


class _SSLClientProtocol(LineReceiver):
    """ Protocol which is used by a client to retrieve a new UID and
        certificate for a machine.
    """
    def __init__(self, deferred):
        """ Initialize SSLClientProtocol.
            
            @param deferred:    Deferred which should be called with the
                                received UID, certificate and private key.
            @type  deferred:    Deferred
        """
        self._deferred = deferred
        self._buff = []
        
        self._uid = None
        self._key = createKey()
    
    def lineReceived(self, line):
        """ Callback which is called by twisted when a line has been received.
        """
        if not self._uid:
            if not definition.validateSuffix(line):
                raise ValueError('Received address suffix is not valid.')
            
            self._uid = line
            
            self.transport.write('{0}{1}{1}'.format(
                dumpCertReq(createCertReq(self._key,
                                          'Process-{0}'.format(line))),
                self.delimiter))
        else:
            self._buff.append(line)
    
    def connectionLost(self, reason):
        """ Callback which is called by twisted when the connection is lost.
        """
        if reason.check(ConnectionDone):
            self._deferred.callback((self._uid, '\n'.join(self._buff),
                                     self._key))
        else:
            self._deferred.errback(reason.getErrorMessage())


class _SSLServerProtocol(LineReceiver):
    """ Protocol which is used by the server to send a new UID and certificate
        for a machine.
    """
    def __init__(self, uidServer):
        """ Initialize the UIDServerProtocol.
            
            @param uidServer:   Instance which is used to provide the new UID
                                and certificate.
            @type  uidServer:   IUIDServer
        """
        self._uidServer = uidServer
        self._buff = []
    
    def connectionMade(self):
        """ Called by twisted when connection is established.
        """
        self.transport.write('{0}{1}'.format(self._uidServer.getUID(),
                                             self.delimiter))
    
    def lineReceived(self, line):
        """ Called by twisted when a line has been received.
        """
        if not line:
            self.transport.write(
                '{0}{1}'.format(self._uidServer.getCertificate(
                    '\n'.join(self._buff)), self.delimiter))
            self.transport.loseConnection()
        else:
            self._buff.append(line)


class SSLClient(ClientFactory):
    """ Factory which is used by a client to retrieve a new UID and certificate
        for a machine.
    """
    def __init__(self, deferred):
        """ Initialize UIDClientFactory.
            
            @param deferred:    Deferred which should be called with the
                                received UID, certificate and private key.
            @type  deferred:    Deferred
        """
        self._deferred = deferred
    
    def buildProtocol(self, addr):
        """ Callback which is called by twisted when a new connection should
            be established.
        """
        return _UIDClientProtocol(self._deferred)


class SSLServer(UIDServer):
    """ # TODO: Add description
    """
    def __init__(self, loadBalancer, timeout, cert, key):
        """ # TODO: Add description
        """
        UIDServer.__init__(self, loadBalancer, timeout)
        
        self._cert = cert
        self._key = key
    
    def buildProtocol(self, addr):
        """ Called by twisted framework when a new connection to the SSLServer
            is attempted.
        """
        return _SSLServerProtocol(self)
    
    def getCertificate(self, req):
        """ Callback method which provides a new certificate for a machine.
            
            @param req:     X509Req object as a string.
            @type  req:     str
            
            @return:        Resulting X509 object as a string.
            @rtype:         str
        """
        return dumpCert(createCert(parseCertReqStr(req), self._cert,
                                   self._key))
    

class Machine(object):
    """ # TODO: Add description
    """
    def __init__(self, ip, container, relay):
        """ Initialize the machine.
            
            @param ip:          IP address of the machine.
            @type  ip:          str
            
            @param container:   Communication ID of the container manager in
                                the machine.
            @type  container:   str
            
            @param relay:       Communication ID of the relay manager in the
                                machine.
            @type  relay:       str
        """
        self._ip = ip
        self._container = container
        self._relay = relay
        self._loadInfo = None
    
    def matchIP(self, ip):
        """ Check if the given IP address matches the IP address of this
            machine instance..
            
            @param ip:      IP address which should be checked.
            @type  ip:      str
            
            @return:        True if there is a match; False otherwise.
            @rtype:         bool
        """
        return self._ip == ip
    
    def updateLoadInfo(self, loadInfo):
        """ Update the current load info.
            
            @param loadInfo:    Newest load info from the machine.
            @type  loadInfo:    ???    # TODO: Define loadInfo
        """
        self._loadInfo = loadInfo
    
    def __eq__(self, other):
        return self._ip == other._ip
    
    def _ne__(self, other):
        return self._ip != other._ip
    
    def __hash__(self):
        return hash(self._ip)


class LoadBalancer(object):
    """ # TODO: Add description
    """
    implements(ILoadBalancer)
    
    def __init__(self, commMngr):
        """ # TODO: Add description
        """
        self._commManager = commMngr
        
        self._machines = set()
        self._relayIter = self._machines.__iter__()
        self._containerIter = self._machines.__iter__()
    
    def registerMachine(self, machine):
        """ Register a new machine in which containers can be started.
            
            @param machine:     Machine which should be registered.
            @type  machine:     core.balancer.Machine
        """
        if machine in self._machines:
            raise InternalError('Tried to register the same machine twice.')
        
        # Inform container manager about the CommID of the relay manager.
        msg = Message()
        msg.msgType = msgTypes.COMM_INFO
        msg.dest = machine._container
        msg.content = machine._relay
        self._commManager.sendMessage(msg)
        
        # Order the new machine to connect to all the other existing machines.
        order = [(machine._relay, machine._ip) for machine in self._machines]
        
        if order:
            msg = Message()
            msg.msgType = msgTypes.ROUTE_INFO
            msg.dest = machine._relay
            msg.content = order
            self._commManager.sendMessage(msg)
        
        self._machines.add(machine)
    
    def unregisterMachine(self, ip):
        """ Unregister a machine.
            
            @param ip:      IP address of the machine which should be
                            unregistered.
            @type  ip:      str
        """
        for machine in self._machines:
            if machine.matchIP(ip):
                break
        else:
            raise InternalError('Tried to unregister a non existent machine.')
        
        self._machines.remove(machine)
    
    def getNextRobotLocation(self, repeat=False):
        """ Returns the CommID and IP address of the robot manager where the
            next robot should be created.
        """
        try:
            machine = self._relayIter.next()
            return (machine._relay, machine._ip)
        except (StopIteration, RuntimeError):
            if repeat:
                raise InternalError('Can not get next robot location.')
            
            self._relayIter = self._machines.__iter__()
            return self.getNextRobotLocation(True)
    
    def getNextContainerLocation(self, repeat=False):
        """ Returns the CommID of the container manager where the next
            container should be created.
        """
        try:
            return self._containerIter.next()._container
        except (StopIteration, RuntimeError):
            if repeat:
                raise InternalError('Can not get next container location.')
            
            self._containerIter = self._machines.__iter__()
            return self.getNextContainerLocation(True)
