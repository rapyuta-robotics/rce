#!/usr/bin/env python
# -*- coding: utf-8 -*-
#     
#     master.py
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
import sys

# zope specific imports
from zope.interface import classImplements

# twisted specific imports
from twisted.python import log
from twisted.web.server import Site
#from autobahn.websocket import listenWS

# Custom imports
from core.interfaces import IControlFactory, IContainerControl, INodeControl, \
    IParameterControl, IEndpointControl
from core.manager import MasterManager
from core.machine import UIDServer, LoadBalancer
from core.command import ContainerCommand, RobotCommand, NodeCommand, \
    ParameterCommand, FileCommand, ConnectionCommand, \
    ServiceInterfaceCommand, ServiceProviderInterfaceCommand, \
    PublisherInterfaceCommand, SubscriberInterfaceCommand, \
    ServiceConverterCommand, ServiceProviderConverterCommand, \
    PublisherConverterCommand, SubscriberConverterCommand, \
    ServiceForwarderCommand, ServiceProviderForwarderCommand, \
    PublisherForwarderCommand, SubscriberForwarderCommand
from comm import definition
from comm import types
from comm.manager import CommManager
from comm.protocol import RCEServerFactory
from remote.callback import MasterCallbackFromRelay, \
    MasterCallbackFromContainer
from remote.control import RemoteRobotControl, RemoteNodeControl, \
    RemoteParameterControl, RemoteEndpointControl, RemoteContainerControl
from remote.message import CommandSerializer, TagSerializer, \
    RequestSerializer, RequestProcessor, ConnectDirectiveSerializer, \
    CommInfoSerializer
from client.handler import AuthenticationHandler
from client.protocol import MasterRobotAuthentication#, \
#    MasterWebSocketProtocol, CloudEngineWebSocketFactory

from settings import MASTER_CONTAINER_PORT, MASTER_RELAY_PORT, MASTER_UID_PORT



from functools import wraps
from types import UnboundMethodType

from util.interfaces import verifyClass

def _createForwardingMethod(attribute, target):
    """ Create a method which forwards the given method call to the saved
        instance.
        
        @param attribute:   Name of the attribute which contains the instance
                            matching the class of the target method.
        @type  attribute:   str
        
        @param target:      Target method to which this call should be
                            forwarded.
        @type  target:      UnboundMethod
    """
    if type(target) is not UnboundMethodType:
        raise TypeError('target has to be of type "UnboundMethod".')
    
    @wraps(target)
    def forwarder(self, *args):
        return target(getattr(self, attribute), *args)
    
    return forwarder

def _setupForwarding(clsDict, interface, attr, attrCls):
    """ Define all methods declared by the interface. All defined methods will
        forward the call to an instance attribute using the same method
        signature as the target methods.
                        
        @param clsDict:     Class dictionary where the created methods are
                            stored and which can then be used to create the
                            class. (Modifies the given reference)
        @type  clsDict:     dict
        
        @param interface:   Interface which is used to get the names of all the
                            methods which should be forwarded. The class
                            created using @param clsDict implements this
                            interface.
        @type  interface:   zope.interface.Interface
        
        @param attr:        The name of the attribute to which the calls will
                            be forwarded.
        @type  attr:        str
        
        @param attrCls:     The class of the attribute to which the calls will
                            be forwarded. It also has to implement the
                            interface @param interface.
    """
    verifyClass(interface, attrCls)
    
    for name in interface.names():
        clsDict[name] = _createForwardingMethod(attr, getattr(attrCls, name))

def _createControlFactory(commManager):
    createRobot = (lambda self, userID, commID:
        RemoteRobotControl(userID, commID, commManager))
    initCont = []
    clsDictCont = {}
    
    initCont.append(lambda self_, user, commID, ctrlID:
        setattr(self_, '_container', RemoteContainerControl(user, ctrlID, 
                                                            commManager)))
    initCont.append(lambda self_, user, commID, ctrlID:
        setattr(self_, '_endpoint', RemoteEndpointControl(user, commID,
                                                          commManager)))
    initCont.append(lambda self_, user, commID, ctrlID:
        setattr(self_, '_parameter', RemoteParameterControl(user, commID,
                                                            commManager)))
    initCont.append(lambda self_, user, commID, ctrlID:
        setattr(self_, '_node', RemoteNodeControl(user, commID, commManager)))
    
    _setupForwarding(clsDictCont, IContainerControl, '_container',
                     RemoteContainerControl)
    _setupForwarding(clsDictCont, IParameterControl, '_parameter',
                     RemoteParameterControl)
    _setupForwarding(clsDictCont, IEndpointControl, '_endpoint',
                     RemoteEndpointControl)
    _setupForwarding(clsDictCont, INodeControl, '_node', RemoteNodeControl)
    
    def __init__(self, userID, commID, ctrlID):
        for f in initCont:
            f(self, userID, commID, ctrlID)
    
    clsDictCont['__init__'] = __init__
    
    containerCls = type('ContainerControl', (object,), clsDictCont)
    
    classImplements(containerCls, IContainerControl)
    classImplements(containerCls, INodeControl)
    classImplements(containerCls, IParameterControl)
    classImplements(containerCls, IEndpointControl)
    
    clsDict = {}
    clsDict['createRobotControl'] = createRobot
    clsDict['createContainerControl'] = (lambda self, userID, commID, ctrlID:
        containerCls(userID, commID, ctrlID))
    
    factory = type('ControlFactroy', (object,), clsDict)
    classImplements(factory, IControlFactory)
    
    return factory()


def main(reactor, commID, uidPort, containerPort, relayPort):
    #f = open('/home/rce-user/master.log', 'w')
    #log.startLogging(f)
    log.startLogging(sys.stdout)
    
    manager = MasterManager(reactor)
    commManager = CommManager(reactor, commID)
    loadBalancer = LoadBalancer(commManager)
    uidServer = UIDServer(loadBalancer, 30)
    manager.registerLoadBalancer(loadBalancer)
    
    manager.registerControlFactory(_createControlFactory(commManager))
    
    cmdSerializer = CommandSerializer()
    cmdSerializer.registerCommand(
        [ContainerCommand, RobotCommand, NodeCommand,
         ParameterCommand, FileCommand, ConnectionCommand,
         ServiceInterfaceCommand, ServiceProviderInterfaceCommand,
         PublisherInterfaceCommand, SubscriberInterfaceCommand,
         ServiceConverterCommand, ServiceProviderConverterCommand,
         PublisherConverterCommand, SubscriberConverterCommand,
         ServiceForwarderCommand, ServiceProviderForwarderCommand,
         PublisherForwarderCommand, SubscriberForwarderCommand])
    commManager.registerContentSerializers([ConnectDirectiveSerializer(),
                                            CommInfoSerializer(),
                                            cmdSerializer,
                                            TagSerializer(),
                                            RequestSerializer()])
    
    commManager.registerMessageProcessors([RequestProcessor(manager)])
    
    reactor.listenTCP(uidPort, uidServer)
    
    cb = MasterCallbackFromContainer(uidServer)
    factory = RCEServerFactory(commManager, [cb], [cb])
    reactor.listenTCP(containerPort, factory)
    
    cb = MasterCallbackFromRelay(uidServer)
    factory = RCEServerFactory(commManager, [cb], [cb])
    factory.addApprovedMessageTypes([types.REQUEST])
    reactor.listenTCP(relayPort, factory)
    
#    factory = CloudEngineWebSocketFactory(MasterWebSocketProtocol, manager,
#                                          'ws://localhost:9000')
#    listenWS(factory)
    
    handler = AuthenticationHandler(manager)
    reactor.listenTCP(9000, Site(MasterRobotAuthentication(handler)))
    
    reactor.addSystemEventTrigger('before', 'shutdown', uidServer.shutdown)
    reactor.addSystemEventTrigger('before', 'shutdown', manager.shutdown)
    reactor.addSystemEventTrigger('before', 'shutdown', commManager.shutdown)
    
    reactor.run()
    
    #f.close()


if __name__ == '__main__':
    from twisted.internet import reactor
    
    main(reactor, definition.MASTER_ADDR, MASTER_UID_PORT,
         MASTER_CONTAINER_PORT, MASTER_RELAY_PORT)
