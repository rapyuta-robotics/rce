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

# Custom imports
from core.interfaces import IControlFactory, IContainerControl, INodeControl, \
    IParameterControl, IEndpointControl
from core.manager import MasterManager
from core.machine import UIDServer, LoadBalancer
from core.command import ContainerCommand, RobotCommand, NodeCommand, \
    IntCommand, StrCommand, FloatCommand, BoolCommand, FileCommand, \
    ServiceInterfaceCommand, ServiceConverterCommand, \
    ServiceProviderInterfaceCommand, ServiceProviderConverterCommand, \
    PublisherInterfaceCommand, PublisherConverterCommand, \
    SubscriberInterfaceCommand, SubscriberConverterCommand, ConnectionCommand
from comm import definition
from comm import types
from comm.manager import CommManager
from comm.protocol import RCEServerFactory
from remote.callback import MasterCallbackFromRelay, \
    MasterCallbackFromContainer
from remote.control import RemoteRobotControl, RemoteNodeControl, \
    RemoteParameterControl, RemoteEndpointControl, RemoteContainerControl
from remote.message import CommandSerializer, TagSerializer, \
    RequestSerializer, RequestProcessor

from settings import MASTER_CONTAINER_PORT, MASTER_RELAY_PORT, MASTER_UID_PORT


def _setupForwarding(clsDict, interface, obj):
    for name in interface.names():
        clsDict[name] = (lambda self_, *args:
            getattr(getattr(self_, obj), name)(*args))


def _createControlFactory(commManager):
    createRobot = (lambda self, userID, commID:
        RemoteRobotControl(userID, commID, commManager))
    initCont = []
    clsDictCont = {}
    
    initCont.append(lambda self_, user, commID, ctrlID:
        setattr(self_, '_container', RemoteContainerControl(user, ctrlID, 
                                                            commManager)))
    
    _setupForwarding(clsDictCont, IContainerControl, '_container')
    
    initCont.append(lambda self_, user, commID, ctrlID:
        setattr(self_, '_node', RemoteNodeControl(user, commID, commManager)))
    
    initCont.append(lambda self_, user, commID, ctrlID:
        setattr(self_, '_parameter', RemoteParameterControl(user, commID,
                                                            commManager)))
    
    initCont.append(lambda self_, user, commID, ctrlID:
        setattr(self_, '_endpoint', RemoteEndpointControl(user, commID,
                                                          commManager)))
    
    _setupForwarding(clsDictCont, INodeControl, '_node')
    _setupForwarding(clsDictCont, IParameterControl, '_parameter')
    _setupForwarding(clsDictCont, IEndpointControl, '_endpoint')
    
    clsDictCont['__init__'] = (lambda self_, userID, commID, ctrlID:
        map(lambda f: f(self_, userID, commID, ctrlID), initCont))
    
    containerCls = type('ContainerControl', (object,), clsDictCont)
    
    clsDict = {}
    clsDict['createRobotControl'] = createRobot
    clsDict['createContainerControl'] = (lambda self, userID, commID, ctrlID:
        containerCls(userID, commID, ctrlID))
    
    factory = type('ControlFactroy', (object,), clsDict)
    classImplements(factory, IControlFactory)
    
    return factory()


def main(reactor, commID, uidPort, containerPort, relayPort):
    log.startLogging(sys.stdout)
    
    commManager = CommManager(reactor, commID)
    
    manager = MasterManager(reactor)
    
    loadBalancer = LoadBalancer()
    uidServer = UIDServer(loadBalancer, 30)
    manager.registerLoadBalancer(loadBalancer)
    
    manager.registerControlFactory(_createControlFactory(commManager))
    
    cmdSerializer = CommandSerializer()
    cmdSerializer.registerCommand(
        [ContainerCommand, RobotCommand, NodeCommand,
         IntCommand, StrCommand, FloatCommand, BoolCommand, FileCommand,
         ServiceInterfaceCommand, ServiceConverterCommand,
         ServiceProviderInterfaceCommand, ServiceProviderConverterCommand,
         PublisherInterfaceCommand, PublisherConverterCommand,
         SubscriberInterfaceCommand, SubscriberConverterCommand,
         ConnectionCommand])
    commManager.registerContentSerializers([cmdSerializer,
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
    
    reactor.addSystemEventTrigger('before', 'shutdown', uidServer.shutdown)
    reactor.addSystemEventTrigger('before', 'shutdown', manager.shutdown)
    reactor.addSystemEventTrigger('before', 'shutdown', commManager.shutdown)
    
    reactor.run()


if __name__ == '__main__':
    from twisted.internet import reactor
    
    main(reactor, definition.MASTER_ADDR, MASTER_UID_PORT,
         MASTER_CONTAINER_PORT, MASTER_RELAY_PORT)
