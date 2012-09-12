#!/usr/bin/env python
# -*- coding: utf-8 -*-
#     
#     relay.py
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

# twisted specific imports
from twisted.python import log
from autobahn.websocket import listenWS

# Custom imports
from core.types import cmd as types
from core.manager import RelayManager, RobotManager
from core.command import ControlDistributor, RobotCommand, \
    ServiceConverterCommand, ServiceProviderConverterCommand, \
    PublisherConverterCommand, SubscriberConverterCommand, \
    ConnectionCommand
from comm import definition
from comm import types as msgTypes
from comm.manager import CommManager
from comm.protocol import RCEClientFactory, RCEServerFactory
from remote.message import CommandSerializer, TagSerializer, \
    CommandProcessor, TagProcessor, ROSMsgSerializer, Messenger
from remote.control import RemoteRequestSender
from remote.callback import RelayCallbackFromEndpoint, RelayCallbackFromRelay
from client.protocol import RobotWebSocketProtocol, CloudEngineWebSocketFactory

from settings import MASTER_RELAY_PORT, RELAY_ROS_PORT, RELAY_RELAY_PORT, \
    CONVERTER_CLASSES


class User(object):
    def __init__(self):
        self.robots = {}
        self.interfaces = {}


class Manager(RelayManager, RobotManager):
    _USER_CLS = User
    _CUSTOM_CONVERTERS = CONVERTER_CLASSES
    _ROBOT_TIMEOUT = 60
    
    def __init__(self, reactor):
        super(Manager, self).__init__(reactor)


def main(reactor, commID, masterIP, masterPort, masterID, rosPort, relayPort):
    log.startLogging(sys.stdout)
    
    commManager = CommManager(reactor, commID)
    
    manager = Manager(reactor)
    
    sender = RemoteRequestSender(commManager)
    manager.registerRequestSender(sender)
    messenger = Messenger(manager, commManager)
    manager.registerMessenger(messenger)
    
    cmdSerializer = CommandSerializer()
    cmdSerializer.registerCommand([RobotCommand,
        ServiceConverterCommand, ServiceProviderConverterCommand,
        PublisherConverterCommand, SubscriberConverterCommand,
        ConnectionCommand])
    commManager.registerContentSerializers([cmdSerializer,
                                            TagSerializer(),
                                            ROSMsgSerializer()])
    
    distributor = ControlDistributor()
    distributor.addHandler(types.ROBOT, manager.addRobot)
    distributor.addHandler(types.RM_ROBOT, manager.removeRobot)
    distributor.addHandler(types.INTERFACE_SRV, manager.addInterface)
    distributor.addHandler(types.INTERFACE_PRO, manager.addInterface)
    distributor.addHandler(types.INTERFACE_PUB, manager.addInterface)
    distributor.addHandler(types.INTERFACE_SUB, manager.addInterface)
    distributor.addHandler(types.RM_INTERFACE, manager.removeInterface)
    distributor.addHandler(types.CONNECTION, manager.modifyConnection)
    
    commManager.registerMessageProcessors([CommandProcessor(distributor),
                                           TagProcessor(distributor),
                                           messenger])
    
    factory = RCEClientFactory(commManager, masterID)
    factory.addApprovedMessageTypes([msgTypes.COMMAND, msgTypes.TAG])
    reactor.connectTCP(masterIP, masterPort, factory)
    
    cb = RelayCallbackFromRelay(manager)
    factory = RCEServerFactory(commManager, [cb], [cb])
    factory.addApprovedMessageTypes([msgTypes.ROS_MSG])
    reactor.listenTCP(relayPort, factory)
    
    cb = RelayCallbackFromEndpoint(manager)
    factory = RCEServerFactory(commManager, [cb], [cb])
    factory.addApprovedMessageTypes([msgTypes.ROS_MSG])
    reactor.listenTCP(rosPort, factory)
    
    factory = CloudEngineWebSocketFactory(RobotWebSocketProtocol, manager,
                                          'ws://localhost:9050')
    listenWS(factory)
    
    reactor.addSystemEventTrigger('before', 'shutdown', manager.shutdown)
    reactor.addSystemEventTrigger('before', 'shutdown', commManager.shutdown)
    
    reactor.run()


def _get_argparse():
    from argparse import ArgumentParser

    parser = ArgumentParser(prog='Relay',
                            description='Manager for robots connections to '
                                        'containers of this machine.')

    parser.add_argument('uid', help='Unique ID which is used to identify this '
                                    'machine.')
    parser.add_argument('masterIP', help='IP address of master process.')

    return parser


if __name__ == '__main__':
    from twisted.internet import reactor
    
    args = _get_argparse().parse_args()
    
    if not definition.validateSuffix(args.uid):
        print 'UID is not a valid.'
        exit(1)
    
    commID = definition.PREFIX_PUB_ADDR + args.uid
    
    main(reactor, commID, args.masterIP, MASTER_RELAY_PORT,
         definition.MASTER_ADDR, RELAY_ROS_PORT, RELAY_RELAY_PORT)
