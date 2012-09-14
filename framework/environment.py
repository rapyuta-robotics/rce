#!/usr/bin/env python
# -*- coding: utf-8 -*-
#     
#     environment.py
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

# Custom imports
from core.types import cmd as types
from core.manager import NodeFwdManager, ParameterManager, \
    ROSInterfaceOnlyManager
from core.command import ControlDistributor, NodeForwarderCommand, \
    IntCommand, StrCommand, FloatCommand, BoolCommand, FileCommand, \
    ServiceInterfaceCommand, ServiceProviderInterfaceCommand, \
    PublisherInterfaceCommand, SubscriberInterfaceCommand, ConnectionCommand
from comm import definition
from comm import types as msgTypes
from comm.manager import CommManager
from comm.protocol import RCEClientFactory
from remote.message import CommandSerializer, TagSerializer, \
    CommandProcessor, TagProcessor, ROSMsgSerializer, Messenger

from settings import RELAY_ROS_PORT, ROS_NODE_PORT


class User(object):
    def __init__(self):
        self.nodes = {}
        self.parameters = {}
        self.interfaces = {}


class Manager(NodeFwdManager, ParameterManager, ROSInterfaceOnlyManager):
    _USER_CLS = User
    
    def __init__(self, reactor):
        super(Manager, self).__init__(reactor)


def main(reactor, commID, relayIP, relayPort, relayID, nodeIP, nodePort,
         nodeID):
    f = open('/opt/rce/data/env.log', 'w')
    log.startLogging(f)
    
    commManager = CommManager(reactor, commID)
    manager = Manager(reactor)
    manager.registerCommManager(commManager)
    messenger = Messenger(manager, commManager)
    manager.registerMessenger(messenger)
    
    cmdSerializer = CommandSerializer()
    cmdSerializer.registerCommand([NodeForwarderCommand,
        IntCommand, StrCommand, FloatCommand, BoolCommand, FileCommand,
        ServiceInterfaceCommand, ServiceProviderInterfaceCommand,
        PublisherInterfaceCommand, SubscriberInterfaceCommand,
        ConnectionCommand])
    commManager.registerContentSerializers([cmdSerializer,
                                            TagSerializer(),
                                            ROSMsgSerializer()])
    
    distributor = ControlDistributor()
    distributor.addHandler(types.NODE, manager.addNode)
    distributor.addHandler(types.RM_NODE, manager.removeNode)
    distributor.addHandler(types.PARAM_INT, manager.addParameter)
    distributor.addHandler(types.PARAM_STR, manager.addParameter)
    distributor.addHandler(types.PARAM_FLOAT, manager.addParameter)
    distributor.addHandler(types.PARAM_BOOL, manager.addParameter)
    distributor.addHandler(types.PARAM_FILE, manager.addParameter)
    distributor.addHandler(types.RM_PARAMETER, manager.removeParameter)
    distributor.addHandler(types.INTERFACE_SRV, manager.addInterface)
    distributor.addHandler(types.INTERFACE_PRO, manager.addInterface)
    distributor.addHandler(types.INTERFACE_PUB, manager.addInterface)
    distributor.addHandler(types.INTERFACE_SUB, manager.addInterface)
    distributor.addHandler(types.RM_INTERFACE, manager.removeInterface)
    distributor.addHandler(types.CONNECTION, manager.modifyConnection)
    
    commManager.registerMessageProcessors([CommandProcessor(distributor),
                                           TagProcessor(distributor),
                                           messenger])
    
    factory = RCEClientFactory(commManager, relayID)
    factory.addApprovedMessageTypes([msgTypes.COMMAND, msgTypes.TAG,
                                     msgTypes.ROS_MSG])
    reactor.connectTCP(relayIP, relayPort, factory)
    
    factory = RCEClientFactory(commManager, nodeID)
    reactor.connectTCP(nodeIP, nodePort, factory)
    
    reactor.addSystemEventTrigger('before', 'shutdown', manager.shutdown)
    reactor.addSystemEventTrigger('before', 'shutdown', commManager.shutdown)
    
    reactor.run()
    
    f.close()


def _get_argparse():
    from argparse import ArgumentParser

    parser = ArgumentParser(prog='Environment',
                            description='Manager for ROS components inside a '
                                        'container.')

    parser.add_argument('commID', help='Communication ID of this process.')
    parser.add_argument('relayIP', help='IP address of relay process of this '
                                        'machine.')
    parser.add_argument('relayID', help='Communication ID of relay process of '
                                        'this machine.')

    return parser


if __name__ == '__main__':
    from twisted.internet import reactor
    
    args = _get_argparse().parse_args()
    
    if not definition.validateAddress(args.commID):
        print 'Communication ID of this process is not a valid.'
        exit(1)
    
    if not definition.validateAddress(args.relayID):
        print 'Communication ID of relay process is not a valid.'
        exit(1)
    
    main(reactor, args.commID, args.relayIP, RELAY_ROS_PORT,
         args.relayID, 'localhost', ROS_NODE_PORT, definition.NEIGHBOR_ADDR)
