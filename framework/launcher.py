#!/usr/bin/env python
# -*- coding: utf-8 -*-
#     
#     launcher.py
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
from core.manager import NodeManager
from core.command import ControlDistributor, NodeCommand
from comm import definition
from comm import types as msgTypes
from comm.manager import CommManager
from comm.protocol import RCEServerFactory
from remote.message import CommandSerializer, TagSerializer, \
    CommandProcessor, TagProcessor

from settings import ROS_NODE_PORT


class User(object):
    def __init__(self):
        self.nodes = {}

NodeManager._USER_CLS = User


def main(reactor, commID, port):
    log.startLogging(sys.stdout)
    
    commManager = CommManager(reactor, commID)
    
    manager = NodeManager(reactor)
    
    cmdSerializer = CommandSerializer()
    cmdSerializer.registerCommand([NodeCommand])
    commManager.registerContentSerializers([cmdSerializer,
                                            TagSerializer()])
    
    distributor = ControlDistributor()
    distributor.addHandler(types.NODE, manager.addNode)
    distributor.addHandler(types.RM_NODE, manager.removeNode)
    commManager.registerMessageProcessors([CommandProcessor(distributor),
                                           TagProcessor(distributor)])
    
    factory = RCEServerFactory(commManager)
    factory.addApprovedMessageTypes([msgTypes.COMMAND, msgTypes.TAG])
    reactor.listenTCP(port, factory)
    
    reactor.addSystemEventTrigger('before', 'shutdown', manager.shutdown)
    reactor.addSystemEventTrigger('before', 'shutdown', commManager.shutdown)
    
    reactor.run()


if __name__ == '__main__':
    from twisted.internet import reactor
    
    main(reactor, definition.NEIGHBOR_ADDR, ROS_NODE_PORT)
