#!/usr/bin/env python
# -*- coding: utf-8 -*-
#     
#     container.py
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
from core.manager import ContainerManager
from core.command import ControlDistributor, ContainerCommand
from comm import definition
from comm import types as msgTypes
from comm.manager import CommManager
from comm.protocol import RCEClientFactory
from remote.message import CommandSerializer, TagSerializer, \
    CommandProcessor, TagProcessor

from settings import MASTER_CONTAINER_PORT, CONF_DIR, DATA_DIR, ROOTFS, \
    ROOT_SRC_DIR, ROOT_PKG_DIR


class User(object):
    def __init__(self):
        self.containers = {}

ContainerManager._USER_CLS = User

ContainerManager._USE_SSL = False

ContainerManager._CONF_DIR = CONF_DIR
ContainerManager._DATA_DIR = DATA_DIR
ContainerManager._ROOTFS = ROOTFS
ContainerManager._ROOT_SRC_DIR = ROOT_SRC_DIR
ContainerManager._ROOT_PKG_DIR = ROOT_PKG_DIR


def main(reactor, commID, masterIP, masterPort, masterID):
    log.startLogging(sys.stdout)
    
    commManager = CommManager(reactor, commID)
    
    manager = ContainerManager(reactor)
    
    cmdSerializer = CommandSerializer()
    cmdSerializer.registerCommand([ContainerCommand])
    commManager.registerContentSerializers([cmdSerializer,
                                            TagSerializer()])
    
    distributor = ControlDistributor()
    distributor.addHandler(types.CONTAINER, manager.createContainer)
    distributor.addHandler(types.RM_CONTAINER, manager.destroyContainer)
    commManager.registerMessageProcessors([CommandProcessor(distributor),
                                           TagProcessor(distributor)])
    
    factory = RCEClientFactory(commManager, masterID)
    factory.addApprovedMessageTypes([msgTypes.COMMAND, msgTypes.TAG])
    reactor.connectTCP(masterIP, masterPort, factory)
    
    reactor.addSystemEventTrigger('before', 'shutdown', manager.shutdown)
    reactor.addSystemEventTrigger('before', 'shutdown', commManager.shutdown)
    
    reactor.run()


def _get_argparse():
    from argparse import ArgumentParser

    parser = ArgumentParser(prog='Container',
                            description='Manager for the containers of the '
                                        'framework in one machine.')

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
    
    commID = definition.PREFIX_PRIV_ADDR + args.uid
    
    main(reactor, commID, args.masterIP, MASTER_CONTAINER_PORT,
         definition.MASTER_ADDR)
