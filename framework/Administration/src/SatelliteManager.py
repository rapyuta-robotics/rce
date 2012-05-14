#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       SatelliteManager.py
#       
#       Copyright 2012 dominique hunziker <dominique.hunziker@gmail.com>
#       
#       This program is free software; you can redistribute it and/or modify
#       it under the terms of the GNU General Public License as published by
#       the Free Software Foundation; either version 2 of the License, or
#       (at your option) any later version.
#       
#       This program is distributed in the hope that it will be useful,
#       but WITHOUT ANY WARRANTY; without even the implied warranty of
#       MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#       GNU General Public License for more details.
#       
#       You should have received a copy of the GNU General Public License
#       along with this program; if not, write to the Free Software
#       Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#       MA 02110-1301, USA.
#       
#       

# twisted specific imports
from twisted.python import log
from twisted.internet.ssl import ClientContextFactory, DefaultOpenSSLContextFactory
from twisted.internet.task import LoopingCall

# Python specific imports
import sys

# Custom imports
import settings
from Exceptions import InternalError
import Comm.Message.Definition as MsgDef
from Comm.Message.TypeBase import MessageTypes as MsgTypes
from Comm.Message.StdProcessor import ConnectDirectiveProcessor
from Comm.Message.ContainerProcessor import CreateEnvProcessor, DestroyEnvProcessor
from Comm.Message.Base import Message, validateAddress
from Comm.Manager import ReappengineManager
from Comm.Factory import ReappengineClientFactory, ReappengineServerFactory
from MiscUtility import generateID

class EnvironmentServerFactory(ReappengineServerFactory):
    """ Factory which is used in the satellite node for the connections to the environment
        nodes.
    """
    def __init__(self, manager):
        ReappengineServerFactory.__init__(self, manager)
        
        self._expectedConnections = {}
        self._upConnections = {}
    
    def addExpectedConnection(self, commID, key):
        """ Add a communication ID to the dictionary of expected incoming
            connections.
            
            @param commID:  CommID of expected connection.
            @type  commID:  str
            
            @param key:     Key which is used to authenticate the connection.
            @type  key:     str
        """
        self._expectedConnections[commID] = key
    
    def filterMessage(self, msgType):
        return msgType not in [ MsgTypes.ROUTE_INFO,
                                MsgTypes.ROS_RESPONSE,
                                MsgTypes.ROS_MSG ]
    
    def authOrigin(self, origin, key):
        if origin not in self._expectedConnections:
            log.msg('Received a initialization request from an unexpected source.')
            return False
        
        if self._expectedConnections[origin] != key:
            log.msg('Tried to initialize a connection without a valid key.')
            return False
        
        return True
    
    def getResponse(self, origin):    
        return { 'origin' : self.manager.commID, 'dest' : origin, 'key' : generateID() }
    
    def saveState(self, content):
        dest = content['dest']
        del self._expectedConnections[dest]
        self._upConnections[dest] = content['key']
    
    def postInitTrigger(self, origin):
        msg = Message()
        msg.msgType = MsgTypes.ROUTE_INFO
        msg.dest = origin
        msg.content = { None : self.manager.commID }
        self.manager.sendMessage(msg)
        self.manager.updateRoutingInfo({ origin : origin })
    
    def unregisterConnection(self, conn):
        dest = conn.dest
        self._expectedConnections[dest] = self._upConnections[dest]
        del self._upConnections[dest]
        
        ReappengineServerFactory.unregisterConnection(self, conn)

class SatelliteClientFactory(ReappengineClientFactory):
    """ Factory which is used in the satellite node for the connections to the
        other satellite nodes.
    """
    def filterMessage(self, msgType):
        return msgType not in [ MsgTypes.ROUTE_INFO,
                                MsgTypes.ROS_MSG ]
    
    def postInitTrigger(self, origin):
        msg = Message()
        msg.msgType = MsgTypes.ROUTE_INFO
        msg.dest = origin
        msg.content = self.manager.getSatelliteRouting()
        self.manager.sendMessage(msg)

class SatelliteServerFactory(ReappengineServerFactory):
    """ Factory which is used in the satellite node for the connection to the
        other satellite nodes.
    """
    def filterMessage(self, msgType):
        return msgType not in [ MsgTypes.ROUTE_INFO,
                                MsgTypes.ROS_MSG ]
    
    def getResponse(self, origin):
        return { 'origin' : self.manager.commID, 'dest' : origin, 'key' : '' }
    
    def postInitTrigger(self, origin):
        msg = Message()
        msg.msgType = MsgTypes.ROUTE_INFO
        msg.dest = origin
        msg.content = self.manager.getSatelliteRouting()
        self.manager.sendMessage(msg)

class ContainerClientFactory(ReappengineClientFactory):
    """ Factory which is used in the satellite node for the connection to the container node.
    """
    def filterMessage(self, msgType):
        return msgType not in [ MsgTypes.ROUTE_INFO,
                                MsgTypes.CONTAINER_STATUS ]
    
    def postInitTrigger(self, origin):
        msg = Message()
        msg.msgType = MsgTypes.ROUTE_INFO
        msg.dest = origin
        msg.content = { None : MsgDef.NEIGHBOR_ADDR }
        self.manager.sendMessage(msg)
        self.manager.containerMngrID = origin   # TODO: How to solve Identification problem with managerID?

class MasterClientFactory(ReappengineClientFactory):
    """ Factory which is used in the satellite node for the connection to the master node.
    """
    def filterMessage(self, msgType):
        return msgType not in [ MsgTypes.ENV_CREATE,
                                MsgTypes.ENV_DESTROY,
                                MsgTypes.ROUTE_INFO,
                                MsgTypes.CONNECT,
                                MsgTypes.ROS_MSG ]
    
    def postInitTrigger(self, origin):
        msg = Message()
        msg.msgType = MsgTypes.ROUTE_INFO
        msg.dest = origin
        msg.content = self.manager.getSatelliteRouting()
        self.manager.sendMessage(msg)

class SatelliteManager(ReappengineManager):
    """ Manager which is used for the satellites nodes, which represent the communication
        relay for the container nodes on a single machine.
    """
    def __init__(self, reactor, commID, ctx):
        """ Initialize the necessary variables for the environment manager.

            @param reactor:     TODO: Add description
            @type  reactor:     reactor

            @param commID:  CommID of this node which can be used by other
                            nodes to identify this node.
            @type  commID:  str
            
            @param ctx:     SSLContext which is used for the connections to
                            the other satellite nodes.
            @type  ctx:     # TODO: Determine type of argument
        """
        super(SatelliteManager, self).__init__(reactor, commID)
        
        # SSL Context which is used to connect to other satellites
        self._ctx = ClientContextFactory()
        self._ctx._contextFactory = ctx
        
        # Storage for all running containers
        self._containers = {}
        self.containerMngrID = None
        
        # Setup list of valid message processors
        self.msgProcessors.extend([ ConnectDirectiveProcessor(self),
                                    CreateEnvProcessor(self),
                                    DestroyEnvProcessor(self) ])     # TODO: Add all valid messages
    
    def createContainer(self, commID, homeFolder):
        """
        """
        if commID in self._containers:
            raise InternalError('There is already a container with the same CommID.')
        
        container = None # TODO: Add container object here
        
        self._containers[commID] = container
    
    def destroyContainer(self, commID):
        """
        """
        if commID not in self._containers:
            log.msg('Tried to terminate an nonexistent container.')
            return
        
        container = self._containers[commID]
        
        # TODO: Delete container
        
        del self._containers[commID]
    
    def getSatelliteRouting(self):
        """ Returns the routing information for all nodes which should be
            routed through this node, i.e. all container nodes managed by this
            satellite node and the satellite node itself.
            
            @rtype:     { str : str }
        """
        dest = self.commID
        msg = { dest : dest }
        
        for key in self._containers:
            msg[key] = dest
        
        return msg
    
    def connectToSatellites(self, satellites):
        """ Connect to specified satellites.
            
            @param satellites:  List of dictionaries containing the necessary
                                information each (ip, port, commID).
            @type  satellites:  [ { str : str } ]
        """
        for satellite in satellites:
            satelliteID = satellite['commID']
            ip = satellite['ip']
            port = satellite['port']
            self.reactor.connectSSL(ip, port, ReappengineClientFactory(self, satelliteID, ''), self._ctx)
    
    def updateLoadInfo(self):
        """ This method is called regularly and is used to send the newest load info
            to the master node/load balancer.
        """
        return # TODO: Until content is valid, keep this here
        
        msg = Message()
        msg.msgType = MsgTypes.ROUTE_INFO
        msg.dest = settings.LOAD_INFO_UPDATE
        msg.content = None # TODO: Add meaningful information
        self.sendMessage(msg)
    
    def shutdown(self):
        """ Method is called when the manager/factory is stopped.
        """
        super(SatelliteManager, self).shutdown()
        
        # TODO: Terminate the containers

def main(reactor, ip, port, commID, satelliteID, key):
    log.startLogging(sys.stdout)
    
    log.msg('Start initialization...')
    
    ctx = DefaultOpenSSLContextFactory('Comm/key.pem', 'Comm/cert.pem')
    
    # Create Manager
    manager = SatelliteManager(reactor, commID, ctx.getContext())

    # Initialize twisted
    log.msg('Initialize twisted')
    
    # Server for connections from the containers
    reactor.listenSSL(port, EnvironmentServerFactory(manager), ctx) # TODO: Select correct port
    
    # Server for connections from other satellites
    reactor.listenSSL(port, SatelliteServerFactory(manager), ctx) # TODO: Select correct port
    
    # Client for connection to ContainerLauncher
    reactor.connectSSL(ip, port, ContainerClientFactory(manager, MsgDef.NEIGHBOR_ADDR, ''), ctx)     # TODO: Select correct Factory, ip/port
    
    # Client for connection to Master
    reactor.connectSSL(ip, port, SatelliteClientFactory(manager, settings.MASTER_COMM_ID, '', True), ctx)     # TODO: Select correct CommID, ip/port
    
    # Setup periodic calling of ROS spin_once in main thread
    log.msg('Add periodic call for Load Balancer Update.')
    LoopingCall(manager.updateLoadInfo).start(settings.LOAD_INFO_UPDATE)
    
    # Start twisted
    log.msg('Initialization completed')
    log.msg('Enter mainloop')
    reactor.run()       # TODO: Call shutdown on manager before reactor is terminated.
    log.msg('Leaving Satellite')

def _get_argparse():
    from argparse import ArgumentParser

    parser = ArgumentParser(prog='Satellite',
                            description='# TODO: Add description')

    parser.add_argument('commID', help='Communication address of this node.')
    parser.add_argument('ip', type=str, help='IP address of the satellite node to which the connection should be established.')
    parser.add_argument('port', type=int, help='Port of the satellite node to which the connection should be established.')
    parser.add_argument('satelliteID', help='Communication address of the node to which a connection should be established.')
    parser.add_argument('key', help='Key which is used to identify this node with the satellite node.')

    return parser

if __name__ == '__main__':
    from twisted.internet import reactor

    parser = _get_argparse()
    args = parser.parse_args()
    
    if not validateAddress(args.commID):
        print 'CommID is not a valid address.'
        exit(1)
     
    if not validateAddress(args.satelliteID):
        print 'SatelliteID is not a valid address.'
        exit(1)
    
    main(reactor, args.ip, args.port, args.commID, args.satelliteID, args.key)
