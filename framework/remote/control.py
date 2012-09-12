#!/usr/bin/env python
# -*- coding: utf-8 -*-
#     
#     control.py
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

# zope specific imports
from zope.interface import implements

# Custom imports
from comm import definition
from comm import types as msgTypes
from comm.message import Message
from core.interfaces import IRequestSender, INodeControl, IParameterControl, \
    IEndpointControl, IRobotControl, IContainerControl
from core.types import cmd as types
from core.command import ConnectionCommand


class RemoteRequestSender(object):
    """ Remote request sender.
    """
    implements(IRequestSender)
    
    def __init__(self, commMngr):
        """ Initialize the remote request sender.
            
            @param commMngr:    CommManager which is used to communicate in
                                this node.
            @type  commMngr:    comm.manager.CommManager
        """
        self._commManager = commMngr
    
    def processRequest(self, request):
        """ Send a request to the master manager.
            
            @param request:     Request which should be sent to the master
                                manager.
            @type  request:     { 'user' : str, 'type' : str, 'args' : tuple }
        """
        msg = Message()
        msg.msgType = msgTypes.REQUEST
        msg.dest = definition.MASTER_ADDR
        msg.content = request
        self._commManager.sendMessage(msg)


class _RemoteControlBase(object):
    """ Base class for all remote controller.
    """
    def __init__(self, userID, commID, commMngr):
        """ Initialize the remote control.
            
            @param userID:      UserID of the owner of the endpoint for which
                                this control is used.
            @type  userID:      str
            
            @param commID:      CommID where the endpoint is localized.
            @type  commID:      str
            
            @param commMngr:    CommManager which is used to communicate in
                                this node.
            @type  commMngr:    comm.manager.CommManager
        """
        self._userID = userID
        self._commID = commID
        self._commManager = commMngr


class RemoteNodeControl(_RemoteControlBase):
    """ Remote control for ROS nodes.
    """
    implements(INodeControl)
    
    def addNode(self, node):
        """ Add a node to the ROS environment.
            
            @param node:    Node description which should be added.
            @type  node:    core.command.NodeCommand
        """
        msg = Message()
        msg.msgType = msgTypes.COMMAND
        msg.dest = self._commID
        msg.content = { 'user' : self._userID, 'cmd' : node }
        self._commManager.sendMessage(msg)
    
    def removeNode(self, tag):
        """ Remove a node from the ROS environment.
            
            @param tag:     Tag which is used to identify the node which should
                            be removed.
            @type  tag:     str
        """
        msg = Message()
        msg.msgType = msgTypes.TAG
        msg.dest = self._commID
        msg.content = { 'user' : self._userID,
                        'type' : types.RM_NODE,
                        'tag'  : tag }
        self._commManager.sendMessage(msg)


class RemoteParameterControl(_RemoteControlBase):
    """ Remote control for ROS parameters.
    """
    implements(IParameterControl)
    
    def addParameter(self, parameter):
        """ Add a parameter to the ROS environment.
            
            @param parameter:   Parameter description which should be added.
            @type  parameter:   core.command._ParameterCommand
        """
        msg = Message()
        msg.msgType = msgTypes.COMMAND
        msg.dest = self._commID
        msg.content = { 'user' : self._userID, 'cmd' : parameter }
        self._commManager.sendMessage(msg)
    
    def removeParameter(self, name):
        """ Remove a parameter from the ROS environment.
            
            @param name:    Name of the parameter which should be removed.
            @type  name:    str
        """
        msg = Message()
        msg.msgType = msgTypes.TAG
        msg.dest = self._commID
        msg.content = { 'user' : self._userID,
                        'type' : types.RM_PARAMETER,
                        'tag'  : name }
        self._commManager.sendMessage(msg)



class RemoteEndpointControl(_RemoteControlBase):
    """ Class which provides a remote control object for an arbitrary endpoint.
    """
    implements(IEndpointControl)
    
    def addInterface(self, interface):
        """ Add an interface to the endpoint.
            
            @param interface:   Interface description which should be added.
            @type  interface:   core.command._EndpointInterfaceCommand
        """
        msg = Message()
        msg.msgType = msgTypes.COMMAND
        msg.dest = self._commID
        msg.content = { 'user' : self._userID, 'cmd' : interface }
        self._commManager.sendMessage(msg)
    
    def removeInterface(self, tag):
        """ Remove an interface from the endpoint.
            
            @param tag:     Tag which is used to identify the interface which
                            should be removed.
            @type  tag:     str
        """
        msg = Message()
        msg.msgType = msgTypes.TAG
        msg.dest = self._commID
        msg.content = { 'user' : self._userID,
                        'type' : types.RM_INTERFACE,
                        'tag'  : tag }
        self._commManager.sendMessage(msg)
    
    def registerConnection(self, tag, commID, remoteTag):
        """ Register a user with an interface to the endpoint.
            
            @param tag:     Tag which is used to identify the interface where
                            the user should be registered.
            @type  tag:     str
            
            @param commID:  Communication ID of the node from where the
                            messages will come or to where they should be sent.
            @type  commID:  str
            
            @param remoteTag:   Identifier which will be used to validate
                                received messages and which will be sent along
                                with outgoing messages.
            @type  remoteTag:    str
        """
        cmd = ConnectionCommand(tag, commID, remoteTag, True)
        
        msg = Message()
        msg.msgType = msgTypes.COMMAND
        msg.dest = self._commID
        msg.content = { 'user' : self._userID, 'cmd' : cmd }
        self._commManager.sendMessage(msg)
    
    def unregisterConnection(self, tag, commID, remoteTag):
        """ Unregister a user from an interface to the endpoint.
            
            @param tag:     Tag which is used to identify the interface which
                            should be removed.
            @type  tag:     str
            
            @param commID:  Communication ID which was used to register the
                            user.
            @type  commID:  str
            
            @param remoteTag:   Identifier which was used to register the
                                target.
            @type  remoteTag:   str
        """
        cmd = ConnectionCommand(tag, commID, remoteTag, False)
        
        msg = Message()
        msg.msgType = msgTypes.COMMAND
        msg.dest = self._commID
        msg.content = { 'user' : self._userID, 'cmd' : cmd }
        self._commManager.sendMessage(msg)


class RemoteRobotControl(RemoteEndpointControl):
    """ Remote control for Robots.
    """
    implements(IRobotControl)
    
    def createRobot(self, robot):
        """ Create a robot.
            
            Should be called only once.
            
            @param robot:   Robot description which should be created.
            @type  robot:   core.command.RobotCommand
        """
        msg = Message()
        msg.msgType = msgTypes.COMMAND
        msg.dest = self._commID
        msg.content = { 'user' : self._userID, 'cmd' : robot }
        self._commManager.sendMessage(msg)
    
    def destroyRobot(self, robotID):
        """ Destroy a robot.
            
            Should be called only once.
            
            @param robotID:     Identifier of the robot which should be
                                destroyed.
            @type  robotID:     str
        """
        msg = Message()
        msg.msgType = msgTypes.TAG
        msg.dest = self._commID
        msg.content = { 'user' : self._userID,
                        'type' : types.RM_ROBOT,
                        'tag'  : robotID }
        self._commManager.sendMessage(msg)

class RemoteContainerControl(_RemoteControlBase):
    """ Remote control for container.
    """
    implements(IContainerControl)
    
    def createContainer(self, container):
        """ Create a container.
            
            Should be called only once.
            
            @param container:   Container description which should be created.
            @type  container:   core.command.ContainerCommand
        """
        msg = Message()
        msg.msgType = msgTypes.COMMAND
        msg.dest = self._commID
        msg.content = { 'user' : self._userID, 'cmd' : container }
        self._commManager.sendMessage(msg)
    
    def destroyContainer(self, cTag):
        """ Destroy a container.
            
            Should be called only once.
            
            @param cTag:    Tag of the container which should be removed.
            @type  cTag:    str
        """
        msg = Message()
        msg.msgType = msgTypes.TAG
        msg.dest = self._commID
        msg.content = { 'user' : self._userID,
                        'type' : types.RM_CONTAINER,
                        'tag'  : cTag }
        self._commManager.sendMessage(msg)
