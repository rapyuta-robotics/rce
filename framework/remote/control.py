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
    
    def processRequest(self, user, reqType, args, resp):
        """ Send a request to the master manager.
            
            @param user:        ID of user who is responsible for request.
            @type  user:        str
            
            @param reqType:     Identifier of this request. For a list of
                                options hava a look at module core.types.req.
            @type  reqType:     str
            
            @param args:        Arguments which should be passed to the request
                                handler in form of a tuple of strings.
            @type  args:        (str)
            
            @param resp:        Deferred whose callbacks are used to give a
                                response to the request.
            @type  resp:        twisted::Deferred
        """
        msg = Message()
        msg.msgType = msgTypes.REQUEST
        msg.dest = definition.MASTER_ADDR
        msg.content = { 'user' : user, 'type' : reqType, 'args' : args }
        self._commManager.sendMessage(msg, resp)


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
    
    def addNode(self, node, resp):
        """ Add a node to the ROS environment.
            
            @param node:    Node description which should be added.
            @type  node:    core.command.NodeCommand
            
            @param resp:    Deferred whose callbacks can be used to give a
                            response to the request.
            @type  resp:    twisted::Deferred
        """
        msg = Message()
        msg.msgType = msgTypes.COMMAND
        msg.dest = self._commID
        msg.content = { 'user' : self._userID, 'cmd' : node }
        self._commManager.sendMessage(msg, resp)
    
    def removeNode(self, tag, resp):
        """ Remove a node from the ROS environment.
            
            @param tag:     Tag which is used to identify the node which should
                            be removed.
            @type  tag:     str
            
            @param resp:    Deferred whose callbacks can be used to give a
                            response to the request.
            @type  resp:    twisted::Deferred
        """
        msg = Message()
        msg.msgType = msgTypes.TAG
        msg.dest = self._commID
        msg.content = { 'user' : self._userID,
                        'type' : types.RM_NODE,
                        'tag'  : tag }
        self._commManager.sendMessage(msg, resp)


class RemoteParameterControl(_RemoteControlBase):
    """ Remote control for ROS parameters.
    """
    implements(IParameterControl)
    
    def addParameter(self, parameter, resp):
        """ Add a parameter to the ROS environment.
            
            @param parameter:   Parameter description which should be added.
            @type  parameter:   core.command._ParameterCommand
            
            @param resp:    Deferred whose callbacks can be used to give a
                            response to the request.
            @type  resp:    twisted::Deferred
        """
        msg = Message()
        msg.msgType = msgTypes.COMMAND
        msg.dest = self._commID
        msg.content = { 'user' : self._userID, 'cmd' : parameter }
        self._commManager.sendMessage(msg, resp)
    
    def removeParameter(self, name, resp):
        """ Remove a parameter from the ROS environment.
            
            @param name:    Name of the parameter which should be removed.
            @type  name:    str
            
            @param resp:    Deferred whose callbacks can be used to give a
                            response to the request.
            @type  resp:    twisted::Deferred
        """
        msg = Message()
        msg.msgType = msgTypes.TAG
        msg.dest = self._commID
        msg.content = { 'user' : self._userID,
                        'type' : types.RM_PARAMETER,
                        'tag'  : name }
        self._commManager.sendMessage(msg, resp)



class RemoteEndpointControl(_RemoteControlBase):
    """ Class which provides a remote control object for an arbitrary endpoint.
    """
    implements(IEndpointControl)
    
    def addInterface(self, interface, resp):
        """ Add an interface to the endpoint.
            
            @param interface:   Interface description which should be added.
            @type  interface:   core.command._EndpointInterfaceCommand
            
            @param resp:    Deferred whose callbacks can be used to give a
                            response to the request.
            @type  resp:    twisted::Deferred
        """
        msg = Message()
        msg.msgType = msgTypes.COMMAND
        msg.dest = self._commID
        msg.content = { 'user' : self._userID, 'cmd' : interface }
        self._commManager.sendMessage(msg, resp)
    
    def removeInterface(self, tag, resp):
        """ Remove an interface from the endpoint.
            
            @param tag:     Tag which is used to identify the interface which
                            should be removed.
            @type  tag:     str
            
            @param resp:    Deferred whose callbacks can be used to give a
                            response to the request.
            @type  resp:    twisted::Deferred
        """
        msg = Message()
        msg.msgType = msgTypes.TAG
        msg.dest = self._commID
        msg.content = { 'user' : self._userID,
                        'type' : types.RM_INTERFACE,
                        'tag'  : tag }
        self._commManager.sendMessage(msg, resp)
    
    def registerConnection(self, tag, commID, remoteTag, resp):
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
            @type  remoteTag:   str
            
            @param resp:    Deferred whose callbacks can be used to give a
                            response to the request.
            @type  resp:    twisted::Deferred
        """
        cmd = ConnectionCommand(tag, commID, remoteTag, True)
        
        msg = Message()
        msg.msgType = msgTypes.COMMAND
        msg.dest = self._commID
        msg.content = { 'user' : self._userID, 'cmd' : cmd }
        self._commManager.sendMessage(msg, resp)
    
    def unregisterConnection(self, tag, commID, remoteTag, resp):
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
            
            @param resp:    Deferred whose callbacks can be used to give a
                            response to the request.
            @type  resp:    twisted::Deferred
        """
        cmd = ConnectionCommand(tag, commID, remoteTag, False)
        
        msg = Message()
        msg.msgType = msgTypes.COMMAND
        msg.dest = self._commID
        msg.content = { 'user' : self._userID, 'cmd' : cmd }
        self._commManager.sendMessage(msg, resp)


class RemoteRobotControl(RemoteEndpointControl):
    """ Remote control for Robots.
    """
    implements(IRobotControl)
    
    def createRobot(self, robot, resp):
        """ Create a robot.
            
            Should be called only once.
            
            @param robot:   Robot description which should be created.
            @type  robot:   core.command.RobotCommand
            
            @param resp:    Deferred whose callbacks can be used to give a
                            response to the request.
            @type  resp:    twisted::Deferred
        """
        msg = Message()
        msg.msgType = msgTypes.COMMAND
        msg.dest = self._commID
        msg.content = { 'user' : self._userID, 'cmd' : robot }
        self._commManager.sendMessage(msg)
    
    def destroyRobot(self, robotID, resp):
        """ Destroy a robot.
            
            Should be called only once.
            
            @param robotID:     Identifier of the robot which should be
                                destroyed.
            @type  robotID:     str
            
            @param resp:    Deferred whose callbacks can be used to give a
                            response to the request.
            @type  resp:    twisted::Deferred
        """
        msg = Message()
        msg.msgType = msgTypes.TAG
        msg.dest = self._commID
        msg.content = { 'user' : self._userID,
                        'type' : types.RM_ROBOT,
                        'tag'  : robotID }
        self._commManager.sendMessage(msg, resp)


class RemoteContainerControl(_RemoteControlBase):
    """ Remote control for container.
    """
    implements(IContainerControl)
    
    def createContainer(self, container, resp):
        """ Create a container.
            
            Should be called only once.
            
            @param container:   Container description which should be created.
            @type  container:   core.command.ContainerCommand
            
            @param resp:    Deferred whose callbacks can be used to give a
                            response to the request.
            @type  resp:    twisted::Deferred
        """
        msg = Message()
        msg.msgType = msgTypes.COMMAND
        msg.dest = self._commID
        msg.content = { 'user' : self._userID, 'cmd' : container }
        self._commManager.sendMessage(msg, resp)
    
    def destroyContainer(self, cTag, resp):
        """ Destroy a container.
            
            Should be called only once.
            
            @param cTag:    Tag of the container which should be removed.
            @type  cTag:    str
            
            @param resp:    Deferred whose callbacks can be used to give a
                            response to the request.
            @type  resp:    twisted::Deferred
        """
        msg = Message()
        msg.msgType = msgTypes.TAG
        msg.dest = self._commID
        msg.content = { 'user' : self._userID,
                        'type' : types.RM_CONTAINER,
                        'tag'  : cTag }
        self._commManager.sendMessage(msg, resp)
