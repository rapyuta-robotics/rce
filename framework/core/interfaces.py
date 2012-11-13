#!/usr/bin/env python
# -*- coding: utf-8 -*-
#     
#     interfaces.py
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
from zope.interface import Attribute, Interface


class IUIDServer(Interface):
    """ Interface which declares the necessary method which all UID server
        implementations    have to implement.
    """
    def getUID(): #@NoSelf
        """ Return a unique ID which can be used as a communication ID.
                        
            @return:    Valid string which can be used to connect to the
                        server.
        """


class ISerializable(Interface):
    """ Interface which declares the necessary methods which all serializable
        objects have to implement.
    """
    IDENTIFIER = Attribute("""
    Identifier which is used to select the appropriate object class.
    """)
    
    def serialize(serializer): #@NoSelf
        """ This method is called whenever the object should be serialized.
            
            @param serializer:  Serializer which provides methods to serialize
                                base types and into which the message has to be
                                serialized.
            @type  serializer:  comm.serializer.Serializer
            
            @raise:     errors.SerializationError
        """
    
    def deserialize(serializer): #@NoSelf
        """ This method is called whenever the object should be deserialized.
            
            This method has to be implemented as a classmethod and should,
            therefore, return a new class instance!
            
            @param serializer:  Serializer which provides methods to
                                deserialize base types and from which the
                                message has to be deserialized.
            @type  serializer:  comm.serializer.Serializer
            
            @return:    New class instance.
            
            @raise:     errors.SerializationError
        """


class IEndpointProxy(Interface):
    """ Interface which declares the necessary methods which all endpoint
        proxies have to implement.
    """
    owner = Attribute("""
    Owner (type: core.uer.User) of this endpoint.
    """)
    
    uid = Attribute("""
    Identifier of this endpoint.
    """)
    
    commID = Attribute("""
    Communication ID of this endpoint.
    """)
    
    def registerInterface(interface): #@NoSelf
        """ Register a new interface with this endpoint.
            
            @param interface:   Interface instance which should be registered.
            @type  interface:   Interface
        """
    
    def unregisterInterface(interface): #@NoSelf
        """ Unregister an interface with this endpoint.
            
            @param interface:   Interface instance which should be
                                unregistered.
            @type  interface:   Interface
        """
    
    def delete(): #@NoSelf
        """ Removes the endpoint.
            
            Make sure to call this method, because this method makes sure that
            all necessary references are removed.
            
            Once this method is called this monitor can no longer be used.
        """


class IEndpointInterfaceMonitor(Interface):
    """ Interface which declares all necessary methods which all endpoint
        interface monitors have to implement.
    """
    IDENTIFIER = Attribute("""
    Identifier which is used to select the appropriate object class.
    """)
    
    def start(): #@NoSelf
        """ This method is used to setup the interface.
        """
    
    def stop(): #@NoSelf
        """ This method is used to stop the interface.
        """
    
    def addConnection(commID, tag): #@NoSelf
        """ Add a connection to another interface to this interface.
            
            @param commID:  Communication ID of other interface.
            @type  commID:  str
            
            @param tag:     Name of other interface.
            @type  tag:     str
        """
    
    def removeConnection(commID, tag): #@NoSelf
        """ Remove a connection to another interface from this interface.
            
            @param commID:  Communication ID of other interface.
            @type  commID:  str
            
            @param tag:     Name of other interface.
            @type  tag:     str
        """


class IRequestSender(Interface):
    """ Interface which declares the necessary method which the request sender
        has to implement.
    """
    def processRequest(user, reqType, args, resp): #@NoSelf
        """ Send a request to the master manager.
            
            @param user:        ID of user who is responsible for request.
            @type  user:        str
            
            @param reqType:     Identifier of this request. For a list of
                                options hava a look at module core.types.req.
            @type  reqType:     str
            
            @param args:        Arguments which should be passed to the request
                                handler in form of a tuple of strings.
            @type  args:        (str)
        """


class IControlFactory(Interface):
    """ Interface which declares the necessary methods which the control
        factories have to implement.
    """
    def createRobotControl(userID, commID): #@NoSelf
        """ Create a new Robot control instance.
            
            @return:    New Robot control instance.
            @rtype:     core.interfaces.IRobotControl
        """
    
    def createContainerControl(userID, commID, ctrlID): #@NoSelf
        """ Create a new Container control instance.
            
            @return:    New Container control instance.
            @rtype:     core.interfaces.IContainerControl
        """


class INodeControl(Interface):
    """ Interface which declares the necessary methods which are required to
        control ROS nodes.
    """
    def addNode(node, resp): #@NoSelf
        """ Add a node to the ROS environment.
            
            @param node:    Node description which should be added.
            @type  node:    core.command.NodeCommand
            
            @param resp:    Deferred whose callbacks can be used to give a
                            response to the request.
            @type  resp:    twisted::Deferred
        """
    
    def removeNode(tag, resp): #@NoSelf
        """ Remove a node from the ROS environment.
            
            @param tag:     Tag which is used to identify the node which
                            should be removed.
            @type  tag:     str
            
            @param resp:    Deferred whose callbacks can be used to give a
                            response to the request.
            @type  resp:    twisted::Deferred
        """


class IParameterControl(Interface):
    """ Interface which declares the necessary methods which are required to
        control ROS parameters.
    """
    def addParameter(parameter, resp): #@NoSelf
        """ Add a parameter to the ROS environment.
            
            @param parameter:   Parameter description which should be added.
            @type  parameter:   core.command._ParameterCommand
            
            @param resp:        Deferred whose callbacks can be used to give a
                                response to the request.
            @type  resp:        twisted::Deferred
        """
    
    def removeParameter(name, resp): #@NoSelf
        """ Remove a parameter from the ROS environment.
            
            @param name:    Name of the parameter which should be removed.
            @type  name:    str
            
            @param resp:    Deferred whose callbacks can be used to give a
                            response to the request.
            @type  resp:    twisted::Deferred
        """


class IEndpointControl(Interface):
    """ Interface which declares the necessary methods which are required to
        control interfaces of an endpoint.
    """
    def addInterface(interface, resp): #@NoSelf
        """ Add an interface to the ROS environment.
            
            @param interface:   Interface description which should be added.
            @type  interface:   core.command._EndpointInterfaceCommand
            
            @param resp:        Deferred whose callbacks can be used to give a
                                response to the request.
            @type  resp:        twisted::Deferred
        """
    
    def removeInterface(tag, resp): #@NoSelf
        """ Remove an interface from the ROS environment.
            
            @param tag:     Tag which is used to identify the interface which
                            should be removed.
            @type  tag:     str
            
            @param resp:    Deferred whose callbacks can be used to give a
                            response to the request.
            @type  resp:    twisted::Deferred
        """
    
    def registerConnection(tag, commID, connID, resp): #@NoSelf
        """ Register a user with an interface to the ROS environment.
            
            @param tag:     Tag which is used to identify the interface where
                            the connection should be registered.
            @type  tag:     str
            
            @param commID:  Communication ID of the node from where the
                            connection originates.
            @type  commID:  str
            
            @param connID:  Identifier of the interface from where the
                            connections originates.
            @type  connID:  str
            
            @param resp:    Deferred whose callbacks can be used to give a
                            response to the request.
            @type  resp:    twisted::Deferred
        """
    
    def unregisterConnection(tag, commID, connID, resp): #@NoSelf
        """ Unregister a user from an interface to the ROS environment.
            
            @param tag:     Tag which is used to identify the interface where
                            the connection should be unregistered.
            @type  tag:     str
            
            @param commID:  Communication ID of the node from where the
                            connection originates.
            @type  commID:  str
            
            @param connID:  Identifier of the interface from where the
                            connections originates.
            @type  connID:  str
            
            @param resp:    Deferred whose callbacks can be used to give a
                            response to the request.
            @type  resp:    twisted::Deferred
        """


class IRobotControl(IEndpointControl):
    """ Interface which declares the necessary methods which are required to
        control robots.
    """
    def createRobot(robot, resp): #@NoSelf
        """ Create a robot.
            
            @param robot:       Robot description which should be created.
            @type  robot:       core.command.RobotCommand
            
            @param resp:        Deferred whose callbacks can be used to give a
                                response to the request.
            @type  resp:        twisted::Deferred
        """
    
    def destroyRobot(robotID, resp): #@NoSelf
        """ Destroy a robot.
            
            @param robotID:     ID of the robot which should be destroyed.
            @type  robotID:     str
            
            @param resp:        Deferred whose callbacks can be used to give a
                                response to the request.
            @type  resp:        twisted::Deferred
        """


class IContainerControl(Interface):
    """ Interface which declares the necessary methods which are required to
        control containers.
    """
    def createContainer(container, resp): #@NoSelf
        """ Create a container.
            
            @param container:   Container description which should be created.
            @type  container:   core.command.ContainerCommand
            
            @param resp:        Deferred whose callbacks can be used to give a
                                response to the request.
            @type  resp:        twisted::Deferred
        """
    
    def destroyContainer(tag, resp): #@NoSelf
        """ Destroy a container.
            
            @param tag:     Tag of the container which should be destroyed.
            @type  tag:     str
            
            @param resp:    Deferred whose callbacks can be used to give a
                            response to the request.
            @type  resp:    twisted::Deferred
        """


class IEndpointConverterCommand(Interface):
    """ Marker interface to identify an endpoint converter command.
    """


class IEndpointInterfaceCommand(Interface):
    """ Marker interface to identify an endpoint interface command.
    """


class IDistributor(Interface):
    """ Interface which declares the necessary methods for a Distributor.
    """
    def addHandler(identifier, handler): #@NoSelf
        """ Add a handler for a type.
            
            If there is already a handler registered for the same identifier,
            then the old handler is removed and only the new handler is kept.
            
            @param identifier:  Identifier which is used to match the type
                                to the handler.
            @type  identifier:  str
            
            @param handler:     Callable which is called when matching type is
                                encountered.
            @type  handler:     Callable
        """
    
    def removeHandler(self, identifier): #@NoSelf
        """ Remove a handler for a type.
            
            @param identifier:  Identifier which is used to match the type to
                                the handler which should be removed.
            @type  identifier:  str
        """


class IMessenger(Interface):
    """ Interface which declares the necessary methods for ROS message
        messenger.
    """
    def send(userID, tag, commID, senderTag, msg, msgID): #@NoSelf
        """ Send a message which was received from an endpoint to another
            interface.
            
            @param userID:      User ID of the interface owner.
            @type  userID:      str
            
            @param tag:         Tag of the interface at the destination.
            @type  tag:         str
            
            @param commID:      Communication ID of the destination.
            @type  commID:      str
            
            @param senderTag:   Tag of the interface who is sending the
                                message.
            @type  senderTag:   str
            
            @param msg:         ROS message in serialized form which should be
                                sent.
            @type  msg:         str
            
            @param msgID:       Identifier which is used to match a request to
                                a response.
            @type  msgID:       str
        """


class ILoadBalancer(Interface):
    """ Interface which declares the necessary methods for the Load Balancer.
    """
    def registerMachine(machine): #@NoSelf
        """ Register a new machine in which containers can be started.
            
            @param machine:     Machine which should be registered.
            @type  machine:     core.balancer.Machine
        """
    
    def unregisterMachine(ip): #@NoSelf
        """ Unregister a machine.
            
            @param ip:      IP address of the machine which should be
                            unregistered.
            @type  ip:      str
        """
    
    def getNextRobotLocation(): #@NoSelf
        """ Returns the CommID and IP address of the robot manager where the
            next robot should be created.
        """
    
    def getNextContainerLocation(): #@NoSelf
        """ Returns the CommID of the container manager where the next
            container should be created.
        """
