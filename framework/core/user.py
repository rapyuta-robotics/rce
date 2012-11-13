#!/usr/bin/env python
# -*- coding: utf-8 -*-
#     
#     user.py
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
import json

# twisted specific imports
from twisted.python import log
from twisted.internet.defer import Deferred, DeferredList

# Custom imports
from errors import InvalidRequest, InternalError
from core.proxy import RobotProxy, ContainerProxy, InterfaceProxy


class User(object):
    """ Class which represents a user. A user can have multiple robots
        and multiple containers.
    """
    def __init__(self, ctrlFactory, mngr, userID):
        """ Initialize the User.
            
            @param ctrlFactory:     Control factory which is used in this node.
            @type  ctrlFactory:     core.interfaces.IControlFactory
                            
            @param mngr:            Manager which is used in this node.
            @type  mngr:            core.manager._MasterManager
            
            @param userID:          ID of the represented user.
            @type  userID:          str
        """
        self._controlFactory = ctrlFactory
        self._manager = mngr
        self._userID = userID
        
        self._active = True
        self._environments = {}
        self._robots = {}
        self._interfaces = {}
    
    @property
    def userID(self):
        """ UserID """
        return self._userID
    
    def createRobot(self, robotID, key, commID):
        """ Create a new robot with the given robotID of this user.
            
            @param robotID:     RobotID which is used to identify the robot
                                which should be created.
            @type  robotID:     str
            
            @param key:         Key which should be used to verify connection
                                from represented robot.
            @type  key:         str
            
            @param commID:      CommID of robot manager where the robot should
                                be created.
            @type  commID:      str
        """
        if not self._active:
            return
        
        if not robotID:
            raise InvalidRequest('Robot ID can not be an empty string.')
        
        if robotID in self._robots:
            raise InvalidRequest('A robot with the same robotID '
                                 'has already been registered.')
        
        control = self._controlFactory.createRobotControl(self._userID, commID)
        self._robots[robotID] = RobotProxy(self, robotID, commID, key, control)
    
    def destroyRobot(self, robotID):
        """ Destroy the robot with the given robotID of this user.
            
            @param robotID:     RobotID which is used to identify the robot
                                which should be destroyed.
            @type  robotID:     str
        """
        try:
            robot = self._robots.pop(robotID)
        except KeyError:
            raise InvalidRequest('The robot has never been registered.')
        
        robot.delete()
    
    def createContainer(self, resp, cTag):
        """ Create a container with the given container tag of this user.
            
            @param resp:    Deferred whose callbacks can be used to give a
                            response to the request.
            @type  resp:    twisted::Deferred
            
            @param cTag:    Tag which will be used to identify the container
                            which should be created.
            @type  cTag:    str
        """
        # TODO: What to do with the response
        if not self._active:
            return
        
        if not cTag:
            raise InvalidRequest('Container tag can not be an empty string.')
        
        if cTag in self._environments:
            raise InvalidRequest('There exists already a container '
                                 'with the same tag.')
        
        commID = self._manager.getCommID()
        ctrlID = self._manager.getNextContainerLocation()
        control = self._controlFactory.createContainerControl(self._userID,
                                                              commID, ctrlID)
        self._environments[cTag] = ContainerProxy(self, cTag, commID, control)
    
    def destroyContainer(self, resp, cTag):
        """ Destroy the container with the given container tag of this user.
            
            @param resp:    Deferred whose callbacks can be used to give a
                            response to the request.
            @type  resp:    twisted::Deferred
            
            @param cTag:    Tag which is used to identify the container
                            which should be destroyed.
            @type  cTag:    str
        """
        # TODO: What to do with the response
        try:
            container = self._environments.pop(cTag)
        except KeyError:
            raise InvalidRequest('There exists no container '
                                 'with the given tag.')
        
        container.delete(resp)
        self._manager.delCommID(container.commID)
    
    def addNode(self, resp, cTag, nTag, pkg, exe, args, name, namespace):
        """ Add a node to the ROS environment matching the given parameters.
            
            @param resp:        Deferred whose callbacks can be used to give a
                                response to the request.
            @type  resp:        twisted::Deferred
            
            @param cTag:        Tag which is used to identify the ROS
                                environment in which the node should be added.
            @type  cTag:        str
            
            @param nTag:        Tag which is used to identify the ROS node
                                which should added.
            @type  nTag:        str

            @param pkg:         Package name where the node can be found.
            @type  pkg:         str

            @param exe:         Name of executable which should be launched.
            @type  exe:         str
            
            @param args:        Arguments which should be used for the launch.
            @type  args:        str
            
            @param name:        Name of the node under which it should be
                                launched.
            @type  name:        str
            
            @param namespace:   Namespace in which the node should be started
                                in the environment.
            @type  namespace:   str
            
            @raise:             errors.InvalidRequest if the nodeName is not a
                                valid ROS name.
                                errors.InvalidRequest if the container tag is
                                not valid.
        """
        if not self._active:
            return
        
        if not nTag:
            raise InvalidRequest('Node tag can not be an empty string.')
        
        try:
            self._environments[cTag].addNode(nTag, pkg, exe, args, name,
                                             namespace, resp)
        except KeyError:
            raise InvalidRequest('ROS environment tag is invalid.')
    
    def removeNode(self, resp, cTag, nTag):
        """ Remove a node from the ROS environment matching the given tag.
            
            @param resp:    Deferred whose callbacks can be used to give a
                            response to the request.
            @type  resp:    twisted::Deferred
            
            @param cTag:    Tag which is used to identify the ROS
                            environment from which the node should be removed.
            @type  cTag:    str
            
            @param nTag:    Tag which is used to identify the ROS node which
                            should removed.
            @type  nTag:    str
            
            @raise:         errors.InvalidRequest if the container tag is not
                            valid.
        """
        if not self._active:
            return
        
        try:
            self._environments[cTag].removeNode(nTag, resp)
        except KeyError:
            raise InvalidRequest('ROS environment tag is invalid.')
    
    def addParameter(self, resp, cTag, name, value, paramType):
        """ Add a parameter to the parameter server in the ROS environment
            matching the given tag.
            
            @param resp:        Deferred whose callbacks can be used to give a
                                response to the request.
            @type  resp:        twisted::Deferred
            
            @param cTag:        Tag which is used to identify the ROS
                                environment in which the parameter should be
                                added.
            @type  cTag:        str
            
            @param name:        Name of the parameter which should be added.
            @type  name:        str
            
            @param value:       Value of the parameter which should be added as
                                a serialized JSON string.
            @type  value:       str
            
            @param paramType:   Type of the parameter to add. Valid options
                                are:
                                    int, str, float, bool, file
            @type  paramType:   str
            
            @raise:             errors.InvalidRequest if the name is not a
                                valid ROS name.
                                errors.InvalidRequest if the container tag is
                                not valid.
        """
        if not self._active:
            return
        
        if not name:
            raise InvalidRequest('Parameter name can not be an empty string.')
        
        try:
            self._environments[cTag].addParameter(name, json.loads(value),
                                                  paramType, resp)
        except KeyError:
            raise InvalidRequest('ROS environment tag is invalid.')
    
    def removeParameter(self, resp, cTag, name):
        """ Remove a parameter from the parameter server in the ROS environment
            matching the given tag.
            
            @param resp:    Deferred whose callbacks can be used to give a
                            response to the request.
            @type  resp:    twisted::Deferred
            
            @param cTag:    Tag which is used to identify the ROS
                            environment from which the parameter should be
                            removed.
            @type  cTag:    str
            
            @param name:    Name of the parameter which should be removed.
            @type  name:    str
            
            @raise:         errors.InvalidRequest if the container tag is not
                            valid.
        """
        if not self._active:
            return
        
        try:
            self._environments[cTag].removeParameter(name, resp)
        except KeyError:
            raise InvalidRequest('ROS environment tag is invalid.')
    
    def addInterface(self, _, iType, destTag, iTag, className, addr=None):
        """ Add an interface for this user.
            
            @param iType:       Type of the interface. Valid types are:
                                    serviceInterface, serviceProviderInterface,
                                    publisherInterface, subscriberInterface,
                                    serviceConverter, serviceProviderConverter,
                                    publisherConverter, subscriberConverter
            @type  iType:       str
            
            @param destTag:     Tag which is used to identify the endpoint
                                where the interface should be added; either
                                containerTag or robotID.
            @type  destTag:     str
                            
            @param iTag:        Tag which is used to identify the new
                                interface.
            @type  iTag:        str
            
            @param className:   Message type/Service type consisting of the
                                package and the name of the message/service,
                                i.e. 'std_msgs/Int32'.
            @type  className:   str
            
            @param addr:        ROS name/address which the interface should
                                use. Only necessary if it is of subtype
                                xxxInterface.
            @type  addr:        str
            
            @raise:             errors.InvalidRequest if the destTag is not
                                valid.
        """
        if not self._active:
            return
        
        if not iTag:
            raise InvalidRequest('Interface tag can not be an empty string.')
        
        if iTag in self._interfaces:
            raise InvalidRequest('There exists already an interface '
                                 'with the same tag.')
        
        if iType.endswith('Converter') or iType.endswith('Forwarder'):
            try:
                endpoint = self._robots[destTag]
            except KeyError:
                raise InvalidRequest('RobotID is invalid.')
            
            msg = 'Add interface "{0}" (type: "{1}") to robot "{3}".'
            
            # Set to None to make sure that core.interface.Interface does not
            # interpret the Interface as a ROS interface.
            addr = None
        elif iType.endswith('Interface'):
            try:
                endpoint = self._environments[destTag]
            except KeyError:
                raise InvalidRequest('Container tag is invalid.')
            
            msg = ('Add interface "{0}" (type: "{1}"; address: "{2}") '
                   'to container "{3}".')
        else:
            raise InvalidRequest('Interface type "{0}" is '
                                 'invalid.'.format(iType))
        
        log.msg(msg.format(iTag, iType, addr, destTag))
        InterfaceProxy(endpoint, iTag, iType, className, addr)
    
    def removeInterface(self, resp, iTag):
        """ Remove an interface for this user.
            
            @param resp:    Deferred whose callbacks can be used to give a
                            response to the request.
            @type  resp:    twisted::Deferred
            
            @param iTag:    Tag which is used to identify the interface to
                            remove.
            @type  iTag:    str
            
            @raise:         errors.InvalidRequest if the interface tag is not
                            valid.
        """
        if not self._active:
            return
        
        try:
            self._interfaces[iTag].delete(resp)
        except KeyError:
            raise InvalidRequest('Interface tag is invalid.')
    
    def registerInterface(self, interface):
        """ Register a new interface with this user.
                 (Callback from core.proxy.InterfaceProxy)
            
            @param interface:   New interface instance which should be
                                registered.
            @type  interface:   Interface
        """
        tag = interface.tag
        
        if tag in self._interfaces:
            raise InternalError('There exists already an interface '
                                'with the same tag.')
        
        self._interfaces[tag] = interface
    
    def unregisterInterface(self, interface):
        """ Unregister an interface with this user.
                 (Callback from core.proxy.InterfaceProxy)
            
            @param interface:   Interface instance which should be
                                unregistered.
            @type  interface:   Interface
        """
        tag = interface.tag
        
        if tag not in self._interfaces:
            raise InternalError('Tried to unregister a non existing '
                                'interface.')
        
        del self._interfaces[tag]
    
    def createConnection(self, resp, iTag1, iTag2):
        """ Create a connection between two interfaces.
            
            @param resp:    Deferred whose callbacks can be used to give a
                            response to the request.
            @type  resp:    twisted::Deferred
            
            @param iTagX:   The two tags of the two interfaces which should
                            be connected.
            @type  iTagX:   str
        """
        if not self._active:
            return
        
        try:
            interface1 = self._interfaces[iTag1]
            interface2 = self._interfaces[iTag2]
        except KeyError:
            raise InvalidRequest('Interface tag is invalid.')
        
        if not interface1.validConnection(interface2):
            raise InvalidRequest('Can not connect the two interfaces.')
        
        resp1 = Deferred()
        resp2 = Deferred()
        resp.chainDeferred(DeferredList((resp1, resp2)))
        
        interface1.registerConnection(interface2, resp1)
        interface2.registerConnection(interface1, resp2)
        
        log.msg('Create connection from "({0}, {1})" to "({2}, {3})".'.format(
                    interface1.commID, interface1.tag,
                    interface2.commID, interface2.tag))
    
    def destroyConnection(self, resp, iTag1, iTag2):
        """ Destroy a connection between two interfaces.
            
            @param resp:    Deferred whose callbacks can be used to give a
                            response to the request.
            @type  resp:    twisted::Deferred
            
            @param iTagX:   The two tags of the two interfaces which should
                            be disconnected.
            @type  iTagX:   str
        """
        if not self._active:
            return
        
        try:
            interface1 = self._interfaces[iTag1]
            interface2 = self._interfaces[iTag2]
        except KeyError:
            raise InvalidRequest('Interface tag is invalid.')
        
        resp1 = Deferred()
        resp2 = Deferred()
        resp.chainDeferred(DeferredList((resp1, resp2)))
        
        interface1.unregisterConnection(interface2, resp1)
        interface2.unregisterConnection(interface1, resp2)
        
        log.msg('Destroy connection from "({0}, {1})" to "({2}, {3})".'.format(
                    interface1.commID, interface1.tag,
                    interface2.commID, interface2.tag))
    
    def processStatusUpdate(self, cTag, nTag, status):
        """ 
        """
        container = self._environments.get(cTag, None)
        
        if not container:
            return
        
        if nTag:
            container.setNodeStatus(nTag, status)
        else:
            container.setContainerStatus(status)
    
    def delete(self):
        """ Removes the user.
            
            Make sure to call this method, because this method makes sure that
            all necessary references are removed.
            
            Once this method is called this user can no longer be used.
        """
        self._active = False
        
        cTags = self._environments.keys()
        rIDs = self._robots.keys()
        
        for cTag in cTags:
            self.destroyContainer(cTag)
        
        for rID in rIDs:
            self.destroyRobot(rID)
