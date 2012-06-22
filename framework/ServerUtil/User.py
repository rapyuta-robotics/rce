#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       User.py
#       
#       This file is part of the RoboEarth Cloud Engine framework.
#       
#       This file was originally created for RoboEearth
#       http://www.roboearth.org/
#       
#       The research leading to these results has received funding from
#       the European Union Seventh Framework Programme FP7/2007-2013 under
#       grant agreement no248942 RoboEarth.
#       
#       Copyright 2012 RoboEarth
#       
#       Licensed under the Apache License, Version 2.0 (the "License");
#       you may not use this file except in compliance with the License.
#       You may obtain a copy of the License at
#       
#       http://www.apache.org/licenses/LICENSE-2.0
#       
#       Unless required by applicable law or agreed to in writing, software
#       distributed under the License is distributed on an "AS IS" BASIS,
#       WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#       See the License for the specific language governing permissions and
#       limitations under the License.
#       
#       \author/s: Dominique Hunziker 
#       
#       

# Custom imports
from Exceptions import InvalidRequest, InternalError
from Container import Container

class User(object):
    """ Class which represents a user. A user can have multiple robots
        and multiple containers.
    """
    def __init__(self, commMngr, serverMngr, userID):
        """ Initialize the User.
            
            @param commMngr:    CommManager which is used in this node.
            @type  commMngr:    CommManager
            
            @param serverMngr:  ServerManager which is used in this node.
            @type  serverMngr:  ServerManager
            
            @param userID:      ID of the represented user.
            @type  userID:      str
        """
        self._commManager = commMngr
        self._serverManager = serverMngr
        self._userID = userID
        
        self._reservedTags = set()
        self._containers = {}
        self._robots = {}
    
    @property
    def userID(self):
        """ UserID """
        return self._userID
    
    def registerRobot(self, robot):
        """ Register a new robot with this user.
            
            @param robot:   Robot which should be registered.
            @type  robot:   Robot
        """
        robotID = robot.robotID
        
        if robotID in self._robots:
            raise InternalError('The same robot already has been registered.')
        
        self._robots[robotID] = robot
    
    def unregisterRobot(self, robot):
        """ Unregister a robot from this user.
            
            @param robot:   Robot which should be unregistered.
            @type  robot:   Robot
        """
        robotID = robot.robotID
        
        if robotID not in self._robots:
            raise InternalError('The robot has never been registered.')
        
        del self._robots[robotID]
    
    def createContainer(self, containerTag):
        """ Create a container with the given containerTag.
            
            @param containerTag:    Tag which will be used to identify the container
                                    which should be created.
            @type  containerTag:    str
        """
        if containerTag in self._containers:
            raise InvalidRequest('There exists already a container with the same tag.')
        
        if containerTag in self._reservedTags:
            raise InvalidRequest('There is already a pending request with the same tag.')
        
        self._reservedTags.add(containerTag)
        
        def _createContainer(commID):
            container = Container( self._commManager,
                                   self._serverManager,
                                   self,
                                   containerTag,
                                   commID )
            
            # Send request to start the container
            container.start()
            
            # Register container in this robot instance
            self._containers[containerTag] = container
            self._reservedTags.remove(containerTag)
            
            # Register container in the manager
            self._serverManager.registerContainer(container)
        
        deferred = self._serverManager.getNewCommID()
        deferred.addCallback(_createContainer)
    
    def destroyContainer(self, containerTag):
        """ Destroy a container matching the given containerTag.
            
            @param containerTag:    Tag which is used to identify the container
                                    which should be destroyed.
            @type  containerTag:    str
        """
        container = self._containers.pop(containerTag)
        container.stop()
        self._serverManager.unregisterContainer(container)
    
    def addNode(self, containerTag, nodeTag, package, executable, namespace):
        """ Add a node to the ROS environment in the container matching the
            given container tag.
            
            @param containerTag:    Container tag which is used to identify the
                                    container in which the node should be added.
            @type  containerTag:    str
            
            @param nodeTag:     Tag which is used to identify the ROS node which
                                should added.
            @type  nodeTag:     str

            @param package:     Package name where the node can be found.
            @type  package:     str

            @param executable:  Name of executable which should be launched.
            @type  executable:  str
            
            @param namespace:   Namesapce in which the node should be started
                                in the environment.
            @type  namespace:   str
            
            @raise:     InvalidRequest if the nodeName is not a valid ROS name.
                        InvalidRequest if the container tag is not valid.
        """
        try:
            self._containers[containerTag].addNode(nodeTag, package, executable, namespace)
        except KeyError:
            raise InvalidRequest('Container tag is invalid.')
    
    def removeNode(self, containerTag, nodeTag):
        """ Remove a node from the ROS environment in the container matching the
            given container tag.
            
            @param containerTag:    Container tag which is used to identify the
                                    container from which the node should be removed.
            @type  containerTag:    str
            
            @param nodeTag:     Tag which is used to identify the ROS node which
                                should removed.
            @type  nodeTag:     str
            
            @raise:     InvalidRequest if the container tag is not valid.
        """
        try:
            self._containers[containerTag].removeNode(nodeTag)
        except KeyError:
            raise InvalidRequest('Container tag is invalid.')
    
    def addInterface(self, containerTag, interfaceTag, address, interfaceType, className):
        """ Add an interface to the container matching the given container tag.
            
            @param containerTag:    Container tag which is used to identify the
                                    container in which the interface should be added.
            @type  containerTag:    str
            
            @param interfaceTag:    Tag which is used to identify the interface to add.
            @type  interfaceTag:    str
            
            @param address:     ROS name/address which the interface should use.
            @type  address:     str
            
            @param interfaceType:   Type of the interface. Valid types are 'service', 'publisher',
                                    and 'subscriber'.
            @type  interfaceType:   str
            
            @param className:   Message type/Service type consisting of the package and the name
                                of the message/service, i.e. 'std_msgs/Int32'.
            @type  className:   str
            
            @raise:     InvalidRequest if the container tag is not valid.
        """
        try:
            self._containers[containerTag].addInterface(interfaceTag, address, className, interfaceType)
        except KeyError:
            raise InvalidRequest('Container tag is invalid.')
    
    def removeInterface(self, containerTag, interfaceTag):
        """ Remove an interface to the container matching the given container tag.
            
            @param containerTag:    Container tag which is used to identify the
                                    container from which the interface should be removed.
            @type  containerTag:    str
            
            @param interfaceTag:    Tag which is used to identify the interface to remove.
            @type  interfaceTag:    str
            
            @raise:     InvalidRequest if the container tag is not valid.
        """
        try:
            self._containers[containerTag].removeInterface(interfaceTag)
        except KeyError:
            raise InvalidRequest('Container tag is invalid.')
    
    def activateInterface(self, containerTag, interfaceTag, robotID):
        """ Activate an interface to the container matching the given container tag.
            
            @param containerTag:    Container tag which is used to identify the
                                    container in which the interface should activated.
            @type  containerTag:    str
            
            @param interfaceTag:    Tag which is used to identify the interface to activate.
            @type  interfaceTag:    str
            
            @param robotID:     ID which is used to identify the robot for which the interface
                                should be activated.
            @type  robotID:     str
            
            @raise:     InvalidRequest if the container tag is not valid.
        """
        try:
            self._containers[containerTag].activateInterface(interfaceTag, robotID, self._commManager.commID)
        except KeyError:
            raise InvalidRequest('Container tag is invalid.')
    
    def deactivateInterface(self, containerTag, interfaceTag, robotID):
        """ Deactivate an interface to the container matching the given container tag.
            
            @param containerTag:    Container tag which is used to identify the
                                    container in which the interface should deactivated.
            @type  containerTag:    str
            
            @param interfaceTag:    Tag which is used to identify the interface to deactivate.
            @type  interfaceTag:    str
            
            @param robotID:     ID which is used to identify the robot for which the interface
                                should be deactivated.
            @type  robotID:     str
            
            @raise:     InvalidRequest if the container tag is not valid.
        """
        try:
            self._containers[containerTag].deactivateInterface(interfaceTag, robotID, self._commManager.commID)
        except KeyError:
            raise InvalidRequest('Container tag is invalid.')
    
    def addParameter(self, containerTag, name, value, paramType):
        """ Add a parameter to the parameter server in the container matching the
            given container tag.
            
            @param containerTag:    Container tag which is used to identify the
                                    container in which the parameter should be added.
            @type  containerTag:    str
            
            @param name:    Name of the parameter which should be added.
            @type  name:    str
            
            @param value:   Value of the parameter which should be added.
            @type  value:   Depends on @param paramType
            
            @param paramType:   Type of the parameter to add. Valid options are:
                                    int, str, float, bool, file
            @type  paramType:   str
            
            @raise:     InvalidRequest if the name is not a valid ROS name.
                        InvalidRequest if the container tag is not valid.
        """
        try:
            self._containers[containerTag].addParameter(name, value, paramType)
        except KeyError:
            raise InvalidRequest('Container tag is invalid.')
    
    def removeParameter(self, containerTag, name):
        """ Remove a parameter from the parameter server in the container matching
            the given container tag.
            
            @param containerTag:    Container tag which is used to identify the
                                    container from which the parameter should be
                                    removed.
            @type  containerTag:    str
            
            @param name:    Name of the parameter which should be removed.
            @type  name:    str
            
            @raise:     InvalidRequest if the container tag is not valid.
        """
        try:
            self._containers[containerTag].removeParameter(name)
        except KeyError:
            raise InvalidRequest('Container tag is invalid.')
    
    def sendROSMsgToContainer(self, containerTag, robotID, msg):
        """ Method is called when a complete message has been received by the robot
            and should now be processed and forwarded.
            
            @param containerTag:    Container tag which is used to identify a previously
                                    started container which should be used as message
                                    destination.
            @type  containerTag:    str
            
            @param robotID:     ID which is used to identify the sender of the message.
            @type  robotID:     str
            
            @param msg:     Corresponds to the dictionary of the field 'data' of the received
                            message. (Necessary keys: type, msgID, interfaceTag, msg)
            @type  msg:     { str : ... }
        """
        self._containers[containerTag].sendToInterface(robotID, msg)
    
    def sendROSMsgToRobot(self, containerTag, robotID, msg):
        """ Method is called when a message should be sent to the robot.
            
            @param containerTag:    Container tag which is used to identify the container
                                    from which the message originated.
            @type  containerTag:    str
            
            @param robotID:     ID which is used to identify the receiving robot.
            @type  robotID:     str
            
            @param msg:     Corresponds to the dictionary of the field 'data' of the message.
                            (Necessary keys: type, msgID, interfaceTag, msg)
            @type  msg:     { str : ... }
        """
        try:
            self._robots[robotID].sendROSMsgToRobot(containerTag, msg)
        except KeyError:
            raise InternalError('Connection to robot does not not exist.')
    
    def sendContainerUpdate(self, containerTag, status):
        """ Method is called when a container status update should be sent to
            the robot.
            
            @param containerTag:    Container tag which is used to identify the
                                    container for which an update is given.
            @type  containerTag:    str
            
            @param status:      New status of the container as a bool.
                                    True <=> Connected; False <=> Not Connected.
            @type  status:      bool
        """
        for robot in self._robots.itervalues():
            robot.sendContainerUpdate(containerTag, status)
