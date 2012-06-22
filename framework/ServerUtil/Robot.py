#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       Robot.py
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
from Exceptions import InternalError
import ClientMsgTypes

class Robot(object):
    """ Class which represents a robot. A robot has a single websocket connection
        and a single User instance.
    """
    def __init__(self, user, robotID, conn):
        """ Initialize the Robot.
            
            @param user:    User to which this Robot belongs.
            @type  user:    User
            
            @param robotID:     ID of the represented robot.
            @type  robotID:     str
            
            @param conn:    Connection to the robot.
            @type  conn:    WebSocketCloudEngineProtocol
        """
        self._user = user
        self._robotID = robotID
        self._conn = conn
        
        self._activeInterfaces = set()
    
    @property
    def robotID(self):
        """ RobotID """
        return self._robotID
    
    def activateInterface(self, containerTag, interfaceTag):
        """ Activate an interface to the container matching the given container tag.
            - Update the list of all activated interfaces.
            - Forwards the call to the User instance. For more information refer to
              the doc string of the User class.
        """
        self._activeInterfaces.add((containerTag, interfaceTag))
        self._user.activateInterface(containerTag, interfaceTag, self._robotID)
    
    def deactivateInterface(self, containerTag, interfaceTag):
        """ Deactivate an interface to the container matching the given container tag.
            - Update the list of all activated interfaces.
            - Forwards the call to the User instance. For more information refer to
              the doc string of the User class.
        """
        interface = (containerTag, interfaceTag)
        
        if interface not in self._activeInterfaces:
            return
        
        self._activeInterfaces.remove(interface)
        self._user.deactivateInterface(containerTag, interfaceTag, self._robotID)
    
    def sendROSMsgToContainer(self, containerTag, msg):
        """ Method is called when a complete message has been received by the robot
            and should now be processed and forwarded.
            Forwards the call to the User instance. For more information refer to
            the doc string of the User class.
        """
        self._user.sendROSMsgToContainer(containerTag, self._robotID, msg)
    
    def sendROSMsgToRobot(self, containerTag, msg):
        """ Method is called when a message should be sent to the robot.
            
            @param containerTag:    Container tag which is used to identify the container
                                    from which the message originated.
            @type  containerTag:    str
            
            @param msg:     Corresponds to the dictionary of the field 'data' of the message.
                            (Necessary keys: type, msgID, interfaceTag, msg)
            @type  msg:     { str : ... }
        """
        try:
            self._conn.sendMessage({ 'type' : ClientMsgTypes.DATA_MESSAGE,
                                     'dest' : self._robotID,
                                     'orig' : containerTag,
                                     'data' : msg })
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
        self._conn.sendMessage({ 'type' : ClientMsgTypes.CONTAINER_STATUS,
                                 'dest' : self._robotID,
                                 'orig' : '$$$$$$',
                                 'data' : { containerTag : status } })
    
    def close(self):
        """ Method is called when the underlying connection has been closed.
        """
        for containerTag, interfaceTag in self._activeInterfaces:
            try:
                self._user.deactivateInterface(containerTag, interfaceTag, self._robotID)
            except InternalError:
                pass
        
        self._activeInterfaces.clear()
