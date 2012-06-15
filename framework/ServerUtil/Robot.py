#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       Robot.py
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
        self._deactivateInterface(containerTag, interfaceTag, self._robotID)
    
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
            self._conn.sendMessage({ 'type' : ClientMsgTypes.MESSAGE,
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
        self._conn.sendMessage({ 'type' : ClientMsgTypes.STATUS_CONTAINER,
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
