#!/usr/bin/env python
# -*- coding: utf-8 -*-
#     
#     rce-comm/rce/comm/cred.py
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
#     Copyright 2013 RoboEarth
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
from rce.comm.interfaces import IRobotCredentials


class RobotCredentials(object):
    """ Robot credentials which are used to authenticate the websocket
        connection from the robot to the cloud engine.
    """
    implements(IRobotCredentials)
    
    def __init__(self, userID, robotID, key):
        """ Initialize the robot credentials.
            
            @param userID:      ID which is used to identify the owner of the
                                robot.
            @type  userID:      str
            
            @param robotID:     ID which is used to identify the robot.
            @type  robotID:     str
            
            @param key:         Key which should used for the authentication
                                of the user/robot.
            @type  key:         str
        """
        self._userID = userID
        self._robotID = robotID
        self._key = key
    
    @property
    def userID(self):
        """ User ID of the robot owner. """
        return self._userID
    
    @property
    def robotID(self):
        """ Robot ID to identify the robot. """
        return self._robotID
    
    def checkKey(self, key):
        """ Check if the provided key matches the stored key which has been
            received from the client.
            
            @param key:         True value of the key which should be used to
                                verify the credentials.
            @type  key:         str
            
            @return:            True if the key matches; False otherwise.
        """
        return self._key == key
