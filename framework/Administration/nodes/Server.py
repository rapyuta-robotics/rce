#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       Server.py
#       
#       Copyright 2011 dominique hunziker <dominique.hunziker@gmail.com>
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

# ROS specific imports
import roslib; roslib.load_manifest('Administration')
import rospy
from Administration.srv import \
addEnvironment, removeEnvironment, addEnvironmentResponse, removeEnvironmentResponse

# Python sepcific imports
import json

# Custom imports
from MessageUtility import serviceCallback, setupService
import ServerManager

########################################################################
# ROS callback functions

""" The ROS callback functions have the following constraints:
    
    @param request: ROS Service Request instance matching the
                    ROS Service.
    @type  request: ROS Service Request
    
    @param manager: Manager instance which is used to handle the
                    different tasks necessary to process the request.
    @type  manager: Manager.Manager
    
    @return:    The callback function return a ROS Service Response
                instance matching the ROS Service.
    @rtype:     ROS Service Response
    
    Important: -ROS callback functions can only have the argument request.
                Therefore, it is necessary to use a wrapper to get
                additional arguments.
                    -> message.setupService
                
               -To enable the use of the exceptions InternalError and
                InvalidRequest in the callback function it is necessary
                to decorate the callback functions.
                    -> message.serviceCallback
"""

@serviceCallback(addEnvironmentResponse)
def addEnvironmentCallback(request, manager):
    """ Callback function for the addEnvironment Service.
    """
    return json.dumps({ 'envID' : manager.getNewEnv() })

@serviceCallback(removeEnvironmentResponse)
def removeEnvironmentCallback(request, manager):
    """ Callback function for the removeEnvironment Service.
    """
    manager.removeEnv(request.envID)

########################################################################
# init / main

def init(manager):
    """ Set up the ROS Services.
        The function needs the Manager.Manager instance which should
        be used to communicate between the different threads and also to
        terminate the running threads.
    """
    # Initialize the node
    rospy.init_node('Server')
    
    # Setup the Services
    setupService('addEnvironment', addEnvironment, addEnvironmentCallback, manager)
    setupService('removeEnvironment', removeEnvironment, removeEnvironmentCallback, manager)

def main():
    manager = ServerManager.ServerManager()
    init(manager)
    manager.spin()
    
    return 0

if __name__ == '__main__':
    main()
