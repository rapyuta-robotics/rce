#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       Environment.py
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
isActive, addNode, getEnvironment, addTask, getTask, getFile, removeTask, \
isActiveResponse, addNodeResponse, getEnvironmentResponse, addTaskResponse, getTaskResponse, getFileResponse, removeTaskResponse

# Python sepcific imports
import json

# Custom imports
from MessageUtility import serviceCallback, setupService, InvalidRequest, InternalError
import EnvironmentManager

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

def isActiveCallback(request, manager):
    """ Callback function for the isActive Service.
    """
    return isActiveResponse(active=manager.isActive())

@serviceCallback(addNodeResponse)
def addNodeCallback(request, manager):
    """ Callback function for the addNode Service.
    """
    try:
        data = json.loads(request.data)
    except ValueError:
        raise InvalidRequest('data is not properly json encoded.')
    
    try:
        add = data['add']
        
        if not isinstance(add, dict):
            raise InvalidRequest('data/add does not contain a dict.')
    except (TypeError, AttributeError):
        raise InvalidRequest('data does not contain a dict.')
    except KeyError:
        add = {}
    
    try:
        remove = data['remove']
        
        if not isinstance(remove, list):
            raise InvalidRequest('data/remove does not contain a list.')
    except KeyError:
        remove = []
    
    for name in add:
        if not manager.isValidNodeName(name):
            raise InvalidRequest('{0} is not a valid node name.'.format(name))
    
    for name in remove:
        if not manager.isValidNodeName(name):
            raise InvalidRequest('{0} is not a valid node name.'.format(name))
    
    if add:
        manager.addNode(add, request.files)
    
    if remove:
        manager.removeNode(remove)

@serviceCallback(getEnvironmentResponse)
def getEnvironmentCallback(request, manager):
    """ Callback function for the getEnvironment Service.
    """
    try:
        return json.dumps(dict(manager.getNodeStatus()))
    except TypeError:
        raise InternalError('Could not get status of the nodes.')

@serviceCallback(addTaskResponse)
def addTaskCallback(request, manager):
    """ Callback function for the addTask Service.
    """
    if request.taskID:
        if not manager.isValidTask(request.taskID):
            raise InvalidRequest('Task ID {0} is invalid.'.format(request.taskID))
    else:
        request.taskID = manager.getNewTask()
    
    if request.data:
        try:
            data = json.loads(request.data)
        except ValueError:
            raise InvalidRequest('data is not properly json encoded.')
        
        manager.addTask(request.taskID, data, request.files)
    
    return json.dumps({ 'taskID' : request.taskID })

@serviceCallback(getTaskResponse)
def getTaskCallback(request, manager):
    """ Callback function for the getTask Service.
    """
    if not manager.isValidTask(request.taskID):
        raise InvalidRequest('Task ID {0} is invalid.'.format(request.taskID))
    
    (status, data) = manager.getResult(request.taskID)
    
    response = { 'taskID' : request.taskID, 'status' : status }
    
    if status == 'completed':
        response['data'] = json.loads(data)
    else:
        response['data'] = {}
    
    return json.dumps(response)

@serviceCallback(getFileResponse)
def getFileCallback(request, manager):
    """ Callback function for the getFile Service.
    """
    msg = manager.getFile(request.taskID, request.ref)
    
    if isinstance(msg, unicode):
        msg= msg.encode('utf-8')
    
    return msg

@serviceCallback(removeTaskResponse)
def removeTaskCallback(request, manager):
    """ Callback function for the removeTask Service.
    """
    if not manager.isValidTask(request.taskID):
        raise InvalidRequest('Task ID {0} is invalid.'.format(request.taskID))
    
    manager.removeTask(request.taskID)

########################################################################
# init / main

def init(manager):
    """ Set up the ROS Services.
        The function needs the Manager.Manager instance which should
        be used to communicate between the different threads and also to
        terminate the running threads.
    """
    # Initialize the node
    rospy.init_node('Environment')
    
    # Setup the Services
    setupService('isActive', isActive, isActiveCallback, manager)
    setupService('addNode', addNode, addNodeCallback, manager)
    setupService('getEnvironment', getEnvironment, getEnvironmentCallback, manager)
    setupService('addTask', addTask, addTaskCallback, manager)
    setupService('getTask', getTask, getTaskCallback, manager)
    setupService('getFile', getFile, getFileCallback, manager)
    setupService('removeTask', removeTask, removeTaskCallback, manager)

def main():
    manager = EnvironmentManager.EnvironmentManager()
    init(manager)
    manager.spin()
    
    return 0

if __name__ == '__main__':
    main()
