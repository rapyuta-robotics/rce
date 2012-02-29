#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       ROSUtility.py
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

# Python specific imports
import traceback

# Custom imports
import settings
from MessageUtility import InvalidRequest

def runService(manager, task, srvName, srvClsType, msg):
    """ Run a service request.
        
        @param manager: Used EnvironmentManager.
        @type  manager: EnvironmentManager.EnvironmentManager
        
        @param task:    Valid task ID.
        @type  task:    str
        
        @param srvName: Name of the service to use.
        @type  srvName: str
        
        @param srvClsType:  Service class which should be used.
                            Has to be of the form 'package/ServiceClass'.
        @type  srvClsType:  str
        
        @param msg:     Message which matches the given service class and
                        which should be sent.
        @type  msg:     ROS Service Request instance
    """
    try:
        srvCls = roslib.message.get_service_class(srvClsType)
        
        rospy.wait_for_service(srvName, timeout=settings.WAIT_FOR_SERVICE_TIMEOUT)
        serviceFunc = rospy.ServiceProxy(srvName, srvCls)
        
        response = serviceFunc(msg)
    except rospy.ROSInterruptException:
        return
    except:
        manager.abortTask(task)
        traceback.print_exc()
        return
    
    manager.addResult(task, response)

def runTopic(topicName, msgClsType, msg):
    """ Publish a message on a topic.
        
        @param topicName:   Name of the topic on which to publish.
        @type  topicName:   str
        
        @param msgClsType:  Message class which should be used.
                            Has to be of the form 'package/MessageClass'.
        @type  msgClsType:  str
        
        @param msg:     Message which matches the given message class and
                        which should be sent.
        @type  msg:     ROS Message instance
    """
    try:
        msgCls = roslib.message.get_message_class(msgClsType)
    except ValueError:
        raise InternalRequest('Message class is invalid.')
    
    try:
        rospy.Publisher(topicName, msgCls).publish(msg)
    except rospy.ROSInterruptException:
        return
    except rospy.ROSSerializationException:
        raise InternalError('Message could not be serialized by ROS.')
