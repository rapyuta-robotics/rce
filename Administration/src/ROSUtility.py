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
import ThreadUtility

class NodeError(Exception):
    """ This error is used if an error occurred while processing the
        settings and saving the node information.
    """

def loadPossibleNodes():
    """ Load the the possible nodes from the settings file.
        
        @return:    Tuple containing:
                    - Dictionary containing the definition for the
                      nodes where:
                        - Key:      Name with which the node is
                                    identified.
                        - Value:    Tuple of the form
                                    ( packageName,
                                      nodeExecutable,
                                      list of provided services
                                    )
                    - Dictionary containing the definiton for the
                      services where:
                        - Key:      Name with which the service is
                                    identified.
                        - Value:    Tuple of the form
                                    ( Service name,
                                      ROS Service class,
                                      ROS Service Request message class
                                    )
        @rype:      (   { str: (str, str, [str]) },
                        { str : (str, srvClass, srvRequest) }
                    )
        
        @raise:     NodeError if an error occurred while parsing the
                    defintion from settings.
    """
    nodeDict = {}
    serviceDict = {}
    
    for node in settings.NODES:
        for key in ['name', 'services']:
            if key not in node:
                raise NodeError('Not all necessary keys supplied.')
        
        nodeName = node['name']
        nodeServices = node['services']
        
        if nodeName in nodeDict:
            raise NodeError('Node name {0} is used multiple times.'.format(nodeName))
        
        (pkgName, nodeClass) = roslib.names.package_resource_name(nodeName)
        nodeDict[nodeName] = (pkgName, nodeClass, ['{0}/{1}'.format(pkgName, srvName) for (srvName, srvCls) in nodeServices])
        
        for (srvName, srvCls) in nodeServices:
            keyName = '{0}/{1}'.format(pkgName, srvName)
            srvCls = '{0}/{1}'.format(pkgName, srvCls)
            
            if keyName in serviceDict:
                raise NodeError('Service name {0} is used multiple times.'.format(srvName))
            
            serviceDict[keyName] = (srvName, srvCls, '{0}Request'.format(srvCls))
    
    return (nodeDict, serviceDict)

def runTask(manager, task, srvName, srvClsType, msg):
    """ Run a task.
        
        @param manager: Used EnvironmentManager.
        @type  manager: EnvironmentManager.EnvironmentManager
        
        @param task:    Valid task ID.
        @type  task:    str
        
        @param srvName: Name of the service to use.
        @type  srvName: str
        
        @param srvClsType:  Service class which should be used.
        @type  srvClsType:  ROS Service class
        
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
