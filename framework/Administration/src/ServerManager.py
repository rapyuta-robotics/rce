#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       ServerManager.py
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
import roslaunch.core
import rospy
from Administration.srv import isActive

# Python specific imports
import time

# Custom imports
import settings
from MessageUtility import InvalidRequest, InternalError
import ROSUtility
from IDUtility import generateID
import ManagerBase

class ServerManager(ManagerBase.ManagerBase):
    """ This class is used to manage the environments, which are ROS nodes.
    """
    def __init__(self):
        """ Initialize the ServerManager.
            
            @raise:     ROSUtility.NodeError if an error occurred while
                        parsing the settings file for the node definitions.
        """
        super(ServerManager, self).__init__()
        
        self._nodeDict = ROSUtility.loadPossibleNodes()[0]
    
    def getNewEnv(self):
        """ Generate a new ID for an environment and add a new node for
            an environment to the ROS.
            
            @return:    New environment ID
            @rtype:     str
            
            @raise:     InternalError if environment node can not be
                        launched.
        """
        while True:
            uid = generateID()
            if uid not in self.getKeys():
                break
        
        namespace = self.buildNamespace(uid)
        self.addProcess(uid, roslaunch.core.Node('Administration', 'Environment.py', name=uid, namespace=namespace, output='screen'), namespace)
        
        return uid
    
    def removeEnv(self, uid):
        """ Terminate the node matching the given environment ID.
            
            @param uid:     Environment ID
            @type  uid:     str
            
            @raise:     InvalidRequest if environment ID is invalid.
        """
        self.removeProcess(uid)
    
    def getPossibleNodes(self):
        """ Return a list of all possible nodes.
            
            @return:    A list of all possible nodes. The information for
                        an individual node consists of a tuple of the
                        form ( nodeName, [services] )
            @rtype:     [ ( str, [str] ) ]
        """
        return [(key, self._nodeDict[key][2]) for key in self._nodeDict]
    
    def subspin(self):
        """ Main loop of the subclass of the Manager.
        """
        interval = 0.05
        limit = (settings.TIMEOUT/4.0)/interval
        counter = 0
        
        while not rospy.is_shutdown():
            time.sleep(interval)
            counter += 1
            
            if counter % limit == 0:
                counter = 0
                self._clean()
    
    def _clean(self):
        """ Delete ROS nodes which are inactive longer than settings.TIMEOUT.
        """
        for key in self.getKeys():
            name = '{0}/isActive'.format(key)
            
            try:
                rospy.wait_for_service(name, timeout=settings.WAIT_FOR_SERVICE_TIMEOUT)
                isActiveFunc = rospy.ServiceProxy(name, isActive)
                
                response = isActiveFunc()
                
                if not response.active:
                    self.removeEnv(key)
            except rospy.ROSException as e:
                self.removeEnv(key)
