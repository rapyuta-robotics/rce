#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       ManagerBase.py
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

# ROS specific imports
import roslib; roslib.load_manifest('Administration')
import roslaunch.core
import roslaunch.scriptapi
import rospy

# Python sepcific imports
import threading
import os

# Custom imports
from MessageUtility import InternalError
import ThreadUtility

class NodeProcess(object):
    """ Class which represents a running node.
    """
    def __init__(self, process, files, namespace):
        """ Constructor.
            
            @param process:  Process in which the node is running.
            @type  process:  roslaunch.Process instance
            
            @param files:    List of all path of the temporary files
                             which are associated with this node.
            @type  files     [str]
            
            @param namespace:  ROS namespace in which the node resides.
            @type  namespace:  str
        """
        self.process = process
        self.files = files
        self.namespace = namespace
    
    def __del__(self):
        """ Destructor.
        """
        if self.process.is_alive():
            self.process.stop()
        
        del self.process
        
        for param in rospy.get_param_names():
            if param.find(self.namespace) == 0:
                rospy.delete_param(param)
        
        for filename in self.files:
            try:
                os.remove(filename)
            except OSError:
                pass
    
    def isAlive(self):
        """ Check whether the node/process is alive.
        """
        return self.process.is_alive()

class ManagerBase(ThreadUtility.QueueWorker):
    """ Base class for all managers.
    """
    def __init__(self):
        """ Constructor. Make sure to call this constructor.
        """
        super(ManagerBase, self).__init__()
        rospy.on_shutdown(self.terminate.set)
        
        self._ROSNamespace=roslib.names.get_ros_namespace()
        
        self._launcher = roslaunch.scriptapi.ROSLaunch()
        self._processLock = threading.Lock()
        self._runningProcesses = {}
        
        self._threadMngr = ThreadUtility.ThreadManager()
    
    @ThreadUtility.job
    def addProcess(self, key, node, namespace, tempFiles=[]):
        """ Add a process to the list.
            
            @param key:     Key with which the process will be accessed
                            in the future.
            @type  key:     str
            
            @param node:    Node which should be added to the list.
            @type  node:    roslaunch.core.Node
            
            @param namespace:   Namespace for the parameters. (Is not the
                                same as the namespace of the node.)
            @type  namespace:   str
            
            @param tempFiles:   Optional argument. (default = [])
                                List of temporary files which should be
                                associated with this node, i.e. which
                                will be removed as soon as the node is
                                removed.
            @type  tempFiles:   [tempfile.TemporaryFile]
            
            @raise: InternalError if the node could not be launched.
        """
        try:
            process = self._launcher.launch(node)
        except roslaunch.core.RLException as e:
            raise InternalError(e)
        
        with self._processLock:
            self._runningProcesses[key] = NodeProcess(process, tempFiles, namespace)
    
    @ThreadUtility.job
    def getProcess(self, key):
        """ Get a process matching the key.
            
            @param key:     Key which is used to identify the process
                            which should be accessed.
            @type  key:     str
            
            @return:        Process which should be added to the list.
            @rtype:         NodeProcess
            
            @raise:     KeyError if the given key does not exist.
        """
        with self._processLock:
            return self._runningProcesses[key]
    
    @ThreadUtility.job
    def removeProcess(self, key):
        """ Remove and terminate a process matching the key.
            
            @param key:     Key which is used to identify the process
                            which should be removed.
            
            @raise:     KeyError if the given key does not exist.
        """
        with self._processLock:
            try:
                del self._runningProcesses[key]
            except KeyError:
                print('Key Error: {0} is not valid.'.format(key))
    
    def getKeys(self):
        """ Get all keys.
        """
        with self._processLock:
            return self._runningProcesses.keys()
    
    def buildNamespace(self, add=''):
        """ Build a namespace by adding a name to the current namespace.
            
            @param add:     Name which should be added to current namespace.
                            Can also be omitted in which case the current
                            namespace is returned.
            @type  add:     str
            
            @return:        Complete absolute namespace.
            @rtype:         str
        """
        return '{0}{1}'.format(self._ROSNamespace, add)
    
    def init(self):
        """ Start the Manager. This method overwrites the method from the
            base class and is used to initialize the loop of the manager.
        """
        self._launcher.start()
    
    def start(self):
        """ Overwrite the method from base class threading.Thread to make
            sure that the manager is not run in a new Thread but in the
            main thread. (However it is still possible to launch the manager
            in a child thread.)
        """
        raise RuntimeError('The Manager can not be used in a child thread. Use run() in the main thread to launch the Manager instead.')
    
    def spin(self):
        """ Main loop of the Manager.
            
            Overwriting this method is forbidden!
            
            Doing so leads to Errors with the ROSLaunch. If additional
            functions should be executed in a loop overwite the method
            subspin().
        """
        subloop = ThreadUtility.Task(self.subspin)
        self._threadMngr.append(subloop, 1)
        subloop.start()
        self.run()
    
    def subspin(self):
        """ Main loop for subclasses of the Manager.
            
            If additional functions need to be added to the main loop of
            a subclass of Manager overwite this method.
        """
        pass
    
    def stop(self):
        """ Stop the Manager. This method overwrites the method from the
            base class and is used to finalize the loop of the manager.
            It is always executed.
        """
        with self._processLock:
            for key in self._runningProcesses:
                self.removeProcess(key)
            
            self._threadMngr.terminateAll()
            
            self._launcher.stop()
