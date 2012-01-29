#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       Manager.py
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

# Python specific imports
import time
import datetime
import threading
import json
import tempfile
import os
from functools import wraps

# Custom imports
import settings
from MessageUtility import serializeFiles, deserializeFiles, serializeResult, deserializeResult, \
SerializeError, InvalidRequest, InternalError
import ThreadUtility
import ROSUtility
from IDUtility import generateID
import SQLite
import ConverterBase
import ConverterUtility
import ManagerBase

def activity(f):
    """ Decorator to enable an automatic update of the activity timestamp. 
    """
    @wraps(f)
    def wrapper(*args, **kw):
        self = args[0]
        
        if not isinstance(self, EnvironmentManager):
            raise ValueError('First argument is not self.')
        
        with self._activityLock:
            self._lastActivity = datetime.datetime.utcnow()
        
        return f(*args, **kw)
    
    return wrapper
    
class EnvironmentManager(ManagerBase.ManagerBase):
    """ This class is used to manage an environment, which consists of
        the ROS nodes and the tasks for the ROS nodes.
    """
    _SQL_BASE = '''CREATE TABLE env (nodeID TEXT UNIQUE, name TEXT, status TEXT DEFAULT 'init');
CREATE TABLE task (taskID TEXT UNIQUE, status TEXT DEFAULT 'init', result BLOB, accessed TEXT);
CREATE TRIGGER task_insert AFTER INSERT ON task
BEGIN
UPDATE task SET accessed = DATETIME('NOW') WHERE taskID == new.taskID;
END;

CREATE TRIGGER task_update AFTER UPDATE ON task
BEGIN
UPDATE task SET accessed = DATETIME('NOW') WHERE taskID == new.taskID AND old.status != 'deleted';
END;'''

    def __init__(self):
        """ Initialize the EnvironmentManager.
            
            @raise:     ROSUtility.NodeError if an error occurred while
                        parsing the settings file for the node definitions.
        """
        super(EnvironmentManager, self).__init__()
        
        (self._nodeDict, self._serviceDict) = ROSUtility.loadPossibleNodes()
        
        # sqlite database connection
        self._db = SQLite.SQLite(self._SQL_BASE)
        self._threadMngr.append(self._db, 0)
        self._db.start()
        
        self._activityLock = threading.Lock()
        self._lastActivity = datetime.datetime.utcnow()
    
    ####################################################################
    # Task ID
    
    def getNewTask(self):
        """ Generate a new ID for a task and add a new entry for a task.
            
            @return:    new ID
            @rtype:     str
        """
        return self._db.newID('task', 'taskID')
    
    def isValidTask(self, task):
        """ Check whether the given task ID is valid or not.
            
            @param task:    Task ID
            @type  task:    str
            
            @return:    True if the task ID is vaild, False otherwise.
            @rtype:     bool
        """
        if len(self._db.query('''SELECT taskID FROM task WHERE taskID == ? AND status != 'deleted';''', (task,))) == 1:
            return True
        
        return False
    
    ####################################################################
    # Node
    
    def isValidNodeName(self, name):
        """ Check whether the given node name is valid or not.
            
            @param name:    Name of the node
            @type  name:    str
            
            @return:    True if the node name is vaild, False otherwise.
            @rtype:     bool
        """
        return name in self._nodeDict
    
    @activity
    def addNode(self, config, binary):
        """ Add new nodes in the environment. Make sure that the node
            names are valid before this method is used.
            
            @param config:  Dictionary which is compatible with the json
                            encoding and which has at it toplevel keys
                            which correspond to the given names. Each
                            value for a key/name should be another
                            dictionary containing the necessary
                            configuration for the matching node.
            @type  config:  { str : {} }
            
            @param binary:  Serialized files object from POST request.
            @type  binary:  str
        """
        files = deserializeFiles(binary)
        
        for name in config:
            tempFiles = []
            parameters = []
            receivedNodeConfig = config[name]
            
            try:
                # First load information about the node which should be launched
                try:
                    (pkgName, nodeCls, listOfServices, nodeConfig) = self._nodeDict[name]
                except KeyError:
                    raise InternalError('Could not get the node information for node {0}.'.format(name))
                
                # Create new entry for node in database
                nodeID = self._db.newID('env', 'nodeID')
                
                # Add the necessary configuration parameters to ParameterServer
                for (configName, configType, configDefaultValue) in nodeConfig:
                    # Try to read the given configuration value from request or else try to use the default value
                    try:
                        configVal = receivedNodeConfig[configName]
                    except KeyError:
                        configVal = configDefaultValue
                        
                        if not configVal:
                            raise InvalidRequest('Configuration parameter {0} is missing.'.format(configName))
                    else:
                        # If we have to add a file to the ParameterServer which we received from the user create a temporary file
                        if configType == 'file':
                            try:
                                configVal = ConverterUtility.resolveReference(configVal)
                            except ValueError:
                                content = configVal
                            else:
                                content = files[configVal]
                                content.seek(0)
                                content = content.read()
                            
                            (tmp, configVal) = tempfile.mkstemp()
                            tempFiles.append(configVal)
                            
                            with open(configVal, 'wb') as f:
                                f.write(content)
                    
                    # Now check the given value and process it accordingly
                    if configType == 'bool':
                        if isinstance(configVal, str) or isinstance(configVal, unicode):
                            configVal = configVal.lower()
                            
                            if configVal == 'true':
                                configVal = True
                            elif configVal == 'false':
                                configVal = False
                            else:
                                raise InvalidRequest('Configuration parameter {0} is not a valid bool.'.format(configName))
                        else:
                            configVal = bool(configVal)
                    elif configType == 'int':
                        try:
                            configVal = int(configVal)
                        except ValueError:
                            raise InvalidRequest('Configuration parameter {0} is not a valid int.'.format(configName))
                    elif configType == 'float':
                        try:
                            configVal = float(configVal)
                        except ValueError:
                            raise InvalidRequest('Configuration parameter {0} is not a valid float.'.format(configName))
                    elif configType == 'str':
                        if isinstance(configVal, unicode):
                            configVal = configVal.encode('utf-8')
                        
                        configVal = str(configVal)
                    elif configType == 'file':
                        if not os.path.isfile(configVal):
                            raise InternalError('Could not find file.')
                    else:
                        raise InternalError('Could not identify the type of the configuration parameter.')
                    
                    # Build the name of the parameter and add it to the ParameterServer
                    paramName = self.buildNamespace('{0}/{1}'.format(nodeID, configName))
                    
                    if rospy.has_param(paramName):
                        raise InternalError('Parameter already exists.')
                    
                    rospy.set_param(paramName, configVal)
                    parameters.append(paramName)
                
                # Create and launch now the node/process
                try:
                    self.addProcess(nodeID, roslaunch.core.Node(pkgName, nodeCls, name=nodeID, namespace=self.buildNamespace()), self.buildNamespace(nodeID), tempFiles)
                except InternalError:
                    self._db.change('''UPDATE env SET name = ?, status = 'aborted' WHERE nodeID == ?''', (name, nodeID))
                    raise
                
                self._db.change('''UPDATE env SET name = ?, status = 'running' WHERE nodeID == ?''', (name, nodeID))
            except Exception as e:
                # If there was any error delete the temporary files again and remove the parameters
                for filename in tempFiles:
                    try:
                        os.remove(filename)
                    except OSError:
                        pass
                
                for parameter in parameters:
                    rospy.delete_param(parameter)
                
                raise e
    
    def getNodeStatus(self):
        """ Get the status of the all nodes in the environment.
            
            @return:    Status of all nodes in the environment. List of
                        tuples of the form (name, status)
            @rtype:     [(str, str)]
        """
        nodes = self._db.query('SELECT nodeID, name, status FROM env')
        
        response = [None]*len(nodes)
        
        for i in xrange(len(nodes)):
            status = nodes[i][2]
            
            if status == 'init' or status == 'running':
                try:
                    if not self.getProcess(nodes[i][0]).isAlive():
                        status = 'terminated'
                except KeyError:
                    status = 'deleted'
            
            response[i] = (nodes[i][1], status)
        
        return response
    
    @activity
    def removeNode(self, names):
        """ Remove the nodes called names from the environment.
            
            @param names:   Names of nodes which should be deleted
            @type  names:   [str]
        """
        for name in names:
            self._db.change('''UPDATE env SET status = 'deleted' WHERE name == ?''', (name,))
    
    ####################################################################
    # Task
    
    @activity
    def addTask(self, task, config, binary):
        """ Add a new task with the given ID in the environment. Make
            sure that task ID is valid before this method is used.
            
            @param task:    Valid task ID (from getNewTask)
            @type  task:    str
            
            @param config:  Dictionary which is compatible with the json
                            encoding and which contains the necessary
                            configuration for the task.
            @type  config:  { str : {} }
            
            @param binary:  Serialized files object from POST request.
            @type  binary:  str
        """
        try:
            service = config['service']
        except (TypeError, AttributeError):
            self.abortTask(task)
            raise InvalidRequest('data does not contain a dict.')
        except KeyError:
            self.abortTask(task)
            raise InvalidRequest('Request does not define which service is requested.')
        
        try:
            msgData = config['msg']
        except KeyError:
            self.abortTask(task)
            raise InvalidRequest('Request does not define any message data.')
        
        try:
            (srvName, srvCls, reqCls) = self._serviceDict[service]
        except KeyError:
            self.abortTask(task)
            raise InvalidRequest('Requested service {0} is not valid.'.format(service))
        
        try:
            converter = ConverterBase.Converter()
            msg = converter.decode(reqCls, msgData, deserializeFiles(binary))
        except SerializeError:
            self.abortTask(task)
            raise InternalError('Could not deserialize files.')
        except (TypeError, ValueError) as e:
            self.abortTask(task)
            raise InvalidRequest(e)
        
        taskThread = ThreadUtility.Task(ROSUtility.runTask, self, task, srvName, srvCls, msg)
        self._threadMngr.append(taskThread, 2)
        self._db.change('''UPDATE task SET status = 'running' WHERE taskID == ?''', (task,))
        taskThread.start()
    
    def abortTask(self, task):
        """ Abort the task with the given ID in the environment. Make
            sure that the task is valid before this method is used.
            
            @param task:    Valid task ID (from getNewTask)
            @type  task:    str
        """
        self._db.change('''UPDATE task SET status = 'aborted' WHERE taskID == ?''', (task,))
    
    @activity
    def removeTask(self, task):
        """ Remove the task with the given ID in the environment. Make
            sure that the task is valid before this method is used.
            
            @param task:    Valid task ID (from getNewTask)
            @type  task:    str
        """
        self._db.change('''UPDATE task SET status = 'deleted' WHERE taskID == ?''', (task,))
    
    ####################################################################
    # Result
    
    @activity
    def addResult(self, task, msg):
        """ Add the result for the given task. Make sure that the task ID
            is valid before this method is used.
            
            @param task:    Valid task ID (from getNewTask)
            @type  task:    str
            
            @param msg:     Message which was received from the Service
                            as response.
            @type  msg:     ROS Service Response message instance
        """
        try:
            converter = ConverterBase.Converter()
            (data, files) = converter.encode(msg)
        except (TypeError, ValueError):
            self.abortTask(task)
            import traceback
            traceback.print_exc()
            return
        
        result = buffer(serializeResult(json.dumps(data), serializeFiles(files)))
        self._db.change('''UPDATE task SET result = ?, status = 'completed' WHERE taskID == ?''', (result, task))
    
    @activity
    def getResult(self, task):
        """ Get the state/result of the task. Make sure that the task ID
            is valid before this method is used.
            
            @param task:    Valid task ID (from getNewTask)
            @type  task:    str
            
            @return:    Tuple containing the information about the task:
                        (status, data, binary)
                        
                        If the status of the task is not yet 'completed'
                        than data and binary will be None.
                        The returned string data is the json formatted
                        string and binary is the serialized form of files.
                        (further information serialize.serializeFiles)
            @rtype:     (str, str, str)
        """
        response = self._db.query('SELECT status, result FROM task WHERE taskID == ?''', (task,))
        
        (status, result) = response[0]
        
        if status == 'completed':
            data, binary = deserializeResult(str(result))
            self._db.change('''UPDATE task SET accessed = DATETIME('NOW') WHERE taskID == ?''', (task,))
        else:
            data = ''
            binary = ''
        
        return (status, data, binary)
    
    ####################################################################
    # Utility
    
    def isActive(self):
        """ Check if the last activity was longer than settings.TIMEOUT
            ago.
        """
        with self._activityLock:
            return self._lastActivity > datetime.datetime.utcnow()-datetime.timedelta(seconds=abs(settings.TIMEOUT))
        
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
                self._clean(settings.TIMEOUT)
    
    def _clean(self, timeDelta):
        """ Mark all entries which are older than the given timeDelta
            for removal. 
            
            @param timeDelta:   Time in seconds without a change to the
                                environment after which the tasks/nodes
                                are removed.
            @type  timeDelta:   int
        """
        cutoff = datetime.datetime.utcnow()-datetime.timedelta(seconds=abs(timeDelta))
        
        for (taskID, accessed) in self._db.query('SELECT taskID, accessed FROM task'):
            if datetime.datetime.strptime(accessed, '%Y-%m-%d %H:%M:%S') < cutoff:
                self.removeTask(taskID)
    
    def _delete(self):
        """ Delete all entries which are marked for removal.
        """
        # Terminate all node processes marked for removal
        for (nodeID,) in self._db.query('''SELECT nodeID FROM env WHERE status == 'deleted' '''):
            self.removeProcess(nodeID)
        
        # Delete all nodes marked for removal
        self._db.change('''DELETE FROM env WHERE status ==  'deleted' ''')
        
        # Delete all task which are marked for removal
        self._db.change('''DELETE FROM task WHERE status ==  'deleted' ''')
