#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       EnvironmentManager.py
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
import os
from functools import wraps

# Custom imports
import settings
from MessageUtility import deserializeFiles, SerializeError, InvalidRequest, InternalError
from MiscUtility import mktempfile
from SQLite import SQLite
from DjangoDB import DjangoDBError, getNodeDef, getInterfaceDef, existsNode
from ConverterBase import Converter
from ManagerBase import ManagerBase

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

class EnvironmentManager(ManagerBase):
    """ This class is used to manage an environment, which consists of
        the ROS nodes and the tasks for the ROS nodes.
    """
    _SQL_BASE = '''PRAGMA foreign_keys = ON;
CREATE TABLE env (nodeID TEXT UNIQUE, name TEXT, status TEXT DEFAULT 'init');
CREATE TABLE task (taskID TEXT UNIQUE, status TEXT DEFAULT 'init', result TEXT, accessed TEXT);
CREATE TABLE files (taskID TEXT, ref TEXT, filename TEXT UNIQUE, FOREIGN KEY(taskID) REFERENCES task(taskID) ON DELETE CASCADE);
CREATE TRIGGER task_insert AFTER INSERT ON task
BEGIN
UPDATE task SET accessed = DATETIME('NOW') WHERE taskID == new.taskID;
END;

CREATE TRIGGER task_update AFTER UPDATE ON task
BEGIN
UPDATE task SET accessed = DATETIME('NOW') WHERE taskID == new.taskID AND old.status != 'deleted';
END;;'''

    def __init__(self):
        """ Initialize the EnvironmentManager.
        """
        super(EnvironmentManager, self).__init__()

        # sqlite database connection
        self._db = SQLite(self._SQL_BASE)
        self._threadMngr.append(self._db, 0)
        self._db.start()

        # activity monitor
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

            @return:    True if the task ID is valid, False otherwise.
            @rtype:     bool
        """
        return len(self._db.query('''SELECT taskID FROM task WHERE taskID == ? AND status != 'deleted' ''', (task,))) == 1

    ####################################################################
    # Node

    def isValidNodeName(self, name):
        """ Check whether the given node name is valid or not.

            @param name:    Name of the node
            @type  name:    str

            @return:    True if the node name is valid, False otherwise.
            @rtype:     bool
        """
        return existsNode(name)

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
        try:
            files = deserializeFiles(binary)
        except SerializeError:
            raise InternalError('Could not deserialize files.')

        for name in config:
            # First load information about the node which should be launched
            try:
                (pkgName, nodeCls, nodeParams) = getNodeDef(name)
            except DjangoDBError:
                raise InternalError('Could not get the node information for node {0}.'.format(name))

            # Create new entry for node in database
            nodeID = self._db.newID('env', 'nodeID')

            # Add the necessary configuration parameters to the ParameterServer
            try:
                for param in nodeParams:
                    param.parse(config[name], files)

                    # Build the name of the parameter and add it to the ParameterServer
                    param.setParam(self.buildNamespace('{0}/{1}'.format(nodeID, param.name)))

                # Create and launch now the node/process
                # for debugging add  > output='screen' < to Node args
                self.addProcess(nodeID, roslaunch.core.Node(pkgName, nodeCls, name=nodeID, namespace=self.buildNamespace(), output='screen'), nodeParams)
            except (InvalidRequest, InternalError):
                self._db.change('''UPDATE env SET name = ?, status = 'aborted' WHERE nodeID == ?''', (name, nodeID))
                raise

            self._db.change('''UPDATE env SET name = ?, status = 'running' WHERE nodeID == ?''', (name, nodeID))

    def getNodeStatus(self):
        """ Get the status of all the nodes in the environment.

            @return:    Status of all nodes in the environment. Dictionary
                        with the node name as key and the status as
                        corresponding value.
            @rtype:     { str : str }
        """
        nodes = self._db.query('SELECT nodeID, name, status FROM env')

        response = {}

        for (processKey, nodeName, status) in nodes:
            if status in ['init', 'running']:
                processID = self.getProcess(processKey)

                if processID:
                    if not processID.isAlive():
                        status = 'terminated'
                else:
                    status = 'deleted'

            response[nodeName] = status

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
        if not isinstance(config, dict):
            self.abortTask(task, 'invalid data')
            raise InvalidRequest('data does not contain a dict.')

        if 'interface' in config:
            interface = config['interface']
        else:
            self.abortTask(task, 'undefined interface')
            raise InvalidRequest('Request does not define which interface is requested.')

        if 'msg' in config:
            msgData = config['msg']
        else:
            self.abortTask(task, 'message data missing')
            raise InvalidRequest('Request does not define any message data.')

        try:
            interface = getInterfaceDef(interface)
        except DjangoDBError:
            self.abortTask(task, 'invalid interface')
            raise InvalidRequest('Requested interface {0} is not valid.'.format(interface))

        try:
            files = deserializeFiles(binary)
        except SerializeError:
            self.abortTask(task, 'serialization error')
            raise InternalError('Could not deserialize files.')

        try:
            interface.parseMessage(msgData, files)
        except InvalidRequest:
            self.abortTask(task, 'invalid request message')
            raise

        try:
            interface.run(self, task)
        except InternalError:
            self.abortTask(task, 'unable to send message')
            raise

    def setTaskRunning(self, task):
        """ Set the status of the task to 'running'. Make sure that the
            task is valid before this method is used.

            @param task:    Valid task ID (from getNewTask)
            @type  task:    str
        """
        self._db.change('''UPDATE task SET status = 'running' WHERE taskID == ?''', (task,))

    def abortTask(self, task, msg=''):
        """ Abort the task with the given ID in the environment. Make
            sure that the task is valid before this method is used.

            @param task:    Valid task ID (from getNewTask)
            @type  task:    str

            @param msg:     Optional argument.
                            Error message which should be saved and returned
                            on request
            @type  msg:     str
        """
        self._db.change('''UPDATE task SET result = ?, status = 'aborted' WHERE taskID == ?''', (json.dumps({ 'error' : msg }), task))

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
    def addResult(self, task, msg=None):
        """ Add the result for the given task. Make sure that the task ID
            is valid before this method is used.

            @param task:    Valid task ID (from getNewTask)
            @type  task:    str

            @param msg:     Message which was received from the interface
                            as response or None if no message is returned.
            @type  msg:     ROS Service Response message instance
        """
        if msg:
            try:
                converter = Converter()
                (data, files) = converter.encode(msg)
            except (TypeError, ValueError):
                self.abortTask(task, 'invalid response message')
                import traceback
                traceback.print_exc()
                return

            for ref in files:
                (f, name) = mktempfile(dir=settings.TMP_RESULT_DIR)
                self._db.change('''INSERT INTO files (taskID, ref, filename) VALUES (?, ?, ?)''', (task, ref, name))
                f.write(files[ref].getvalue())
                f.close()
                os.chmod(name, 0644)
        else:
            data = {}

        self._db.change('''UPDATE task SET result = ?, status = 'completed' WHERE taskID == ?''', (json.dumps(data), task))

    @activity
    def getResult(self, task):
        """ Get the state/result of the task. Make sure that the task ID
            is valid before this method is used.

            @param task:    Valid task ID (from getNewTask)
            @type  task:    str

            @return:    Tuple containing the information about the task:
                        (status, data)

                        If the status of the task is not yet 'completed'
                        than data will be None.
                        The returned string data is the json formatted
                        string.
            @rtype:     (str, str)
        """
        response = self._db.query('SELECT status, result FROM task WHERE taskID == ?''', (task,))
        self._db.change('''UPDATE task SET accessed = DATETIME('NOW') WHERE taskID == ?''', (task,))

        if not response:
            raise InvalidRequest('Given task ID is invalid.')

        return response[0]

    @activity
    def getFile(self, task, ref):
        """ Get the a file/result of the task. Make sure that the task ID
            is valid before this method is used.

            @param task:    Valid task ID (from getNewTask)
            @type  task:    str

            @param ref:     Valid reference
            @type  ref:     str

            @return:    Absolute path to the temporary file which was
                        requested.
            @rtype:     str
        """
        response = self._db.query('SELECT filename FROM files WHERE taskID == ? AND ref == ?''', (task, ref))
        self._db.change('''UPDATE task SET accessed = DATETIME('NOW') WHERE taskID == ?''', (task,))

        if not response:
            raise InvalidRequest('Given task ID / file reference is invalid.')

        return response[0][0]


    ####################################################################
    # Utility

    def addTaskToThreadMngr(self, thread):
        """ Add a thread to the current used thread manager.

            @param thread:  Thread instance which should be added to the
                            current thread manager.
            @type  thread:  ManagedThread
        """
        self._threadMngr.append(thread, 2)

    def isActive(self):
        """ Check if the last activity was longer than settings.TIMEOUT
            ago.
        """
        with self._activityLock:
            return self._lastActivity > datetime.datetime.utcnow() - datetime.timedelta(seconds=abs(settings.TIMEOUT))

    def subspin(self):
        """ Main loop of the subclass of the Manager.
        """
        interval = 0.05
        limit = (settings.TIMEOUT / 4.0) / interval
        counter = 0

        while not rospy.is_shutdown():
            time.sleep(interval)
            counter += 1

            if counter % limit == 0:
                counter = 0
                self._clean(settings.TIMEOUT)

    def stop(self):
        """ Overwrites the method from base class. This is necessary to
            make sure that all temporary files are removed.
        """
        for (filename,) in self._db.query('''SELECT filename FROM files'''):
            try:
                os.remove(filename)
            except OSError:
                pass

        super(EnvironmentManager, self).stop()

    def _clean(self, timeDelta):
        """ Mark all entries which are older than the given timeDelta
            for removal.

            @param timeDelta:   Time in seconds without a change to the
                                environment after which the tasks/nodes
                                are removed.
            @type  timeDelta:   int
        """
        cutoff = datetime.datetime.utcnow() - datetime.timedelta(seconds=abs(timeDelta))

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
        self._db.change('''DELETE FROM env WHERE status == 'deleted' ''')

        # Delete all temporary files which are associated with the tasks marked for removal
        for (filename,) in self._db.query('''SELECT files.filename FROM files INNER JOIN task ON files.taskID = task.taskID WHERE task.status == 'deleted' '''):
            try:
                os.remove(filename)
            except OSError:
                pass

        # Delete all task which are marked for removal
        self._db.change('''DELETE FROM task WHERE status == 'deleted' ''')
