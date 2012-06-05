#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       InterfaceMonitor.py
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
import rospy
import genpy

# Python specific imports
from threading import Event, Lock, Thread
from datetime import datetime, timedelta
import time

# Custom imports
import settings
from Exceptions import InternalError, SerializationError
import ComponentDefinition
from MiscUtility import generateID

class _InterfaceMonitor(object):
    """ Base class which is used to handle and monitor an interface.
    """
    def __init__(self, interface, manager):
        """ Initialize the Interface instance.

            @param interface:   Interface instance.
            @type  interface:   Interface
        """
        self._interfaceName = interface.name
        self._interfaceTag = interface.tag
        self._manager = manager
        self.ready = False
        
        # Dictionary of all push receivers
        self.pushReceivers = {}
    
    @property
    def tag(self):
        """ Tag which is used to identify the interface. """
        return self._interfaceTag
    
    def _start(self):
        """ This method should be overwritten to implement necessary start
            up procedures.
        """
        pass

    def start(self):
        """ This method is used to setup the interface.

            Don't overwrite this method; instead overwrite the method _start.

            @raise:     InternalError if the interface can not be started.
        """
        if not self._manager:
            raise InternalError('Can not start an interface without a registered manager.')

        if self.ready:
            return

        self._start()

        self._manager.registerInterface(self)
        self.ready = True
    
    def addPushReceiver(self, commID, name):
        """ Add a receiver which should be automatically supplied with the messages.
            
            @param dest:    Communication ID of destination.
            @type  dest:    str
            
            @param name:    Name of the interface where the message should be published.
            @type  name:    str
        """
        if commID in self.pushReceivers:
            raise InternalError('"{0}" is already registered as a receiver for this interface.'.format(commID))
        
        self.pushReceivers[commID] = name
    
    def _send(self, msg, pushResult):
        """ This method should be overwritten to implement the send functionality.
        """
        raise InternalError('Interface does not support sending of a message.')

    def send(self, msg, pushResult=(None, None)):
        """ This method is used to send a message to a node.

            Don't overwrite this method; instead overwrite the method _send.
            If the interface does not overwrite the method _send, it is assumed that
            the interface does not support this action and an InternalError is
            raised.

            @param msg:     Message which should be sent in serialized form.
            @type  msg:     str
            
            @param pushResult:  Tuple of the form (commID, userID) to indicate to
                                which communication node this result should be sent
                                and which user is responsible for the request.
            @type  pushResult:  ( str, str )
            
            @return:    ID which can be used to match a received message to a
                        sent one if the pushResult flag is False or the .
            @rtype:     str
        """
        if not self.ready:
            raise InternalError('Interface is not ready to send a message.')

        return self._send(msg, pushResult)

    def _receive(self, taskID):
        """ This method should be overwritten to implement the receive functionality.
        """
        raise InternalError('Interface does not support receiving of a message.')

    def receive(self, taskID):
        """ This method is used to receive a message from a node.

            Don't overwrite this method; instead overwrite the method _receive.
            If the interface does not overwrite the method _receive, it is assumed
            that the interface does not support this action and an InternalError is
            raised.

            @param taskID:  ID which can be used to match a sent message to received
                            one or if this is not possible this argument will be
                            ignored.
            @type  taskID:  str

            @return:    Received message in serialized form.
            @rtype:     str
        """
        if not self.ready:
            raise InternalError('Interface is not ready to receive a message.')

        return self._receive(taskID)

    def _stop(self):
        """ This method should be overwritten to implement necessary tear
            down procedures.
        """
        pass

    def stop(self):
        """ This method is used to stop the interface.

            Don't overwrite this method; instead overwrite the method _stop.
        """
        if not self.ready:
            return

        self._stop()

        self.ready = False
        self._manager.unregisterInterface(self)

class ServiceMonitor(_InterfaceMonitor):
    """ Class which is used to handle and monitor a service interface.
    """
    IDENTIFIER = ComponentDefinition.INTERFACE_SRV
    
    class ServiceTask(object):
        """ Data container for a single Task, which consists of a
            request and its corresponding result.
        """
        __slots__ = ['_completed', '_error', '_srv', '_msg', '_id']

        def __init__(self, srv, msg, pushResult):
            """ Initialize the Task.

                @param srv:     ServiceInterface which uses this task.
                @type  srv:     ServiceInterface

                @param msg:     Message which should be sent as a request
                                in its serialized form.
                @type  msg:     str
            
                @param pushResult:  Tuple of the form (commID, uid) to indicate to
                                    which communication node this result should be sent
                                    and a unique ID to identify the request/response.
                @type  pushResult:  ( str, str )
            """
            self._id = None
            self._srv = srv
            self._msg = msg
            self._commID, self._pushID = pushResult
            
            self.lastAccessed = None
            
            self._hasID = Event()
            self._completed = False
            self._error = False
            self._signalCompletion = False

        def setID(self, taskID):
            """ Set the ID of this task.

                @param taskID:  ID which identifies this Task.
                @type  taskID:  str

                @raise:    InternalError if Task already has an ID.
            """
            if self._hasID.isSet():
                raise InternalError('Task already has an ID.')

            self._id = taskID
            self._hasID.set()

        def run(self):
            """ This method contains the "task" and is the part which should
                be executed in a separate thread.
            """
            msg = rospy.AnyMsg()
            msg._buff = self._msg

            try:
                rospy.wait_for_service(self._srv._interfaceName, timeout=settings.WAIT_FOR_SERVICE_TIMEOUT)
                serviceFunc = rospy.ServiceProxy(self._srv._interfaceName, self._srv.srvCls)
                response = serviceFunc(msg)
            except rospy.ROSInterruptException:
                return
            except Exception as e:
                self._msg = str(e)
                self._error = True
            else:
                self._msg = response._buff
                self.lastAccessed = datetime.now()
                self._completed = True
            
            if self._commID:
                self._manager.sendROSMessage(self._msg, self._commID, '', self._pushID)
            else:
                self._hasID.wait(1)
                
                if self._signalCompleted:
                    self._srv._signalCompletion(self._id)

        def getResult(self):
            """ Get the result of this task.

                - Returns serialized message which was received as response
                  to the sent request.
                - Returns None if the task has not yet been completed.
                - Raises an InternalError if there was an error in the run()
                  method.
            """
            self._signalCompleted = True

            if self._completed:
                self.lastAccessed = datetime.now()
                return self._msg
            elif self._error:
                # TODO: Good idea to raise an error here?
                raise InternalError(self._msg)
            else:
                return None

    def __init__(self, interface):
        super(ServiceMonitor, self).__init__(interface)

        try:
            self.srvCls = genpy.message.get_service_class(interface.srvClass)
        except (ValueError):
            raise SerializationError('Could not load Service class.')

        self.srvCls._request_class = rospy.AnyMsg
        self.srvCls._response_class = rospy.AnyMsg

        self._tasks = {}
        self._tasksLock = Lock()
        
        t = Thread(target=self._clean)
        t.daemon = True
        t.start()

    __init__.__doc__ = _InterfaceMonitor.__init__.__doc__

    def _send(self, msg, pushResult):
        task = ServiceMonitor.ServiceTask(self, msg, pushResult)
        self._manager.runTaskInSeparateThread(task.run)
        
        if pushResult[0]:
            while 1:
                uid = generateID()
    
                with self._tasksLock:
                    if uid not in self._tasks.keys():
                        self._tasks[uid] = task
                        break
    
            task.setID(uid)
    
            return uid
        else:
            return None
    
    ### TODO: Not used in pure push implementation
    def _receive(self, taskID):
        with self._tasksLock:
            try:
                msg = self._tasks[taskID].getResult()
            except KeyError:
                raise InternalError('Invalid taskID used.')

        if not msg:
            # There is no response yet
            # TODO: Signal back, that there was no response, or identify empty msg with no response
            msg = ''

        return msg

    def _signalCompletion(self, taskID):
        """ Callback function for ServiceTask instances to signal
            completion of the task (only on demand executed).

            @param taskID:  ID which identifies the completed task.
            @type  taskID:  str
        """
        # TODO: Add / change method send for manager...
        # TODO: What has to be sent to signal completion of task?
        pass
    
    def _clean(self):
        """ This method is used to remove old results from storage.
        """
        while 1:
            time.sleep(settings.RESULT_TIMEOUT/4)
            
            timeout = datetime.now() - timedelta(seconds=settings.RESULT_TIMEOUT)
            
            with self._tasksLock:
                for key in self._tasks.keys():
                    if self._tasks.lastAccessed and self._tasks.lastAccessed > timeout:
                        del self._tasks[key]

class PublisherMonitor(_InterfaceMonitor):
    """ Represents a publisher interface for a node.
    """
    IDENTIFIER = ComponentDefinition.INTERFACE_PUB

    def _start(self):
        self._publisher = rospy.Publisher(self._interfaceName, rospy.AnyMsg, latch=True)

    def _send(self, msgData, _):
        msg = rospy.AnyMsg()
        msg._buff = msgData

        try:
            self._publisher.publish(msg)
        except rospy.ROSInterruptException:
            pass
        except rospy.ROSSerializationException:
            raise InternalError('Message could not be serialized by ROS.')

        return ''

    def _stop(self):
        self._publisher.unregister()
        self._publisher = None

class SubscriberMonitor(_InterfaceMonitor):
    """ Represents a subscriber interface for a node.
    """
    IDENTIFIER = ComponentDefinition.INTERFACE_SUB

    def __init__(self, interface):
        super(SubscriberMonitor, self).__init__(interface)

        self._lastMsg = None
        self._msgLock = Lock()

    __init__.__doc__ = _InterfaceMonitor.__init__.__doc__

    def _start(self):
        self._subscriber = rospy.Subscriber(self._interfaceName, rospy.AnyMsg, self._callbackFunc)

    def _receive(self, taskID):
        with self._msgLock:
            return self._lastMsg

    def _stop(self):
        self._subscriber.unregister()
        self._subscriber = None

    def _callbackFunc(self, msg):
        with self._msgLock:
            for commID in self.pushReceivers:
                self._manager.sendROSMessage(msg._buff, commID, self.pushReceivers[commID], '')
            
            self._lastMsg = msg._buff
