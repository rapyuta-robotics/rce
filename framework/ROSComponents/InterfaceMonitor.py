#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       InterfaceMonitor.py
#              
#       This file is part of the RoboEarth Cloud Engine framework.
#       
#       This file was originally created for RoboEearth
#       http://www.roboearth.org/
#       
#       The research leading to these results has received funding from
#       the European Union Seventh Framework Programme FP7/2007-2013 under
#       grant agreement no248942 RoboEarth.
#       
#       Copyright 2012 RoboEarth
#       
#       Licensed under the Apache License, Version 2.0 (the "License");
#       you may not use this file except in compliance with the License.
#       You may obtain a copy of the License at
#       
#       http://www.apache.org/licenses/LICENSE-2.0
#       
#       Unless required by applicable law or agreed to in writing, software
#       distributed under the License is distributed on an "AS IS" BASIS,
#       WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#       See the License for the specific language governing permissions and
#       limitations under the License.
#       
#       \author/s: Dominique Hunziker 
#       
#       

# ROS specific imports
import rospy

# twisted specific imports
from twisted.internet.task import LoopingCall

# Python specific imports
from threading import Lock
from datetime import datetime, timedelta
from uuid import uuid4

# Custom imports
import settings
from Exceptions import InternalError, InvalidRequest
import ComponentDefinition

class _InterfaceMonitor(object):
    """ Base class which is used to handle and monitor an interface.
    """
    def __init__(self, manager, reactor, interface):
        """ Initialize the Interface instance.
            
            @param manager:     Manager which is used in this node and implements
                                necessary callbacks.
            
            @param reactor:     Twisted reactor which is used in this node.
            
            @param interface:   Interface instance.
            @type  interface:   Interface
        """
        self._interfaceName = interface.name
        self._interfaceTag = interface.tag
        self._manager = manager
        self._reactor = reactor
        self._ready = False
        
        # List of all users: tuple (commID, userTag/interfaceTag)
        self._users = []
    
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
        if self._ready:
            return

        self._start()
        self._ready = True

    def _stop(self):
        """ This method should be overwritten to implement necessary tear
            down procedures.
        """
        pass

    def stop(self):
        """ This method is used to stop the interface.

            Don't overwrite this method; instead overwrite the method _stop.
        """
        if not self._ready:
            return

        self._stop()
        self._ready = False
    
    def addUser(self, commID, tag):
        """ Add a user of this interface.
            
            @param dest:    Communication ID of origin/destination.
            @type  dest:    str
            
            @param tag:     Name of the interface or name of the user.
            @type  tag:     str
        """
        user = (commID, tag)
        
        if user in self._users:
            raise InternalError('The same user is already registered in this interface.')
        
        self._users.append(user)
    
    def removeUser(self, commID, tag):
        """ Remove a user of this interface.
            
            @param dest:    Communication ID of origin/destination.
            @type  dest:    str
            
            @param tag:     Name of the interface or name of the user.
            @type  tag:     str
        """
        user = (commID, tag)
        
        if user not in self._users:
            raise InternalError('The user is not registered in this interface.')
        
        self._users.remove(user)
    
    def _send(self, msg, pushResult):
        """ This method should be overwritten to implement the send functionality.
        """
        raise InternalError('Interface does not support sending of a message.')

    def send(self, msg, msgID, commID, user, push):
        """ This method is used to send a message to a node.

            Don't overwrite this method; instead overwrite the method _send.
            If the interface does not overwrite the method _send, it is assumed that
            the interface does not support this action and an InternalError is
            raised.

            @param msg:     Message which should be sent in serialized form.
            @type  msg:     str
            
            @param msgID:   Message ID which is used to match response messages.
            @type  msgID:   str
            
            @param commID:  CommID from where the message originated.
            @type  commID:  str
            
            @param user:    Identifier which is used to identify the sender.
            @type  user:    str
            
            @param push:    Flag to indicate whether possible response messages
                            should be pushed back.
            @type  push:    bool
        """
        if not self._ready:
            raise InternalError('Interface is not ready to send a message.')
        
        user = (commID, user)
        
        if user not in self._users:
            raise InvalidRequest('User "{0}" is not allowed to use the interface "{1}".'.format(user, self._interfaceTag))
        
        if not push:
            user = None
        
        self._send(msg, msgID, user)

    def _receive(self, taskID):
        """ This method should be overwritten to implement the receive functionality.
        """
        raise InternalError('Interface does not support receiving of a message.')

    def receive(self, msgID, commID, user):
        """ This method is used to receive a message from a node.

            Don't overwrite this method; instead overwrite the method _receive.
            If the interface does not overwrite the method _receive, it is assumed
            that the interface does not support this action and an InternalError is
            raised.

            @param msgID:   ID which can be used to match a sent message to received
                            one or if this is not possible this argument will be
                            ignored.
            @type  msgID:   str
            
            @param commID:  CommID from where the message originated.
            @type  commID:  str
            
            @param user:    Identifier which is used to identify the sender.
            @type  user:    str
        """
        if not self._ready:
            raise InternalError('Interface is not ready to receive a message.')
        
        user = (commID, user)
        
        if user not in self._users:
            raise InvalidRequest('User "{0}" is not allowed to use the interface "{1}".'.format(user, self._interfaceTag))
        
        self._receive(msgID)

class ServiceMonitor(_InterfaceMonitor):
    """ Class which is used to handle and monitor a service interface.
    """
    IDENTIFIER = ComponentDefinition.INTERFACE_SRV
    
    class ServiceTask(object):
        """ Data container for a single Task, which consists of a
            request and its corresponding result.
        """
        __slots__ = ['lastAccessed', '_error', '_srv', '_msg', '_msgID', '_pushResult']

        def __init__(self, srv, msg, msgID, pushResult):
            """ Initialize the Task.

                @param srv:     ServiceInterface to which this task belongs.
                @type  srv:     ServiceInterface

                @param msg:     Message which should be sent as a request
                                in its serialized form.
                @type  msg:     str
                
                @param msgID:   Message ID which is used to match response messages.
                @type  msgID:   str
            
                @param pushResult:  Tuple of the form (commID, uid) to indicate to
                                    which communication node this result should be sent
                                    and a unique ID to identify the request/response.
                @type  pushResult:  ( str, str )
            """
            self._srv = srv
            self._msg = msg
            self._msgID = msgID
            self._pushResult = pushResult
            
            self.lastAccessed = None
            self._error = None

        def run(self):
            """ This method contains the "task" and is the part which should
                be executed in a separate thread.
            """
            msg = rospy.AnyMsg()
            msg._buff = self._msg

            try:
                rospy.wait_for_service(self._srv._interfaceName, timeout=settings.WAIT_FOR_SERVICE_TIMEOUT)
                serviceFunc = rospy.ServiceProxy(self._srv._interfaceName, self._srv._srvCls)
                response = serviceFunc(msg)
            except rospy.ROSInterruptException:
                return
            except Exception as e:
                self._error = e
            else:
                self._msg = response._buff
                self.lastAccessed = datetime.now()
            
            if self._pushResult:
                if self._error:
                    raise self._error
                else:
                    commID, user = self._pushResult
                    self._srv._manager.sendROSMessage( self._msg,
                                                       commID,
                                                       self._srv._interfaceTag,
                                                       user,
                                                       self._msgID )
        
        ### TODO: Not used for pure push implementation
        def getResult(self):
            """ Get the result of this task.

                - Returns serialized message which was received as response
                  to the sent request.
                - Returns None if the task has not yet been completed.
                - Raises an Error if there was an error in the run() method.
            """
            self._signalCompleted = True
            
            if self._error:
                # TODO: What to do with an error?
                raise self._error

            if self.lastAccessed:
                self.lastAccessed = datetime.now()
                return self._msg
            else:
                return None

    def __init__(self, manager, reactor, interface):
        super(ServiceMonitor, self).__init__(manager, reactor, interface)
        
        args = interface.srvCls.split('/')
        
        if len(args) != 2:
            raise InvalidRequest('Service type is not valid. Has to be of the form pkg/msg, i.e. std_msgs/Int8.')
        
        self._srvCls = manager.loader.loadSrv(*args)
        self._srvCls._request_class = rospy.AnyMsg
        self._srvCls._response_class = rospy.AnyMsg

        self._tasks = {}
        
        self._cleanUpCall = LoopingCall(self._clean)

    __init__.__doc__ = _InterfaceMonitor.__init__.__doc__
    
    def _start(self):
        self._cleanUpCall.start(settings.RESULT_TIMEOUT/4)
    
    def _stop(self):
        self._reactor.callLater(1.5*settings.RESULT_TIMEOUT, self._cleanUpCall.stop)
    
    def _send(self, msg, msgID, pushResult):
        task = ServiceMonitor.ServiceTask(self, msg, msgID, pushResult)
        self._reactor.callInThread(task.run)
        
        if not pushResult:
            self._tasks[msgID] = task
    
    ### TODO: Not used in pure push implementation
    def _receive(self, taskID):
        try:
            msg = self._tasks[taskID].getResult()
        except KeyError:
            raise InternalError('Invalid taskID used.')

        if not msg:
            # There is no response yet
            # TODO: Signal back, that there was no response, or identify empty msg with no response
            msg = ''
        else:
            # TODO: Add necessary values
            self._manager.sendROSMessage()
    
    def _clean(self):
        """ This method is used to remove old results from storage.
        """
        timeout = datetime.now() - timedelta(seconds=settings.RESULT_TIMEOUT)
        
        for key in [key for key in self._tasks.iterkeys() if self._tasks.lastAccessed and self._tasks.lastAccessed > timeout]:
            del self._tasks[key]

class PublisherMonitor(_InterfaceMonitor):
    """ Represents a publisher interface for a node.
    """
    IDENTIFIER = ComponentDefinition.INTERFACE_PUB
    
    def __init__(self, manager, reactor, interface):
        super(PublisherMonitor, self).__init__(manager, reactor, interface)
        
        args = interface.msgCls.split('/')
        
        if len(args) != 2:
            raise InvalidRequest('Message type is not valid. Has to be of the form pkg/msg, i.e. std_msgs/Int8.')
        
        self._msgCls = manager.loader.loadMsg(*args)

    __init__.__doc__ = _InterfaceMonitor.__init__.__doc__
    
    def _start(self):
        self._publisher = rospy.Publisher(self._interfaceName, self._msgCls, latch=True)

    def _stop(self):
        self._publisher.unregister()
        self._publisher = None

    def _send(self, msgData, msgID, pushResult):
        msg = rospy.AnyMsg()
        msg._buff = msgData

        try:
            self._publisher.publish(msg)
        except rospy.ROSInterruptException:
            pass
        except rospy.ROSSerializationException:
            raise InternalError('Message could not be serialized by ROS.')

class SubscriberMonitor(_InterfaceMonitor):
    """ Represents a subscriber interface for a node.
    """
    IDENTIFIER = ComponentDefinition.INTERFACE_SUB

    def __init__(self, manager, reactor, interface):
        super(SubscriberMonitor, self).__init__(manager, reactor, interface)

        self._lastMsg = None
        self._msgLock = Lock()

    __init__.__doc__ = _InterfaceMonitor.__init__.__doc__

    def _start(self):
        self._subscriber = rospy.Subscriber(self._interfaceName, rospy.AnyMsg, self._callbackFunc)

    def _stop(self):
        self._subscriber.unregister()
        self._subscriber = None

    ### TODO: Not used in pure push implementation
    def _receive(self, taskID):
        with self._msgLock:
            return self._lastMsg

    def _callbackFunc(self, msg):
        msg = msg._buff
        
        for commID, user in self._users:
            self._manager.sendROSMessage( msg,
                                          commID,
                                          self._interfaceTag,
                                          user,
                                          uuid4().hex )
        
        with self._msgLock:
            self._lastMsg = msg
