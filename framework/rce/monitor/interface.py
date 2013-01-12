#!/usr/bin/env python
# -*- coding: utf-8 -*-
#     
#     interface.py
#     
#     This file is part of the RoboEarth Cloud Engine framework.
#     
#     This file was originally created for RoboEearth
#     http://www.roboearth.org/
#     
#     The research leading to these results has received funding from
#     the European Union Seventh Framework Programme FP7/2007-2013 under
#     grant agreement no248942 RoboEarth.
#     
#     Copyright 2013 RoboEarth
#     
#     Licensed under the Apache License, Version 2.0 (the "License");
#     you may not use this file except in compliance with the License.
#     You may obtain a copy of the License at
#     
#     http://www.apache.org/licenses/LICENSE-2.0
#     
#     Unless required by applicable law or agreed to in writing, software
#     distributed under the License is distributed on an "AS IS" BASIS,
#     WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#     See the License for the specific language governing permissions and
#     limitations under the License.
#     
#     \author/s: Dominique Hunziker 
#     
#     

# Python specific imports
from threading import Event, Lock
from uuid import uuid4

# ROS specific imports
import rospy
import genpy

# twisted specific imports
from twisted.internet.threads import deferToThread

# Custom imports
from rce.error import InvalidRequest
from rce.slave.interface import Interface


class _ROSInterfaceBase(Interface):
    """
    """
    def __init__(self, owner, uid, msgType, addr):
        """
        """
        Interface.__init__(self, owner, uid)
        
        self._name = addr


class ServiceClientInterface(_ROSInterfaceBase):
    """ Class which is used to handle and monitor a service interface.
    """
    def __init__(self, owner, uid, msgType, addr):
        _ROSInterfaceBase.__init__(self, owner, uid, msgType, addr)
        
        args = msgType.split('/')
        
        if len(args) != 2:
            raise InvalidRequest('Service type is not valid. Has to be of the '
                                 'form pkg/msg, i.e. std_msgs/Int8.')
        
        self._srvCls = owner.loader.loadSrv(*args)
        self._srvCls._request_class = rospy.AnyMsg
        self._srvCls._response_class = rospy.AnyMsg

    __init__.__doc__ = _ROSInterfaceBase.__init__.__doc__
    
    def _send(self, msg, msgID, protocol, remoteID):
        d = deferToThread(self._threadedCall, msg)
        d.addCallback(self._respond, msgID, protocol, remoteID)
        d.addErrback(self._errHandler)
    
    def _threadedCall(self, msg):
        rosMsg = rospy.AnyMsg()
        rosMsg._buff = msg
        
        rospy.wait_for_service(self._name, timeout=5)
        serviceFunc = rospy.ServiceProxy(self._name,
                                         self._srvCls)
        return serviceFunc(rosMsg)
    
    def _respond(self, resp, msgID, protocol, remoteID):
        self.respond(resp._buff, msgID, protocol, remoteID)
    
    def _errHandler(self, e):
        if e.check(rospy.ROSInterruptException):
            pass  # TODO: How should the error be returned?
        elif e.check(rospy.ROSSerializationException):
            pass  # TODO: How should the error be returned?
        else:
            e.printTraceback()


class ServiceProviderInterface(_ROSInterfaceBase):
    """ Represents a service-provider interface for a node.
    """
    def __init__(self, owner, uid, msgType, addr):
        _ROSInterfaceBase.__init__(self, owner, uid, msgType, addr)
        
        args = msgType.split('/')
        
        if len(args) != 2:
            raise InvalidRequest('Service type is not valid. Has to be of the '
                                 'form pkg/msg, i.e. std_msgs/Int8.')
        
        self._service = None
        self._pendingLock = Lock()
        self._pending = {}
        
        self._srvCls = owner.loader.loadSrv(*args)
        self._srvCls._request_class = rospy.AnyMsg
        self._srvCls._response_class = rospy.AnyMsg
    
    __init__.__doc__ = _ROSInterfaceBase.__init__.__doc__
    
    def _start(self):
        self._service = rospy.Service(self._name, self._srvCls, self._callback)
    
    def _stop(self):
        self._service.shutdown()
        self._service = None
        
        with self._pendingLock:
            for event in self._pending.itervalues():
                event.set()
            
            self._pending = {}
    
    def _send(self, msg, msgID, protocol, remoteID):
        rosMsg = rospy.AnyMsg()
        rosMsg._buff = msg
        
        try:
            with self._pendingLock:
                event = self._pending[msgID]
                self._pending[msgID] = rosMsg
        except KeyError:
            return
        
        event.set()
    
    def _callback(self, request):
        """ This method is called by the ROS framework when a Service request
            has arrived.
            Each call runs in a separate thread and has to block until a
            response is present, because the return value of this method is
            used as response to the request.
        """
        msgID = uuid4().hex
        event = Event()
        
        with self._pendingLock:
            self._pending[msgID] = event
        
        ###
        ### TODO: Do this check somewhere else...
        ###
#        if len(self._conn) > 1:
#            raise InternalError('Can not connect more than one interface to a '
#                                'service-provider interface.')
        
        self.received(request._buff, msgID)
        
        # Block execution here until the event is set, i.e. a response has
        # arrived
        event.wait()
        
        with self._pendingLock:
            response = self._pending.pop(msgID, None)
        
        if not isinstance(response, genpy.message.Message):
            # TODO: Change exception?
            raise rospy.ROSInterruptException('Interrupted.')
        
        return response


class PublisherInterface(_ROSInterfaceBase):
    """ Represents a publisher interface for a node.
    """
    def __init__(self, owner, uid, msgType, addr):
        _ROSInterfaceBase.__init__(self, owner, uid, msgType, addr)
        
        args = msgType.split('/')
        
        if len(args) != 2:
            raise InvalidRequest('Message type is not valid. Has to be of the '
                                 'form pkg/msg, i.e. std_msgs/Int8.')
        
        self._msgCls = owner.loader.loadMsg(*args)

    __init__.__doc__ = _ROSInterfaceBase.__init__.__doc__
    
    def _start(self):
        # TODO: Is 'latch=True' really necessary?
        #       Should this be configurable?
        self._publisher = rospy.Publisher(self._name, self._msgCls, latch=True)

    def _stop(self):
        self._publisher.unregister()
        self._publisher = None

    def _send(self, msg, msgID, protocol, remoteID):
        rosMsg = rospy.AnyMsg()
        rosMsg._buff = msg

        try:
            self._publisher.publish(rosMsg)
        except rospy.ROSInterruptException:
            pass  # TODO: How should the error be returned?
        except rospy.ROSSerializationException:
            pass  # TODO: How should the error be returned?


class SubscriberInterface(_ROSInterfaceBase):
    """ Represents a subscriber interface for a node.
    """
    def _start(self):
        self._subscriber = rospy.Subscriber(self._name, rospy.AnyMsg,
                                            self._callback)

    def _stop(self):
        self._subscriber.unregister()
        self._subscriber = None

    def _callback(self, msg):
        self.received(msg._buff, uuid4().hex)
