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
from genmsg.names import package_resource_name
from genpy.message import Message
import rospy

# twisted specific imports
from twisted.internet.threads import deferToThreadPool

# Custom imports
from rce.error import InvalidRequest, InternalError
from rce.slave.interface import Interface


class _ROSInterfaceBase(Interface):
    """ Abstract base class which provides the basics for the ROS-side
        interfaces.
    """
    def __init__(self, owner, status, uid, clsName, addr):
        """ Initialize the ROS-side Interface.
            
            @param owner:       Namespace to which this interface belongs.
            @type  owner:       rce.environment.Environment
            
            @param status:      Status observer which is used to inform the
                                Master of the interface's status.
            @type  status:      twisted.spread.pb.RemoteReference
            
            @param uid:         Unique ID which is used to identify the
                                interface in the internal communication.
            @type  uid:         uuid.UUID
            
            @param clsName:     Message type/Service type consisting of the
                                package and the name of the message/service,
                                i.e. 'std_msgs/Int32'.
            @type  clsName:     str
            
            @param addr:        ROS name/address which the interface should
                                use.
            @type  addr:        str
        """
        Interface.__init__(self, owner, status, uid)
        
        self._name = addr
        self._reactor = owner.reactor


class ServiceClientInterface(_ROSInterfaceBase):
    """ Class which is used as a Service-Client Interface.
    """
    def __init__(self, owner, status, uid, clsName, addr):
        _ROSInterfaceBase.__init__(self, owner, status, uid, clsName, addr)
        
        try:
            pkg, name = package_resource_name(clsName)
        except ValueError:
            raise InvalidRequest('Service type is not valid. Has to be of the '
                                 'form pkg/srv, i.e. std_srvs/Empty.')
        
        self._srvCls = owner.loader.loadSrv(pkg, name)
        self._srvCls._request_class = rospy.AnyMsg
        self._srvCls._response_class = rospy.AnyMsg

    __init__.__doc__ = _ROSInterfaceBase.__init__.__doc__
    
    def _send(self, msg, msgID, protocol, remoteID):
        d = deferToThreadPool(self._reactor, self._reactor.getThreadPool(),
                              self._threadedCall, msg)
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
    """ Class which is used as a Service-Provider Interface.
    """
    def __init__(self, owner, status, uid, clsName, addr):
        _ROSInterfaceBase.__init__(self, owner, status, uid, clsName, addr)
        
        try:
            pkg, name = package_resource_name(clsName)
        except ValueError:
            raise InvalidRequest('Service type is not valid. Has to be of the '
                                 'form pkg/srv, i.e. std_srvs/Empty.')
        
        self._service = None
        self._pendingLock = Lock()
        self._pending = {}
        
        self._srvCls = owner.loader.loadSrv(pkg, name)
        self._srvCls._request_class = rospy.AnyMsg
        self._srvCls._response_class = rospy.AnyMsg
    
    __init__.__doc__ = _ROSInterfaceBase.__init__.__doc__
    
    def remote_connect(self, protocol, remoteID):
        if self._protocols:
            raise InternalError('Can not register more than one interface '
                                'at a time with a Service-Provider.')
        
        return _ROSInterfaceBase.remote_connect(self, protocol, remoteID)
    
    remote_connect.__doc__ = _ROSInterfaceBase.remote_connect.__doc__
    
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
        
        self._reactor.callFromThread(self.received, request._buff, msgID)
        
        # Block execution here until the event is set, i.e. a response has
        # arrived
        event.wait()
        
        with self._pendingLock:
            response = self._pending.pop(msgID, None)
        
        if not isinstance(response, Message):
            # TODO: Change exception?
            raise rospy.ROSInterruptException('Interrupted.')
        
        return response


class PublisherInterface(_ROSInterfaceBase):
    """ Class which is used as a Publisher Interface.
    """
    def __init__(self, owner, status, uid, clsName, addr):
        _ROSInterfaceBase.__init__(self, owner, status, uid, clsName, addr)
        
        try:
            pkg, name = package_resource_name(clsName)
        except ValueError:
            raise InvalidRequest('Message type is not valid. Has to be of the '
                                 'form pkg/msg, i.e. std_msgs/Int8.')
        
        self._msgCls = owner.loader.loadMsg(pkg, name)

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
    """ Class which is used as a Subscriber Interface.
    """
    def _start(self):
        self._subscriber = rospy.Subscriber(self._name, rospy.AnyMsg,
                                            self._callback)

    def _stop(self):
        self._subscriber.unregister()
        self._subscriber = None

    def _callback(self, msg):
        self._reactor.callFromThread(self.received, msg._buff, uuid4().hex)
