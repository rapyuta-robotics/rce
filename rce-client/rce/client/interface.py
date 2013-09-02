#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     rce-client/rce/client/interface.py
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
#     Copyright 2012 RoboEarth
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
import zlib
from uuid import uuid4
from threading import Condition, Lock

try:
    from cStringIO import StringIO
except ImportError:
    from StringIO import StringIO

# ROS specific imports; if available
try:
    import rospy
    import genpy.message
    HAS_ROS = True
except ImportError:
    HAS_ROS = False

# twisted specific imports
from twisted.internet.defer import Deferred
from twisted.internet.threads import deferToThreadPool


# Compression level used for communication
#     0:    use no compression
#     1-9:  use compression (1: fastest; 9: slowest, best compression)
_GZIP_LVL = 0


class InterfaceDisabledError(Exception):
    """ Exception is raised when an interface is called even though the
        interface is disabled.
    """


class _Base(object):
    """ Abstract base for all Interface classes.
    """
    UNIQUE = False
    CALLABLE = False

    _UP_MSG = "Interface '{0}' is up."
    _DOWN_MSG = "Interface '{0}' is down."

    def __init__(self, conn, iTag, clsName):
        """ Initialize the Interface.
        """
        self._status = False
        self._statusListener = set()

        print(self._UP_MSG.format(iTag))
        self._conn = conn
        self._iTag = iTag
        self._clsName = clsName

        conn.registerInterface(iTag, self)
        self._registered = True

    def __del__(self):
        """ Finalize the Interface.
        """
        self._unregister()

        if hasattr(self, '_iTag'):
            print(self._DOWN_MSG.format(self._iTag))

    @property
    def status(self):
        """ Status (enabled/disabled) of the interface.
        """
        return self._status

    def registerStatusListener(self, cb):
        """ Register a status listener which will be called with status changes.

            @param cb:          Callable which takes this interface instance as
                                first and the status flag as second argument.
            @type  cb:          callable
        """
        self._statusListener.add(cb)

    def unregisterStatusListener(self, cb):
        """ Unregister a previously registered status listener.

            @param cb:          Callable which should unregistered.
            @type  cb:          callable
        """
        self._statusListener.discard(cb)

    def setEnabled(self, status):
        """ Callback for RCE client to enable/disable the interface.
        """
        self._status = status

        if status:
            self._start()
        else:
            self._stop()

        for cb in self._statusListener:
            cb(self, status)

    def _start(self):
        """ Hook which is called when the status of the interface switches from
            disabled to enabled.
        """

    def _stop(self):
        """ Hook which is called when the status of the interface switches from
            enabled to disabled.
        """

    def _unregister(self):
        """ Internally used method to unregister the interface from the RCE
            client.
        """
        if hasattr(self, '_registered') and self._registered:
            self._conn.unregisterInterface(self._iTag, self)
            self._registered = False


class _CallableBase(_Base):
    """ Abstract base for all Interface classes which are callable by the RCE
        client.
    """
    CALLABLE = True

    def callback(self, msgType, msg, msgID):
        """ Callback for RCE. To implement the callback overwrite the hook
            '_callback'.
        """
        if not self._status:
            raise InterfaceDisabledError('A disabled interface should not be '
                                         'called by the RCE client.')

        if not msgType == self._clsName:
            raise TypeError('Received unexpected message type.')

        self._callback(msg, msgID)

    def _callback(self, msg, msgID):
        """ Callback to process the received message.
        """
        raise NotImplementedError('Method _callback has not been implemented.')


class _Publisher(_Base):
    """ Abstract implementation of a Publisher Interface.
    """
    _UP_MSG = "Publisher to RCE Interface '{0}' is up."
    _DOWN_MSG = "Publisher to RCE Interface '{0}' is down."

    def publish(self, msg):
        """ Publish a message.
        """
        if not self._status:
            raise InterfaceDisabledError('A disabled interface should not be '
                                         'called.')

        self._conn.sendMessage(self._iTag, self._clsName, msg, 'nil')


class _Subscriber(_CallableBase):
    """ Abstract implementation of a Subscriber Interface.
    """
    _UP_MSG = "Subscriber to RCE Interface '{0}' is up."
    _DOWN_MSG = "Subscriber to RCE Interface '{0}' is down."

    def __init__(self, conn, iTag, msgType, cb):
        """ Initialize the Subscriber.
        """
        self._cb = cb

        super(_Subscriber, self).__init__(conn, iTag, msgType)

    def unsubscribe(self):
        """ Unsubscribe interface from topic. Afterwards no more messages are
            given to the registered callback. To resubscribe a new interface
            has to be created.
        """
        self._unregister()

    def _callback(self, msg, _):
        """ Callback hook.
        """
        self._cb(msg)


class _ServiceClient(_CallableBase):
    """ Abstract implementation of a Service Client Interface.
    """
    UNIQUE = True

    _UP_MSG = "Service Client to RCE Interface '{0}' is up."
    _DOWN_MSG = "Service Client to RCE Interface '{0}' is down."

    def __init__(self, conn, iTag, srvType):
        """ Initialize the Service Client.
        """
        self._responses = {}

        super(_ServiceClient, self).__init__(conn, iTag, srvType)

    def _call(self, msg, cb, *args):
        """ Internally used method which should be used to call the service.

            @param msg:         Request message which should be sent.

            @param cb:          Callback which should be called to process
                                response. The response will be the first
                                argument and additional arguments passed to the
                                _call method will be passed to the callback.

            @param *args:       Additional arguments which will be passed to the
                                callback.
        """
        if not self._status:
            raise InterfaceDisabledError('A disabled interface should not be '
                                         'called.')

        if not callable(cb):
            raise TypeError('Callback has to be callable.')

        uid = uuid4().hex
        deferred = Deferred()
        deferred.addCallback(cb, *args)
        self._responses[uid] = deferred

        self._conn.sendMessage(self._iTag, self._clsName, msg, uid)

    def _callback(self, msg, msgID):
        """ Callback hook.
        """
        deferred = self._responses.pop(msgID, None)

        if deferred:
            deferred.callback(msg)
        else:
            print('Received service response which can not be associated '
                  'with any request.')


class _ServiceProvider(_CallableBase):
    """ Abstract implementation of a Service Provider Interface.
    """
    UNIQUE = True

    _UP_MSG = "Service Provider to RCE Interface '{0}' is up."
    _DOWN_MSG = "Service Provider to RCE Interface '{0}' is down."

    def __init__(self, conn, iTag, srvType, cb, args):
        """ Initialize the Service Provider.
        """
        self._cb = cb
        self._args = args

        super(_ServiceProvider, self).__init__(conn, iTag, srvType)

    def unregister(self):
        """ Unregister the service provider. Afterwards no more service requests
            are given to the registered callback. To reregister the service
            provider a new interface has to be created.
        """
        self._unregister()

    def _callback(self, msg, msgID):
        """ Callback hook.
        """
        reactor = self._conn._reactor
        d = deferToThreadPool(reactor, reactor.getThreadPool(),
                              self._cb, msg, *self._args)
        d.addCallback(self._response_success, msgID)
        d.addErrback(self._response_failure, msgID)

    def _response_success(self, msg, msgID):
        """ Internally used method which is executed when the service has been
            successfully called.
        """
        if not self._status:
            # Can not help it if the response takes some time and in the mean
            # time the interface is disabled; therefore, don't raise an error
            # instead just skip sending the response
            return

        self._conn.sendMessage(self._iTag, self._clsName, msg, msgID)

    def _response_failure(self, failure, msgID):
        """ Internally used method which is executed when the service call has
            failed.
        """
        if not self._status:
            # Can not help it if the response takes some time and in the mean
            # time the interface is disabled; therefore, don't raise an error
            # instead just skip sending the response
            return

        # TODO: Return something useful to the cloud here!
        print('Service call failed.')


class Publisher(_Publisher):
    """ Representation of a Publisher Interface.
    """


class Subscriber(_Subscriber):
    """ Representation of a Subscriber Interface.
    """


class ServiceClient(_ServiceClient):
    """ Representation of a Service Client Interface.
    """
    def __init__(self, conn, iTag, srvType, cb):
        """ Initialize the Service Client.
        """
        super(ServiceClient, self).__init__(conn, iTag, srvType)

        self._cb = cb

    def call(self, msg, cb=None):
        """ Call the Service Client.

            @param msg:     Request message which should be sent.
            @type  msg:     JSON compatible dictionary.

            @param cb:      Callback function which will be called with the
                            response message as argument. If parameter is
                            omitted the default callback is tried as fall-back.
            @type  cb:      Callable / None
        """
        self._call(msg, cb or self._cb)


class ServiceProvider(_ServiceProvider):
    """ Representation of a Service Provider Interface.
    """


if HAS_ROS:
    class TimeoutExceeded(Exception):
        """ Exception is raised when the timeout has passed without getting the
            reference of the Event.
        """


    class _EventRef(object):
        """ Helper class which acts as a threading.Event, but which can be used
            to pass a reference when signaling the event.
        """
        def __init__(self):
            self._cond = Condition(Lock())
            self._flag = False
            self._ref = None

        def isSet(self):
            return self._flag

        def set(self, ref):
            with self._cond:
                assert self._ref is None
                self._ref = ref
                self._flag = True
                self._cond.notifyAll()

        def get(self, timeout=None):
            with self._cond:
                if not self._flag:
                    self._cond.wait(timeout)

                if not self._flag:
                    raise TimeoutExceeded('Could not get the reference.')

                return self._ref

        def clear(self, ref):
            with self._cond:
                self._ref = None
                self._flag = False


    class ROSPublisher(_Publisher):
        """ Representation of a Publisher Interface using ROS.
        """
        def __init__(self, conn, iTag, msgType, addr):
            """ Initialize the Publisher.
            """
            self._sub = None
            self._addr = addr

            super(ROSPublisher, self).__init__(conn, iTag, msgType)

        def _rosCB(self, msg):
            """ Internally used callback for ROS Subscriber.
            """
            if _GZIP_LVL:
                self.publish(StringIO(zlib.compress(msg._buff, _GZIP_LVL)))
            else:
                self.publish(StringIO(msg._buff))

        def _start(self):
            self._sub = rospy.Subscriber(self._addr, rospy.AnyMsg, self._rosCB)
            print("Local ROS Subscriber on topic '{0}' is "
                  'up.'.format(self._addr))

        def _stop(self):
            self._sub.unregister()
            self._sub = None
            print("Local ROS Subscriber on topic '{0}' is "
                  'down.'.format(self._addr))

        def __del__(self):
            """ Finalize the Publisher.
            """
            if self._sub:
                self._stop()

            super(ROSPublisher, self).__del__()


    class ROSSubscriber(_Subscriber):
        """ Representation of a Subscriber Interface using ROS.
        """
        def __init__(self, conn, iTag, msgType, addr):
            """ Initialize the Subscriber.
            """
            self._pub = None
            self._addr = addr
            self._args = msgType.split('/')

            if len(self._args) != 2:
                raise ValueError('Message type is not valid. Has to be of the '
                                 'form pkg/msg, i.e. std_msgs/Int8.')

            super(ROSSubscriber, self).__init__(conn, iTag, msgType,
                                                self._rceCB)

        def _rceCB(self, msg):
            """ Internally used method to send received messages to the ROS
                Publisher.
            """
            rosMsg = rospy.AnyMsg()

            if _GZIP_LVL:
                rosMsg._buff = zlib.decompress(msg.getvalue())
            else:
                rosMsg._buff = msg.getvalue()

            self._pub.publish(rosMsg)

        def _start(self):
            self._pub = rospy.Publisher(self._addr,
                                        self._conn.loader.loadMsg(*self._args))
            print("Local ROS Publisher on topic '{0}' is "
                  'up.'.format(self._addr))

        def _stop(self):
            self._pub.unregister()
            self._pub = None
            print("Local ROS Publisher on topic '{0}' is "
                  'down.'.format(self._addr))

        def __del__(self):
            """ Finalize the Subscriber.
            """
            if self._pub:
                self._stop()

            super(ROSSubscriber, self).__del__()


    class ROSServiceClient(_ServiceClient):
        """ Representation of a Service Client Interface using ROS.
        """
        def __init__(self, conn, iTag, srvType, addr):
            """ Initialize the Service Client.
            """
            self._service = None
            self._addr = addr
            self._lock = Lock()
            self._pending = set()

            args = srvType.split('/')

            if len(args) != 2:
                raise ValueError('Service type is not valid. Has to be of the '
                                 'form pkg/srv, i.e. std_msgs/Int8.')

            self._srvCls = conn.loader.loadSrv(*args)
            self._srvCls._request_class = rospy.AnyMsg
            self._srvCls._response_class = rospy.AnyMsg

            super(ROSServiceClient, self).__init__(conn, iTag, srvType)

        def _rosCB(self, rosReq):
            """ Internally used callback for ROS Service.
            """
            event = _EventRef()

            if _GZIP_LVL:
                req = StringIO(zlib.compress(rosReq._buff, _GZIP_LVL))
            else:
                req = StringIO(rosReq._buff)

            with self._lock:
                self._pending.add(event)

            self._call(req, self._rceCB, event)

            with self._lock:
                self._pending.discard(event)

            rosResp = event.get()

            if not isinstance(rosResp, genpy.message.Message):
                raise rospy.ServiceException('Service call interrupted.')

            return rosResp

        def _rceCB(self, resp, event):
            """ Internally used method to send received message to the ROS
                Service as response.
            """
            rosResp = rospy.AnyMsg()

            if _GZIP_LVL:
                rosResp._buff = zlib.decompress(resp.getvalue())
            else:
                rosResp._buff = resp.getvalue()

            event.set(rosResp)

        def _start(self):
            self._service = rospy.Service(self._addr, self._srvCls, self._rosCB)
            print("Local ROS Service on address '{0}' is "
                  'up.'.format(self._addr))

        def _stop(self):
            self._service.shutdown()
            self._service = None
            print("Local ROS Service on address '{0}' is "
                  'down.'.format(self._addr))

            with self._lock:
                for event in self._pending:
                    event.set(None)

        def __del__(self):
            """ Finalize the Service.
            """
            if self._service:
                self._stop()

            assert len(self._pending) == 0

            super(ROSServiceClient, self).__del__()


    class ROSServiceProvider(_ServiceProvider):
        """ Representation of a Service Provider Interface using ROS.
        """
        def __init__(self, conn, iTag, srvType, addr):
            """ Initialize the Service Client.
            """
            self._addr = addr

            args = srvType.split('/')

            if len(args) != 2:
                raise ValueError('Service type is not valid. Has to be of the '
                                 'form pkg/srv, i.e. std_msgs/Int8.')

            self._srvCls = conn.loader.loadSrv(*args)
            self._srvCls._request_class = rospy.AnyMsg
            self._srvCls._response_class = rospy.AnyMsg

            super(ROSServiceProvider, self).__init__(conn, iTag, srvType,
                                                     self._rceCB, ())

        def _rceCB(self, req):
            """ Internally used method to send received message to the ROS
                Service as request.
            """
            rosReq = rospy.AnyMsg()

            if _GZIP_LVL:
                rosReq._buff = zlib.decompress(req.getvalue())
            else:
                rosReq._buff = req.getvalue()

            rospy.wait_for_service(self._addr, timeout=5)
            serviceFunc = rospy.ServiceProxy(self._addr, self._srvCls)
            rosResp = serviceFunc(rosReq)

            if _GZIP_LVL:
                resp = StringIO(zlib.compress(rosResp._buff, _GZIP_LVL))
            else:
                resp = StringIO(rosResp._buff)

            return resp
