#!/usr/bin/env python
# -*- coding: utf-8 -*-
#     
#     monitor.py
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
import os
import re
import tempfile
import shlex
import zlib
from threading import Event, Lock
from uuid import uuid4

try:
    from cStringIO import StringIO, InputType, OutputType
    from StringIO import StringIO as pyStringIO
    
    def _checkIsStringIO(obj):
        return isinstance(obj, (InputType, OutputType, pyStringIO))
except ImportError:
    from StringIO import StringIO
    
    def _checkIsStringIO(obj):
        return isinstance(obj, StringIO)

# ROS specific imports
import rospy
import genpy.message

# zope specific imports
from zope.interface import implements

# twisted specific imports
from twisted.python import log
from twisted.internet.protocol import ProcessProtocol

# Custom imports
from settings import GZIP_LVL
from errors import InternalError, InvalidRequest
from util.interfaces import verifyObject
from core.types import cmd as types
from core.interfaces import IEndpointInterfaceMonitor, \
    IEndpointInterfaceCommand, IEndpointConverterCommand
from util.loader import ResourceNotFound


class NodeMonitor(object):
    """ Class which is used to launch and monitor a node.
    """
    class _ROSNodeProtocol(ProcessProtocol):
        """ Protocol which is used to handle the ROS nodes.
        """
        def __init__(self, nodeMonitor):
            self._nodeMonitor = nodeMonitor
        
        def connectionMade(self):
            self._nodeMonitor.started(self)
        
        def processEnded(self, reason):
            self._nodeMonitor.stopped(reason.value.exitCode)
    
    
    _STOP_ESCALATION = [ ('INT', 15), ('TERM', 2), ('KILL', None) ]
    _RE_FIND = re.compile('\\$\\( *find +(?P<pkg>[a-zA-Z][a-zA-z0-9_]*) *\\)')
    _RE_ENV = re.compile('\\$\\( *env +(?P<var>[a-zA-Z][a-zA-z0-9_]*) *\\)')
    
    def __init__(self, manager, userID, node):
        """ Initialize the NodeMonitor instance.
            
            @param manager:     Manager instance used in this node which has
                                references to the ROS component loader and the
                                twisted reactor.
            
            @param userID:      User ID of the interface owner.
            @type  userID:      str
            
            @param node:        Node instance which should be monitored.
            @type  node:        Node
        """
        self._manager = manager
        self._userID = userID
        self._node = node
        self._name = ''
        
        self._process = None
        self._started = False
        self._running = False
        self._exitCode = None
        
        self._escalationLvl = 0
    
    @property
    def tag(self):
        """ Node tag to identify the node. """
        return self._node.tag
    
    def _replaceFind(self, match):
        """ Internally used method to replace found matches of _RE_FIND regular
            expression with corresponding package path.
        """
        path = self._manager.loader.findPkgPath(match.group('pkg'))
        
        if ' ' in path:
            return '"{0}"'.format(path)
        else:
            return path
    
    def _replaceEnv(self, match):
        """ Internally used method to replace found matches of _RE_ENV regular
            expression with corresponding environment variable.
        """
        try:
            return os.environ[match.group('var')]
        except KeyError:
            raise InvalidRequest('Can not find environment variable: '
                                 '{0}'.format(match.group('var')))
    
    def start(self):
        """ Launch the node.

            @raise:     InternalError if Node is already launched.
        """
        if self._started:
            raise InternalError('Can not launch an already running node.')
        
        pkg = self._node.pkg
        exe = self._node.exe
        
        self._name = '{0}/{1}'.format(pkg, exe)
        
        # Find and validate executable
        try:
            cmd = [self._manager.loader.findNode(pkg, exe)]
        except ResourceNotFound as e:
            raise InvalidRequest('Could not identify which node to launch: '
                                 '{0}'.format(e))
        
        # Add arguments
        args = self._RE_FIND.subn(self._replaceFind, self._node.args)[0]
        args = self._RE_ENV.subn(self._replaceEnv, args)[0]
        
        for char in '$;':
            if char in args:
                raise InvalidRequest('Argument can not contain special '
                                     "character '{0}'.".format(char))
        
        cmd += shlex.split(args)
        
        # Process name and namespace argument
        name = self._node.name
        namespace = self._node.namespace
        
        if name:
            cmd.append('__name:={0}'.format(name))
        else:
            name = exe
        
        if namespace:
            cmd.append('__ns:={0}'.format(namespace))
        
        self._cmd = cmd
        
        # Start node
        self._started = True
        log.msg('Start Node {0}/{1} [pkg: {2}, exe: '
                '{3}].'.format(namespace, name, pkg, exe))
        self._manager.reactor.spawnProcess(
            NodeMonitor._ROSNodeProtocol(self), cmd[0], cmd, env=os.environ)
    
    def started(self, process):
        """ Callback for ROSProcessProtocol to signal that the process has
            been launched.
        """
        self._process = process
        self._running = True
        self._manager.registerNode(self._userID, self)
    
    def isAlive(self):
        """ Check whether the node/process is alive.
        """
        if not self._started:
            return True
        
        return self._running
    
    def stopped(self, exitCode):
        """ Callback for ROSProcessProtocol to signal that the process has
            died.
        """
        self._manager.unregisterNode(self._userID, self)
        self._running = False
        self._process = None
        self._exitCode = exitCode
        
        if exitCode:
            log.msg('Node ({0}) terminated with exit code: '
                    '{1}\ncmd = {2}'.format(self._name, exitCode, self._cmd))
    
    def stop(self):
        """ Stop the node.
        """
        if not self._running:
            return
        
        if self._escalationLvl == 0:
            log.msg('Stop Node {0}/{1}.'.format(self._node.namespace,
                                                self._node.exe))
        escalation = self._STOP_ESCALATION[self._escalationLvl]
        
        self._process.transport.signalProcess(escalation[0])
        
        if escalation[1]:
            self._escalationLvl += 1
            self._manager.reactor.callLater(escalation[1], self.stop)


class _ParamMonitor(object):
    """ Base class which provides the basic functionalities to monitor a
        parameter.
    """
    def __init__(self, parameter):
        """ Add a parameter to the parameter server.
            
            @param parameter:   Parameter command describing parameter which
                                should be monitored.
            @type  parameter:   subclass of core.command._ParameterCommand
        """
        self._init(parameter.name, parameter.value)
    
    def _init(self, name, value):
        """ Internally used method by the constructor. Necessary for FileParam.
        """
        self._registered = False
        self._name = name
        
        try:
            if rospy.has_param(name):
                log.msg('Warning: Parameter already exists.')
            
            rospy.set_param(name, value)
            self._registered = True
        except rospy.ROSException as e:
            raise InternalError('ROS Parameter Server reported an error: '
                                '{0}'.format(e))
    
    @property
    def name(self):
        """ Name of the parameter. """
        return self._name
    
    def remove(self):
        """ Method which is used to remove the parameter from the parameter
            server.
        """
        if self._registered:
            try:
                rospy.delete_param(self._name)
            except rospy.ROSException:
                pass
            
            self._registered = False
    
    def __del__(self):
        """ Remove a parameter from the parameter server.
        """
        self.remove()


class IntMonitor(_ParamMonitor):
    """ Class which allows to handle an integer parameter.
    """
    IDENTIFIER = types.PARAM_INT


class StrMonitor(_ParamMonitor):
    """ Class which allows to handle a string parameter.
    """
    IDENTIFIER = types.PARAM_STR


class FloatMonitor(_ParamMonitor):
    """ Class which allows to handle a float parameter.
    """
    IDENTIFIER = types.PARAM_FLOAT


class BoolMonitor(_ParamMonitor):
    """ Class which allows to handle a bool parameter.
    """
    IDENTIFIER = types.PARAM_BOOL


class FileMonitor(_ParamMonitor):
    """ Class which allows to handle a file parameter.
    """
    IDENTIFIER = types.PARAM_FILE
    
    def __init__(self, parameter):
        """ Add a parameter to the parameter server.
        """
        self._fileCreated = False
        
        (f, fileName) = FileMonitor._mktempfile()
        f.write(parameter.value)
        f.close()

        if not os.path.isfile(fileName):
            raise InternalError('Could not find file.')

        self._fileCreated = True
        
        self._init(parameter.name, fileName)
    
    @staticmethod
    def _mktempfile(directory=None):
        """ Wrapper around tempfile.mkstemp function such that a file object
            is returned and not an integer. Make sure to close the file object
            again.
        """
        (fd, fname) = tempfile.mkstemp(dir=directory)
        return (os.fdopen(fd, 'wb'), fname)
    
    def __del__(self):
        """ Remove a parameter from the parameter server.
        """
        if self._fileCreated:
            try:
                os.remove(self._value)
            except OSError:
                pass
        
        super(FileMonitor, self).__del__()


class _EndpointInterfaceMonitor(object):
    """ Abstract base class which represents an endpoint interface monitor.
    """
    implements(IEndpointInterfaceMonitor)
    
    def __init__(self, userID, tag):
        """ Initialize the basic endpoint interface. 
        """
        self._userID = userID
        self._tag = tag
        self._conn = set() # All connections: tuple (commID, interfaceTag)
        self._ready = False
    
    @property
    def tag(self):
        """ Tag which is used to identify the endpoint interface which is
            monitored.
        """
        return self._tag
    
    def addConnection(self, commID, tag):
        """ Add a connection to another interface to this interface.
            
            @param commID:  Communication ID of other interface.
            @type  commID:  str
            
            @param tag:     Name of other interface.
            @type  tag:     str
        """
        conn = (commID, tag)
        
        if conn in self._conn:
            raise InternalError('The same connection is already registered '
                                'in this interface.')
        
        self._conn.add(conn)
    
    def removeConnection(self, commID, tag):
        """ Remove a connection to another interface from this interface.
            
            @param commID:  Communication ID of other interface.
            @type  commID:  str
            
            @param tag:     Name of other interface.
            @type  tag:     str
        """
        conn = (commID, tag)
        
        if conn not in self._conn:
            raise InternalError('The connection is not registered in '
                                'this interface.')
        
        self._conn.remove(conn)
    
    def _start(self):
        """ This method should be overwritten to implement necessary start
            up procedures.
        """
        pass

    def start(self):
        """ This method is used to setup the interface.

            Don't overwrite this method; instead overwrite the method _start.

            @raise:     errors.InternalError if the interface can not be
                        started.
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
    
    def _send(self, msg, msgID, conn):
        """ This method should be overwritten to implement the send
            functionality.
            
            @param msg:     ROS message which should be sent as a serialized
                            string.
            @type  msg:     str
            
            @param msgID:   Identifier which is used to match request and
                            response.
            @type  msgID:   str
            
            @param conn:    Tuple which is used as a destination for a possible
                            response and which is of the form: (commID, tag)
            @type  conn:    None / (str, str)
        """
        raise InternalError('Interface does not support sending of a message.')

    def send(self, msg, msgID, commID, sender):
        """ This method is used to send a message to the endpoint.

            Don't overwrite this method; instead overwrite the method _send.
            If the interface does not overwrite the method _send, it is assumed
            that the interface does not support this action and an
            InternalError is raised.

            @param msg:     Message which should be sent in serialized form.
            @type  msg:     str
            
            @param msgID:   Message ID which is used to match response message.
            @type  msgID:   str
            
            @param commID:  CommID from where the message originated.
            @type  commID:  str
            
            @param sender:  Tag which is used to identify the sender interface.
            @type  sender:  str
        """
        if not self._ready:
            raise InternalError('Interface is not ready to send a message.')
        
        conn = (commID, sender)
        
        if conn not in self._conn:
            raise InvalidRequest('Interface "({0}, {1})" is not allowed to '
                                 'use the interface "{2}".'.format(
                                     commID, sender, self._tag))
        
        self._send(msg, msgID, conn)


class _InterfaceMonitor(_EndpointInterfaceMonitor):
    """ Base class which is used to handle and monitor a ROS interface.
    """
    def __init__(self, manager, userID, interface):
        """ Initialize the Interface monitor.
            
            @param manager:     Manager which is used in this node and
                                implements necessary callback.
            @type  manager:     core.manager._InterfaceManager
            
            @param userID:      User ID of the interface owner.
            @type  userID:      str
            
            @param interface:   Interface instance which describes interface
                                which should be monitored.
            @type  interface:   IEndpointInterface
            
            @raise:     util.interfaces.InterfaceError
        """
        verifyObject(IEndpointInterfaceCommand, interface)
        
        super(_InterfaceMonitor, self).__init__(userID, interface.tag)
        
        self._name = interface.name
        self._manager = manager
        self._ready = False


class ServiceMonitor(_InterfaceMonitor):
    """ Class which is used to handle and monitor a service interface.
    """
    IDENTIFIER = types.INTERFACE_SRV
    
    class ServiceTask(object):
        """ Data container for a single Task, which consists of a
            request and its corresponding result.
        """
        __slots__ = [ '_srv',
                      '_msg',
                      '_msgID',
                      '_dest' ]

        def __init__(self, srv, msg, msgID, dest):
            """ Initialize the Task.

                @param srv:     ServiceInterface to which this task belongs.
                @type  srv:     ServiceInterface

                @param msg:     Message which should be sent as a request
                                in its serialized form.
                @type  msg:     str
                
                @param msgID:   Message ID which is used to match response
                                messages.
                @type  msgID:   str
            
                @param dest:    Tuple of the form (commID, uid) to indicate
                                to which communication node this result
                                should be sent and a unique ID to identify
                                the request/response.
                @type  dest:    (str, str)
            """
            self._srv = srv
            self._msg = msg
            self._msgID = msgID
            self._dest = dest

        def run(self):
            """ This method contains the "task" and is the part which should
                be executed in a separate thread.
            """
            msg = rospy.AnyMsg()
            msg._buff = self._msg

            try:
                rospy.wait_for_service(self._srv._name, timeout=5)
                serviceFunc = rospy.ServiceProxy(self._srv._name,
                                                 self._srv._srvCls)
                response = serviceFunc(msg)
            except rospy.ROSInterruptException:
                return
            except Exception:
                raise
            
            commID, sender = self._dest
            self._srv._manager.send(self._srv._userID, sender, commID,
                                    self._srv._tag, response._buff,
                                    self._msgID)

    def __init__(self, manager, userID, interface):
        super(ServiceMonitor, self).__init__(manager, userID, interface)
        
        args = interface.msgType.split('/')
        
        if len(args) != 2:
            raise InvalidRequest('Service type is not valid. Has to be of the '
                                 'form pkg/msg, i.e. std_msgs/Int8.')
        
        self._srvCls = manager.loader.loadSrv(*args)
        self._srvCls._request_class = rospy.AnyMsg
        self._srvCls._response_class = rospy.AnyMsg

    __init__.__doc__ = _InterfaceMonitor.__init__.__doc__
    
    def _send(self, msg, msgID, dest):
        task = ServiceMonitor.ServiceTask(self, msg, msgID, dest)
        self._manager.reactor.callInThread(task.run)


class ServiceProviderMonitor(_InterfaceMonitor):
    """ Represents a service-provider interface for a node.
    """
    IDENTIFIER = types.INTERFACE_PRO
    
    def __init__(self, manager, userID, interface):
        super(ServiceProviderMonitor, self).__init__(manager, userID,
                                                     interface)
        
        args = interface.msgType.split('/')
        
        if len(args) != 2:
            raise InvalidRequest('Service type is not valid. Has to be of the '
                                 'form pkg/msg, i.e. std_msgs/Int8.')
        
        self._service = None
        self._pendingLock = Lock()
        self._pending = {}
        
        self._srvCls = manager.loader.loadSrv(*args)
        self._srvCls._request_class = rospy.AnyMsg
        self._srvCls._response_class = rospy.AnyMsg
    
    def _start(self):
        self._service = rospy.Service(self._name, self._srvCls, self._callback)
    
    def _stop(self):
        self._service.shutdown()
        self._service = None
        
        with self._pendingLock:
            for event in self._pending.itervalues():
                event.set()
            
            self._pending = {}
    
    def _send(self, msgData, msgID, _):
        msg = rospy.AnyMsg()
        msg._buff = msgData
        
        with self._pendingLock:
            event = self._pending[msgID]
            self._pending[msgID] = msg
        
        event.set()
    
    def _callback(self, request):
        msgID = uuid4().hex
        event = Event()
        
        with self._pendingLock:
            self._pending[msgID] = event
        
        if len(self._conn) > 1:
            raise InternalError('Can not connect more than one interface to a '
                                'service-provider interface.')
        
        for commID, sender in self._conn:
            self._manager.send(self._userID, sender, commID, self._tag,
                               request._buff, msgID)
        
        event.wait()
        
        with self._pendingLock:
            response = self._pending.pop(msgID, None)
        
        if not isinstance(response, genpy.message.Message):
            raise Exception('Interrupted.') # TODO: Change exception?
        
        return response
    
    def addConnection(self, commID, tag):
        if self._conn:
            raise InternalError('There is already a connection registered for '
                                'this service-provider.')
        
        super(ServiceProviderMonitor, self).addConnection(commID, tag)
    
    addConnection.__doc__ = _InterfaceMonitor.addConnection.__doc__


class PublisherMonitor(_InterfaceMonitor):
    """ Represents a publisher interface for a node.
    """
    IDENTIFIER = types.INTERFACE_PUB
    
    def __init__(self, manager, userID, interface):
        super(PublisherMonitor, self).__init__(manager, userID, interface)
        
        args = interface.msgType.split('/')
        
        if len(args) != 2:
            raise InvalidRequest('Message type is not valid. Has to be of the '
                                 'form pkg/msg, i.e. std_msgs/Int8.')
        
        self._msgCls = manager.loader.loadMsg(*args)

    __init__.__doc__ = _InterfaceMonitor.__init__.__doc__
    
    def _start(self):
        self._publisher = rospy.Publisher(self._name, self._msgCls, latch=True)

    def _stop(self):
        self._publisher.unregister()
        self._publisher = None

    def _send(self, msgData, msgID, _):
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
    IDENTIFIER = types.INTERFACE_SUB

    def _start(self):
        self._subscriber = rospy.Subscriber(self._name, rospy.AnyMsg,
                                            self._callback)

    def _stop(self):
        self._subscriber.unregister()
        self._subscriber = None

    def _callback(self, msg):
        for commID, sender in self._conn:
            self._manager.send(self._userID, sender, commID, self._tag,
                               msg._buff, uuid4().hex)


class _ConverterMonitor(_EndpointInterfaceMonitor):
    """ Base class which is used to handle and monitor a converter.
    """
    def __init__(self, manager, userID, converter):
        """ Initialize the Converter monitor.
            
            @param manager:     Manager which is used in this node and
                                implements necessary callback.
            @type  manager:     core.manager._RobotManager
            
            @param userID:      User ID of the interface owner.
            @type  userID:      str
            
            @param converter:   Converter instance which describes converter
                                which should be monitored.
            @type  converter:   core.interfaces.IEndpointConverterCommand
            
            @raise:     util.interfaces.InterfaceError
        """
        verifyObject(IEndpointConverterCommand, converter)
        
        super(_ConverterMonitor, self).__init__(userID, converter.tag)
        
        self._msgType = converter.msgType
        self._robotID = converter.endpointID
        self._converter = manager.converter
        self._manager = manager
        
        self._inputMsgCls = None
        self._outputMsgCls = None
        
        self._loadClass(manager.loader)
    
    def _loadClass(self, loader):
        """ This method is used to load the necessary ROS message class
            definition. And is called as last step in the constructor.
            
            This method has to be overwritten!
            
            References to the loaded classes have to be stored in the instance
            attributes '_inputMsgCls' and '_outputMsgCls'.
            If there is no reference stored to an attribute it is assumed that
            the converter can not convert corresponding messages and and error
            is raised.
        """
        raise NotImplementedError('The method "_loadClass" has to '
                                  'be implemented.')
    
    def receive(self, msgType, msgID, msg):
        """ Convert a JSON encoded message into a ROS message.
            
            @param msgType:     Message type of the received message.
            @type  msgType:     str
            
            @param msgID:       Identifier which is used to match request /
                                response message.
            @type  msgID:       str
            
            @param msg:         JSON compatible message which should be
                                processed.
            @type  msg:         dict
            
            @raise:         errors.InternalError if the Converter can not
                            convert incoming messages or
                            erors.InvalidRequest if the message can not be
                            converted the message due to an invalid format
        """
        if not self._inputMsgCls:
            raise InternalError('This converter can not convert '
                                'incoming messages.')
        
        if msgType != self._msgType:
            raise InvalidRequest('Sent message type does not match the used '
                                 'message type for this interface.')
        
        try:
            msg = self._converter.decode(self._inputMsgCls, msg)
        except (TypeError, ValueError) as e:
            raise InvalidRequest(str(e))
        
        buf = StringIO()
        msg.serialize(buf)
        msg = buf.getvalue()
        
        for commID, target in self._conn:
            self._manager.send(self._userID, target, commID, self._tag, msg,
                               msgID)
    
    def _send(self, msg, msgID, _):
        """ Convert a ROS message into a JSON encoded message.
            
            @param msg:     Received ROS message in serialized form.
            @type  msg:     str
            
            @param msgID:   Unique ID to identify the message.
            @type  msgID:   str
            
            @raise:         errors.InternalError if the Converter can not
                            convert outgoing messages or
                            erors.InvalidRequest if the message can not be
                            converted the message due to an invalid format
        """
        if not self._outputMsgCls:
            raise InternalError('This converter can not convert '
                                'outgoing messages.')
        
        rosMsg = self._outputMsgCls()
        rosMsg.deserialize(msg)
        
        try:
            jsonMsg = self._converter.encode(rosMsg)
        except (TypeError, ValueError) as e:
            raise InvalidRequest(str(e))
        
        self._manager.sendToClient(self._userID, self._robotID,
            { 'dest' : self._robotID, 'orig' : self._tag,
              'type' : self._msgType, 'msgID' : msgID, 'msg' : jsonMsg })


class ServiceConverter(_ConverterMonitor):
    """ Class which is used to handle and monitor a Service Converter.
    """
    IDENTIFIER = types.CONVERTER_SRV
    
    def _loadClass(self, loader):
        args = self._msgType.split('/')
        
        if len(args) != 2:
            raise InvalidRequest('srv type is not valid. Has to be of the '
                                 'from pkg/msg, i.e. std_msgs/Int8.')
        
        srvCls = loader.loadSrv(*args)
        self._inputMsgCls = srvCls._response_class
        self._outputMsgCls = srvCls._request_class


class ServiceProviderConverter(_ConverterMonitor):
    """ Class which is used to handle and monitor a Service-Provider Converter.
    """
    IDENTIFIER = types.CONVERTER_PRO
    
    def _loadClass(self, loader):
        args = self._msgType.split('/')
        
        if len(args) != 2:
            raise InvalidRequest('srv type is not valid. Has to be of the '
                                 'from pkg/msg, i.e. std_msgs/Int8.')
        
        srvCls = loader.loadSrv(*args)
        self._inputMsgCls = srvCls._request_class
        self._outputMsgCls = srvCls._response_class


class PublisherConverter(_ConverterMonitor):
    """ Class which is used to handle and monitor a Publisher Converter.
    """
    IDENTIFIER = types.CONVERTER_PUB
    
    def _loadClass(self, loader):
        args = self._msgType.split('/')
        
        if len(args) != 2:
            raise InvalidRequest('msg type is not valid. Has to be of the '
                                 'from pkg/msg, i.e. std_msgs/Int8.')
        
        self._outputMsgCls = loader.loadMsg(*args)


class SubscriberConverter(_ConverterMonitor):
    """ Class which is used to handle and monitor a Subscriber Converter.
    """
    IDENTIFIER = types.CONVERTER_PUB
    
    def _loadClass(self, loader):
        args = self._msgType.split('/')
        
        if len(args) != 2:
            raise InvalidRequest('msg type is not valid. Has to be of the '
                                 'from pkg/msg, i.e. std_msgs/Int8.')
        
        self._inputMsgCls = loader.loadMsg(*args)


class _ForwarderMonitor(_EndpointInterfaceMonitor):
    """ Base class which is used to handle and monitor a forwarder.
    """
    def __init__(self, manager, userID, forwarder):
        """ Initialize the Forwarder monitor.
            
            @param manager:     Manager which is used in this node and
                                implements necessary callback.
            @type  manager:     core.manager._RobotManager
            
            @param userID:      User ID of the interface owner.
            @type  userID:      str
            
            @param forwarder:   Forwarder instance which describes forwarder
                                which should be monitored.
            @type  forwarder:   core.interfaces.IEndpointConverterCommand
            
            @raise:     util.interfaces.InterfaceError
        """
        verifyObject(IEndpointConverterCommand, forwarder)
        
        super(_ForwarderMonitor, self).__init__(userID, forwarder.tag)
        
        self._msgType = forwarder.msgType
        self._robotID = forwarder.endpointID
        self._manager = manager
    
    def receive(self, msgType, msgID, msg):
        """ Convert a JSON encoded message into a ROS message.
            
            @param msgType:     Message type of the received message.
            @type  msgType:     str
            
            @param msgID:       Identifier which is used to match request /
                                response message.
            @type  msgID:       str
            
            @param msg:         JSON compatible message which should be
                                processed.
            @type  msg:         dict
            
            @raise:         erors.InvalidRequest if the message can not be
                            forwarded the message due to an invalid format
        """
        if msgType != self._msgType:
            raise InvalidRequest('Sent message type does not match the used '
                                 'message type for this interface.')
        
        if not _checkIsStringIO(msg):
            raise InvalidRequest('Sent message is not a binary message.')
        
        msg = zlib.decompress(msg.getvalue())
        
        for commID, target in self._conn:
            self._manager.send(self._userID, target, commID, self._tag, msg,
                               msgID)
    
    def _send(self, msg, msgID, _):
        """ Convert a ROS message into a JSON encoded message.
            
            @param msg:     Received ROS message in serialized form.
            @type  msg:     str
            
            @param msgID:   Unique ID to identify the message.
            @type  msgID:   str
        """
        self._manager.sendToClient(self._userID, self._robotID,
            { 'dest' : self._robotID, 'orig' : self._tag,
              'type' : self._msgType, 'msgID' : msgID,
              'msg' : StringIO(zlib.compress(msg, GZIP_LVL)) })


class ServiceForwarder(_ForwarderMonitor):
    """ Class which is used to handle and monitor a Service Converter.
    """
    IDENTIFIER = types.FORWARDER_SRV


class ServiceProviderForwarder(_ForwarderMonitor):
    """ Class which is used to handle and monitor a Service-Provider Converter.
    """
    IDENTIFIER = types.FORWARDER_PRO


class PublisherForwarder(_ForwarderMonitor):
    """ Class which is used to handle and monitor a Publisher Converter.
    """
    IDENTIFIER = types.FORWARDER_PUB


class SubscriberForwarder(_ForwarderMonitor):
    """ Class which is used to handle and monitor a Subscriber Converter.
    """
    IDENTIFIER = types.FORWARDER_SUB
