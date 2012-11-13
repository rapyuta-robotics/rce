#!/usr/bin/env python
# -*- coding: utf-8 -*-
#     
#     proxy.py
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

# ROS specific imports
import rosgraph.names

# zope specific imports
from zope.interface import implements

# twisted specific imports
from twisted.python import log
from twisted.internet.defer import Deferred, DeferredList

# Custom imports
import settings
from errors import InternalError, InvalidRequest
from util.interfaces import verifyObject
from core.interfaces import IEndpointProxy, IEndpointControl, IRobotControl, \
    INodeControl, IParameterControl, IContainerControl
from core.command import NodeCommand, ContainerCommand, RobotCommand, \
    ParameterCommand, ArrayCommand, FileCommand, \
    ServiceInterfaceCommand, ServiceProviderInterfaceCommand, \
    PublisherInterfaceCommand, SubscriberInterfaceCommand, \
    ServiceConverterCommand, ServiceProviderConverterCommand, \
    PublisherConverterCommand, SubscriberConverterCommand, \
    ServiceForwarderCommand, ServiceProviderForwarderCommand, \
    PublisherForwarderCommand, SubscriberForwarderCommand


class _Status(object):
    """ Base class for status objects.
    """
    INIT = 0
    
    def __init__(self):
        """ Initialize the status instance.
        """
        self._status = self.INIT
        self._failed = False
    
    @property
    def status(self):
        """ Status as integer. """
        return self._status
    
    def setStatus(self, new, failed=False):
        """ Advance the status to 'new'. If flag failed is set the status can
            not be changed anymore.
        """
        if self._failed:
            return
        
        if new <= self._status:
            raise InternalError('Can not set status to a predecessor.')
        
        self._status = new
        self._failed = failed
    
    def setFailed(self):
        """ Set the current status to failed. If flag failed is set the status
            can not be changed anymore.
        """
        self._failed = True


class _InterfaceImpl(object):
    """ Contains interface type specific stuff.
    """
    COMMAND = None
    
    @classmethod
    def createCommand(cls, tag, msgType, endpointID, addr):
        return cls.COMMAND(tag, msgType, endpointID, addr)


class _ConverterImpl(object):
    """ Contains interface type specific stuff.
    """
    COMMAND = None
    
    @classmethod
    def createCommand(cls, tag, msgType, endpointID, _):
        return cls.COMMAND(tag, msgType, endpointID, '')
    

class _ServiceImpl(object):
    """ Contains interface type specific stuff.
    """
    NUMBER_OF_CONNECTIONS = 1
    
    @staticmethod
    def validConnection(otherCls):
        return issubclass(otherCls, _ServiceProviderImpl)


class _ServiceProviderImpl(object):
    """ Contains interface type specific stuff.
    """
    NUMBER_OF_CONNECTIONS = 1
    
    @staticmethod
    def validConnection(otherCls):
        return issubclass(otherCls, _ServiceImpl)


class _PublisherImpl(object):
    """ Contains interface type specific stuff.
    """
    NUMBER_OF_CONNECTIONS = None
    
    @staticmethod
    def validConnection(otherCls):
        return issubclass(otherCls, _SubscriberImpl)


class _SubscriberImpl(object):
    """ Contains interface type specific stuff.
    """
    NUMBER_OF_CONNECTIONS = None
    
    @staticmethod
    def validConnection(otherCls):
        return issubclass(otherCls, _PublisherImpl)


class _ServiceInterface(_InterfaceImpl, _ServiceImpl):
    """ Contains interface specific stuff.
    """
    COMMAND = ServiceInterfaceCommand


class _ServiceProviderInterface(_InterfaceImpl, _ServiceProviderImpl):
    """ Contains interface specific stuff.
    """
    COMMAND = ServiceProviderInterfaceCommand
    

class _PublisherInterface(_InterfaceImpl, _PublisherImpl):
    """ Contains interface specific stuff.
    """
    COMMAND = PublisherInterfaceCommand


class _SubscriberInterface(_InterfaceImpl, _SubscriberImpl):
    """ Contains interface specific stuff.
    """
    COMMAND = SubscriberInterfaceCommand


class _ServiceConverter(_ConverterImpl, _ServiceImpl):
    """ Contains interface specific stuff.
    """
    COMMAND = ServiceConverterCommand


class _ServiceProviderConverter(_ConverterImpl, _ServiceProviderImpl):
    """ Contains interface specific stuff.
    """
    COMMAND = ServiceProviderConverterCommand


class _PublisherConverter(_ConverterImpl, _PublisherImpl):
    """ Contains interface specific stuff.
    """
    COMMAND = PublisherConverterCommand


class _SubscriberConverter(_ConverterImpl, _SubscriberImpl):
    """ Contains interface specific stuff.
    """
    COMMAND = SubscriberConverterCommand


class _ServiceForwarder(_ConverterImpl, _ServiceImpl):
    """ Contains interface specific stuff.
    """
    COMMAND = ServiceForwarderCommand


class _ServiceProviderForwarder(_ConverterImpl, _ServiceProviderImpl):
    """ Contains interface specific stuff.
    """
    COMMAND = ServiceProviderForwarderCommand


class _PublisherForwarder(_ConverterImpl, _PublisherImpl):
    """ Contains interface specific stuff.
    """
    COMMAND = PublisherForwarderCommand


class _SubscriberForwarder(_ConverterImpl, _SubscriberImpl):
    """ Contains interface specific stuff.
    """
    COMMAND = SubscriberForwarderCommand


class InterfaceProxy(object):
    """ Class which represents an interface. It is associated with an endpoint.
    """
    _MAP = { 'ServiceInterface'         : _ServiceInterface,
             'ServiceProviderInterface' : _ServiceProviderInterface,
             'PublisherInterface'       : _PublisherInterface,
             'SubscriberInterface'      : _SubscriberInterface,
             'ServiceConverter'         : _ServiceConverter,
             'ServiceProviderConverter' : _ServiceProviderConverter,
             'PublisherConverter'       : _PublisherConverter,
             'SubscriberConverter'      : _SubscriberConverter,
             'ServiceForwarder'         : _ServiceForwarder,
             'ServiceProviderForwarder' : _ServiceProviderForwarder,
             'PublisherForwarder'       : _PublisherForwarder,
             'SubscriberForwarder'      : _SubscriberForwarder }
    
    def __init__(self, endpoint, tag, iType, msgType, rosAddr, resp):
        """ Initialize the Interface.
            
            @param endpoint:    Endpoint to which this interface belongs.
            @type  endpoint:    core.interfaces.IEndpointProxy
            
            @param tag:         Tag which is used to identify this interface.
            @type  tag:         str
            
            @param iType:       Interface type; valid values are:
                                    service, publisher, subscriber
            @type  iType:       str
            
            @param msgType:     Message type of the form package/messageClass,
                                i.e. 'std_msgs/Int8'.
            @type  msgType:     str
            
            @param rosAddr:     Address which is used for the interface in the
                                ROS environment, i.e. '/namespace/interface';
                                only necessary if it is a ROS environment
                                interface.
            @type  rosAddr:     str
            
            @raise:     errors.InvalidRequest, util.interfaces.InterfaceError
        """
        verifyObject(IEndpointProxy, endpoint)
        
        if iType not in self._MAP:
            raise InvalidRequest('"{0}" is not a valid interface type.')
        
        self._endpoint = endpoint
        self._control = None
        self._tag = tag
        self._iType = self._MAP[iType]
        self._msgType = msgType
        self._rosAddr = rosAddr
        
        self._conn = set()
        
        endpoint.registerInterface(self, rosAddr)
        endpoint.owner.registerInterface(self)
    
    @property
    def tag(self):
        """ Tag used to identify this interface by the user. """
        return self._tag
    
    @property
    def commID(self):
        """ CommID of endpoint to which this interface belongs. """
        return self._endpoint.commID
    
    def registerControl(self, control):
        """ Register control for the communication with the endpoint.
            
            @param control: Control which should be registered for the
                            communication with the endpoint.
            @type  control: core.interfaces.IEndpointControl
            
            @raise:     errors.InternalError, util.interfaces.InterfaceError
        """
        if self._control:
            raise InternalError('There is already a control for the '
                                'communication registered.')
        
        verifyObject(IEndpointControl, control)
        self._control = control
    
    def validConnection(self, otherInterface):
        """ Check whether this interface can be connected to the other
            interface.
            
            @return:    True if it is possible; False otherwise.
        """
        return (self._msgType == otherInterface._msgType and
                self._iType.validConnection(otherInterface._iType))
    
    def registerConnection(self, interface, resp):
        """ Register a connection such that the other endpoint is allowed to
            communicate with this interface.
            
            @param interface:   Interface to which the connection should be
                                established.
            @type  interface:   core.interface.Interface
            
            @param resp:    Deferred whose callbacks can be used to give a
                            response to the request.
            @type  resp:    twisted::Deferred
        """
        if interface == self:
            raise InvalidRequest('Can not connect to itself.')
        
        if interface in self._conn:
            raise InvalidRequest('Can not create the same connection twice.')
        
        if (self._iType.NUMBER_OF_CONNECTIONS and
                self._iType.NUMBER_OF_CONNECTIONS <= len(self._conn)):
            raise InvalidRequest('Can not add more than {0} connections to '
                                 'this interface.'.format(
                                     self._iType.NUMBER_OF_CONNECTIONS))
        
        deferredList = []
        
        if not self._conn:
            deferredList.append(Deferred())
            
            # There are no connections to this interface so far; -> add it
            self._control.addInterface(
                self._iType.createCommand(self._tag, self._msgType,
                                          self._endpoint.uid, self._rosAddr),
                deferredList[0]
            )
        
        if deferredList:
            deferredList.append(Deferred())
            resp.chainDeferred(DeferredList(deferredList))
            resp = deferredList[-1]
        
        self._control.registerConnection(self._tag, interface.commID,
                                         interface.tag, resp)
        self._conn.add(interface)
    
    def unregisterConnection(self, interface, resp):
        """ Unregister a connection such that the other endpoint is no longer
            allowed to communicate with this interface.
            
            @param interface:   Interface to which the connection should be
                                destroyed.
            @type  interface:   core.interface.Interface
            
            @param resp:    Deferred whose callbacks can be used to give a
                            response to the request.
            @type  resp:    twisted::Deferred
        """
        if interface not in self._conn:
            raise InvalidRequest('Can not remove a connection '
                                 'which is not here.')
        
        self._conn.remove(interface)
        
        if not self._conn:
            # There are no more references to this interface; -> remove it
            self._control.removeInterface(self._tag, resp)
        else:
            self._control.unregisterConnection(self._tag, interface.commID,
                                               interface.tag, resp)
    
    def delete(self, resp=None):
        """ Removes the interface.
            
            Make sure to call this method, because this method makes sure that
            all necessary references are removed.
            
            Once this method is called this monitor can no longer be used.
            
            @param resp:    Deferred whose callbacks can be used to give a
                            response to the request.
            @type  resp:    twisted::Deferred
        """
        self._endpoint.owner.unregisterInterface(self, self._rosAddr)
        self._endpoint.unregisterInterface(self)
        
        if self._conn:
            deferredList = [Deferred()]
            
            for interface in self._conn:
                deferredList.append(Deferred())
                interface.unregisterConnection(self, deferredList[-1])
            
            self._control.removeInterface(self._tag, deferredList[0])
            
            if resp:
                resp.chainDeferred(DeferredList(deferredList))
        
        self._endpoint = None


class _EndpointProxy(object):
    """ Base class which provides the necessary methods for all endpoint
        proxies.
    """
    implements(IEndpointProxy)
    
    def __init__(self, user, uid, commID, control):
        """ Initialize the endpoint monitor.
            
            @param user:        User to which this endpoint belongs.
            @type  user:        core.user.User
            
            @param uid:         Identifier of this endpoint.
            @type  uid:         str
            
            @param commID:      Communication ID of this endpoint.
            @type  commID:      str
            
            @param control:     Control which is used to send the commands to
                                the endpoint.
            @type  control:     core.interfaces.IEndpointControl
        """
        verifyObject(IEndpointControl, control)
        
        self._user = user
        self._uid = uid
        self._commID = commID
        self._control = control
        self._interfaces = set()
        self._interfaceAddr = set()
    
    @property
    def owner(self):
        """ Owner (type: core.user.User) of this endpoint. """
        return self._user
    
    @property
    def uid(self):
        """ Identifier of this endpoint. """
        return self._uid
    
    @property
    def commID(self):
        """ Communication ID of this endpoint. """
        return self._commID
    
    def registerInterface(self, interface, addr=''):
        """ Register a new interface with this endpoint.
            
            @param interface:   Interface instance which should be registered.
            @type  interface:   core.interface.Interface
            
            @param addr:        ROS address of the interface, if it has one.
            @type  addr:        str
        """
        if addr and addr in self._interfaceAddr:
            raise InvalidRequest('ROS address for interface is already in '
                                 'use.')
        
        if interface in self._interfaces:
            raise InternalError('There exists already an interface '
                                'with the same tag.')
        
        interface.registerControl(self._control)
        self._interfaces.add(interface)
        
        if addr:
            self._interfaceAddr.add(addr)
    
    def unregisterInterface(self, interface, addr=''):
        """ Unregister an interface with this endpoint.
            
            @param interface:   Interface instance which should be
                                unregistered.
            @type  interface:   core.interface.Interface
            
            @param addr:        ROS address of the interface, if it has one.
            @type  addr:        str
        """
        if interface not in self._interfaces:
            raise InternalError('Tried to unregister a non existing '
                                'interface.')
        
        self._interfaces.remove(interface)
        
        if addr:
            self._interfaceAddr.discard(addr)
    
    def delete(self):
        """ Removes the proxy, i.e. the represented endpoint.
            
            Make sure to call this method, because this method makes sure that
            all necessary references are removed.
            
            Once this method is called this monitor can no longer be used.
        """
        for interface in self._interfaces.copy():
            interface.delete()
        
        self._user = None
        self._control = None


class _RobotStatus(_Status):
    """ Class which is used to keep track of the status of a robot.
    """
    CREATE = INIT = 0
    CREATED = 1
    DESTROY = 2
    DESTROYED = 3


class RobotProxy(_EndpointProxy):
    """ Class which is used to keep track of the status of a Robot.
    """
    def __init__(self, user, robotID, commID, key, control):
        """ Initialize the Robot proxy.
            
            @param user:        User to which this robot belongs.
            @type  user:        core.user.User
            
            @param robotID:     RobotID which is used to identify the robot.
            @type  robotID:     str
            
            @param commID:      Communication ID of this robot.
            @type  commID:      str
            
            @param key:         Key which should be used to verify connection
                                from represented robot.
            @type  key:         str
                                    
            @param control:     Control which is used to communicate with the
                                robot.
            @type  control:     core.interfaces.IRobotControl
        """
        verifyObject(IRobotControl, control)
        
        super(RobotProxy, self).__init__(user, robotID, commID, control)
        
        self._status = _RobotStatus()
        
        resp = Deferred()
        
        def cb(msg):
            self._status.setStatus(_RobotStatus.CREATED)
            return msg
        
        def eb(e):
            self._status.setFailed()
            return e
        
        resp.addCallbacks(cb, eb)
            
        control.createRobot(RobotCommand(robotID, key), resp)
    
    def delete(self):
        """ Removes the proxy, i.e. the represented robot.
            
            Make sure to call this method, because this method makes sure that
            all necessary references are removed.
            
            Once this method is called this monitor can no longer be used.
        """
        self._status.setStatus(_RobotStatus.DESTROY)
        
        resp = Deferred()
        
        def cb(msg):
            self._status.setStatus(_RobotStatus.DESTROYED)
            return msg
        
        def eb(e):
            self._status.setFailed()
            return e
        
        resp.addCallbacks(cb, eb)
        
        self._control.destroyRobot(self._uid, resp)
        
        super(RobotProxy, self).delete()
        

class _ProcessStatus(_Status):
    """ Class which is used to keep track of the status of a process.
    """
    STARTUP = INIT = 0
    RUNNING = 1
    STOPPING = 2
    STOPPED = 3


class _ROSEnvProxy(_EndpointProxy):
    """ Class which is used to keep track of the status of a ROS environment.
    """
    def __init__(self, user, tag, commID, control):
        """ Initialize the ROS environment proxy.
            
            @param user:        User instance to which this ROS environment
                                belongs.
            @type  user:        core.user.User
            
            @param tag:         Tag of this environment.
            @type  tag:         str
            
            @param commID:      CommID of the ROS environment.
            @type  commID:      str
             
            @param control:     Control which is used to communicate with the
                                ROS environment.
            @type  control:     core.interfaces.IROSEnvControl
        """
        verifyObject(INodeControl, control)
        verifyObject(IParameterControl, control)
        
        super(_ROSEnvProxy, self).__init__(user, tag, commID, control)
        
        self._nodes = {}
    
    def addNode(self, tag, pkg, exe, args, name, namespace, resp):
        """ Add a node to the monitored ROS environment.
            
            @param tag:         Tag which is used to identify the ROS node
                                which should added.
            @type  tag:         str

            @param pkg:         Package name where the node can be found.
            @type  pkg:         str

            @param exe:         Name of executable which should be launched.
            @type  exe:         str
            
            @param args:        Arguments which should be used for the launch.
            @type  args:        str
            
            @param name:        Name of the node under which it should be
                                launched.
            @type  name:        str
            
            @param namespace:   Namespace in which the node should use be
                                launched.
            @type  namespace:   str
            
            @param resp:        Deferred whose callbacks can be used to give a
                                response to the request.
            @type  resp:        twisted::Deferred
            
            @raise:             errors.InvalidRequest
        """
        if tag in self._nodes:
            raise InvalidRequest('There exists already a node with the same '
                                 'tag.')
        
        # First make sure, that the received strings are str objects
        # and not unicode objects
        try:
            pkg = self._castStr(pkg, 'package')
            exe = self._castStr(exe, 'executable')
            name = self._castStr(name, 'name')
            namespace = self._castStr(namespace, 'namespace')
        except ValueError as e:
            raise InvalidRequest(str(e))
        
        # Validate the namespace
        if not rosgraph.names.is_legal_name(namespace):
            raise InvalidRequest('The namespace "{0}" is not '
                                 'valid.'.format(namespace))
        
        if settings.DEBUG:
            nodeDesc = '"{0}/{1}" (tag: "{2}")'.format(pkg, exe, tag)
            log.msg('Start node {0} in ROS environment '
                    '"{1}".'.format(nodeDesc, self._commID))
            
            def cb(msg):
                log.msg('Node {0} successfully started in ROS environment '
                        '"{1}".'.format(nodeDesc, self._commID))
                return msg
            
            def eb(e):
                log.msg('Node {0} could not be started in ROS environment '
                        '"{1}":\n{2}'.format(nodeDesc, self._commID,
                                             e.getErrorMessage()))
                return e
            
            resp.addCallbacks(cb, eb)
        
        status = _ProcessStatus()
        
        def statusEB(e):
            status.setFailed()
            return e
        
        resp.addErrback(statusEB)
        
        # Add the node
        self._nodes[tag] = status
        self._control.addNode(NodeCommand(tag, pkg, exe, args, name,
                                          namespace), resp)
    
    def removeNode(self, tag, resp):
        """ Remove a node from the monitored ROS environment.
            
            @param tag:     Tag which is used to identify the ROS node which
                            should be removed.
            @type  tag:     str
            
            @param resp:    Deferred whose callbacks can be used to give a
                            response to the request.
            @type  resp:    twisted::Deferred
        """
        if tag not in self._nodes:
            raise InvalidRequest('There does not exist a node with the given '
                                 'tag.')
        
        if settings.DEBUG:
            log.msg('Stop node (tag: "{0}") in ROS environment '
                    '"{1}".'.format(tag, self._commID))
            
            def cb(msg):
                log.msg('Node (tag: "{0}") successfully stopped in ROS '
                        'environment "{1}".'.format(tag, self._commID))
                return msg
            
            def eb(e):
                log.msg('Node (tag: "{0}") could not be stopped in ROS '
                        'environment "{1}":\n{2}'.format(tag, self._commID,
                                                         e.getErrorMessage()))
                return e
            
            resp.addCallbacks(cb, eb)
        
        status = self._nodes[tag]
        
        def statusEB(e):
            status.setFailed()
            return e
        
        resp.addErrback(statusEB)
        
        status.setStatus(_ProcessStatus.STOPPING)
        self._control.removeNode(tag, resp)
    
    def setNodeStatus(self, nTag, newStatus):
        """ Set the received status of the node process.
            
            @param nTag:        Node tag to identify the node process.
            @type  nTag:        str
            
            @param newStatus:   Flag which indicates the status of the node
                                process:
                                    True    Node has been started
                                    False   Node has been terminated
            @type  newStatus:   bool
        """
        status = self._nodes[nTag]
        
        if newStatus:
            status.setStatus(_ProcessStatus.RUNNING)
        elif status.status == _ProcessStatus.RUNNING:
            status.setFailed()
        else:
            status.setStatus(_ProcessStatus.STOPPED)
    
    def _castStr(self, value, desc=''):
        """ Internally used method to convert a value to a string.
        """
        if isinstance(value, str):
            return value
        elif isinstance(value, unicode):
            try:
                return str(value)
            except UnicodeEncodeError:
                if desc:
                    raise ValueError("The {0} '{1}' is not a"
                                     'valid string.'.format(desc, value))
                else:
                    raise ValueError('Value is not a valid string.')
            
        if desc:
            raise ValueError('Type of {0} is not a string.'.format(desc))
        else:
            raise ValueError('Value is not a valid string.')
        
    def _castBool(self, value):
        """ Internally used method to convert a value to a bool.
        """
        if isinstance(value, basestring):
            value = value.strip().lower()
            
            if value == 'true':
                return True
            elif value == 'false':
                return False
        else:
            try:
                return bool(int(value))
            except ValueError:
                pass
        
        raise ValueError('Value is not a valid bool.')
    
    def _cast(self, value, paramType):
        """ Internally used method to convert a value.
        """
        if paramType == 'int':
            return int(value)
        elif paramType == 'str':
            return self._castStr(value)
        elif paramType == 'float':
            return float(value)
        elif paramType == 'bool':
            return self._castBool(value)
        
        raise ValueError('Parameter type is invalid.')
    
    def addParameter(self, name, value, paramType, resp):
        """ Add a parameter to the parameter server in the monitored ROS
            environment.
            
            @param name:        Name of the parameter which should be added.
            @type  name:        str
            
            @param value:       Value of the parameter which should be added.
            @type  value:       Depends on @param paramType
            
            @param paramType:   Type of the parameter to add. Valid base types
                                are:
                                    int, str, float, bool
                                
                                The base types can be summarized to an array,
                                for example:
                                    [int, str, bool, str]
                                
                                Additionally, a special type is defined for
                                adding a file; this type can't be used in an
                                array:
                                    file
            @type  paramType:   str
            
            @param resp:        Deferred whose callbacks can be used to give a
                                response to the request.
            @type  resp:        twisted::Deferred
            
            @raise:             errors.InvalidRequest if the name is not a
                                valid ROS name.
        """
        if not rosgraph.names.is_legal_name(name):
            raise InvalidRequest('The name "{0}" is not valid.'.format(name))
        
        paramType = paramType.strip()
        
        try:
            if paramType == 'file':
                param = FileCommand(name, self._castStr(value))
            else:
                if paramType[0] == '[' and paramType[-1] == ']':
                    paramType = map(lambda x: x.strip(),
                                    paramType[1:-1].split(','))
                    cmd = ArrayCommand
                else:
                    value = [value]
                    paramType = [paramType]
                    cmd = ParameterCommand
            
                if len(value) != len(paramType):
                    raise InvalidRequest('Length of parameter specification '
                                         'does not match length of supplied '
                                         'values.')
                
                value = [self._cast(*pair) for pair in zip(value, paramType)]
                paramType = ''.join(p.upper()[0] for p in paramType)
                param = cmd(name, value, paramType)
        except ValueError as e:
            raise InvalidRequest(str(e))
        
        if settings.DEBUG:
            log.msg('Add parameter "{0}" to ROS environment '
                    '"{1}".'.format(name, self._commID))
            
            def cb(msg):
                log.msg('Parameter "{0}" successfully added to ROS environment'
                        ' "{1}".'.format(name, self._commID))
                return msg
            
            def eb(e):
                log.msg('Parameter "{0}" could not be added to ROS environment'
                        ' "{1}":\n{2}'.format(name, self._commID,
                                              e.getErrorMessage()))
                return e
            
            resp.addCallbacks(cb, eb)
        
        self._control.addParameter(param, resp)
    
    def removeParameter(self, name, resp):
        """ Remove a parameter from the parameter server in the monitored ROS
            environment.
            
            @param name:    Name of the parameter which should be removed.
            @type  name:    str
            
            @param resp:    Deferred whose callbacks can be used to give a
                            response to the request.
            @type  resp:    twisted::Deferred
        """
        if settings.DEBUG:
            log.msg('Remove parameter "{0}" from ROS environment '
                    '"{1}".'.format(name, self._commID))
            
            def cb(msg):
                log.msg('Parameter "{0}" successfully removed from ROS '
                        'environment "{1}".'.format(name, self._commID))
                return msg
            
            def eb(e):
                log.msg('Parameter "{0}" could not be removed from ROS '
                        'environment "{1}":\n{2}'.format(name, self._commID,
                                                         e.getErrorMessage()))
                return e
            
            resp.addCallbacks(cb, eb)
        
        self._control.removeParameter(name, resp)


class ContainerProxy(_ROSEnvProxy):
    """ Class which is used to keep track of the status of a container
        (containing a ROS environment).
    """
    def __init__(self, user, cTag, commID, control, resp):
        """ Initialize the Container proxy.
            
            @param user:        User instance to which this Container belongs.
            @type  user:        core.user.User
                                    
            @param control:     Control which is used to communicate with the
                                Container.
            @type  control:     core.interfaces.IContainerControl
            
            @param commID:      CommID of the ROS environment inside the
                                Container.
            @type  commID:      str
            
            @param cTag:        Container tag which is used by the user to
                                identify this Container.
            @type  cTag:        str
            
            @param resp:        Deferred whose callbacks can be used to give a
                                response to the request.
            @type  resp:        twisted::Deferred
        """
        verifyObject(IContainerControl, control)
        
        super(ContainerProxy, self).__init__(user, cTag, commID, control)
        
        def statusCB(msg):
            self._status.setStatus(_ProcessStatus.RUNNING)
            return msg
        
        def statusEB(e):
            self._status.setStatus(_ProcessStatus.RUNNING, True)
            return e
        
        resp.addCallbacks(statusCB, statusEB)
        
        if settings.DEBUG:
            log.msg('Start container "{0}".'.format(commID))
            
            def cb(msg):
                log.msg('Container "{0}" successfully started.'.format(commID))
                return msg
            
            def eb(e):
                log.msg('Container "{0}" could not be '
                        'started.'.format(commID, e.getErrorMessage()))
                return e
            
            resp.addCallbacks(cb, eb)
        
        self._status = _ProcessStatus()
        control.createContainer(ContainerCommand(cTag, commID), resp)
    
    def delete(self, resp=None):
        """ Removes the monitor, i.e. the represented container.
            
            Make sure to call this method, because this method makes sure
            that all necessary references are removed.
            
            Once this method is called this monitor can no longer be used.
            
            @param resp:    Deferred whose callbacks can be used to give a
                            response to the request.
            @type  resp:    twisted::Deferred
        """
        if not resp:
            resp = Deferred()
        
        def statusCB(msg):
            self._status.setStatus(_ProcessStatus.STOPPED)
            return msg
        
        def statusEB(e):
            self._status.setStatus(_ProcessStatus.STOPPED, True)
            return e
        
        resp.addCallbacks(statusCB, statusEB)
        
        if settings.DEBUG:
            log.msg('Stop container "{0}".'.format(self._commID))
            
            def cb(msg):
                log.msg('Container "{0}" successfully '
                        'stopped.'.format(self._commID))
                return msg
            
            def eb(e):
                log.msg('Container "{0}" could not be '
                        'stopped.'.format(self._commID, e.getErrorMessage()))
                return e
            
            resp.addCallbacks(cb, eb)
        
        self._status.setStatus(_ProcessStatus.STOPPING)
        self._control.destroyContainer(self._uid)
        
        super(ContainerProxy, self).delete()
