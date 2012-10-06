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

# Custom imports
from errors import InternalError, InvalidRequest
from util.interfaces import verifyObject
from core.interfaces import IEndpointProxy, IEndpointControl
from core.command import \
    ServiceInterfaceCommand, ServiceProviderInterfaceCommand, \
    PublisherInterfaceCommand, SubscriberInterfaceCommand, \
    ServiceConverterCommand, ServiceProviderConverterCommand, \
    PublisherConverterCommand, SubscriberConverterCommand, \
    ServiceForwarderCommand, ServiceProviderForwarderCommand, \
    PublisherForwarderCommand, SubscriberForwarderCommand


class _Interface(object):
    """ Contains interface type specific stuff.
    """
    COMMAND = None
    
    @classmethod
    def createContainer(cls, tag, msgType, endpointID, addr):
        return cls.COMMAND(tag, msgType, endpointID, addr)


class _Converter(object):
    """ Contains interface type specific stuff.
    """
    COMMAND = None
    
    @classmethod
    def createContainer(cls, tag, msgType, endpointID, _):
        return cls.COMMAND(tag, msgType, endpointID, '')
    

class _Service(object):
    """ Contains interface type specific stuff.
    """
    NUMBER_OF_CONNECTIONS = 1
    
    @staticmethod
    def validConnection(otherCls):
        return issubclass(otherCls, _ServiceProvider)


class _ServiceProvider(object):
    """ Contains interface type specific stuff.
    """
    NUMBER_OF_CONNECTIONS = 1
    
    @staticmethod
    def validConnection(otherCls):
        return issubclass(otherCls, _Service)


class _Publisher(object):
    """ Contains interface type specific stuff.
    """
    NUMBER_OF_CONNECTIONS = None
    
    @staticmethod
    def validConnection(otherCls):
        return issubclass(otherCls, _Subscriber)


class _Subscriber(object):
    """ Contains interface type specific stuff.
    """
    NUMBER_OF_CONNECTIONS = None
    
    @staticmethod
    def validConnection(otherCls):
        return issubclass(otherCls, _Publisher)


class _ServiceInterface(_Interface, _Service):
    """ Contains interface specific stuff.
    """
    COMMAND = ServiceInterfaceCommand


class _ServiceProviderInterface(_Interface, _ServiceProvider):
    """ Contains interface specific stuff.
    """
    COMMAND = ServiceProviderInterfaceCommand
    

class _PublisherInterface(_Interface, _Publisher):
    """ Contains interface specific stuff.
    """
    COMMAND = PublisherInterfaceCommand


class _SubscriberInterface(_Interface, _Subscriber):
    """ Contains interface specific stuff.
    """
    COMMAND = SubscriberInterfaceCommand


class _ServiceConverter(_Converter, _Service):
    """ Contains interface specific stuff.
    """
    COMMAND = ServiceConverterCommand


class _ServiceProviderConverter(_Converter, _ServiceProvider):
    """ Contains interface specific stuff.
    """
    COMMAND = ServiceProviderConverterCommand


class _PublisherConverter(_Converter, _Publisher):
    """ Contains interface specific stuff.
    """
    COMMAND = PublisherConverterCommand


class _SubscriberConverter(_Converter, _Subscriber):
    """ Contains interface specific stuff.
    """
    COMMAND = SubscriberConverterCommand


class _ServiceForwarder(_Converter, _Service):
    """ Contains interface specific stuff.
    """
    COMMAND = ServiceForwarderCommand


class _ServiceProviderForwarder(_Converter, _ServiceProvider):
    """ Contains interface specific stuff.
    """
    COMMAND = ServiceProviderForwarderCommand


class _PublisherForwarder(_Converter, _Publisher):
    """ Contains interface specific stuff.
    """
    COMMAND = PublisherForwarderCommand


class _SubscriberForwarder(_Converter, _Subscriber):
    """ Contains interface specific stuff.
    """
    COMMAND = SubscriberForwarderCommand


class Interface(object):
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
    
    def __init__(self, endpoint, tag, iType, msgType, rosAddr=None):
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
            @type  rosAddr:     str / None
            
            @raise:     errors.InvalidRequest, util.interfaces.InterfaceError
        """
        verifyObject(IEndpointProxy, endpoint)
        
        if iType not in Interface._MAP:
            raise InvalidRequest('"{0}" is not a valid interface type.')
        
        self._endpoint = endpoint
        self._control = None
        self._tag = tag
        self._iType = Interface._MAP[iType]
        self._msgType = msgType
        self._rosAddr = rosAddr
        
        self._conn = set()
        
        endpoint.registerInterface(self)
        endpoint.owner.registerInterface(self)
        
        if rosAddr:
            try:
                endpoint.reserveAddr(rosAddr)
            except ValueError:
                raise InvalidRequest('Another interface with the same ROS '
                                     'address already exists.')
    
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
    
    def registerConnection(self, interface):
        """ Register a connection such that the other endpoint is allowed to
            communicate with this interface.
            
            @param interface:   Interface to which the connection should be
                                established.
            @type  interface:   core.interface.Interface
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
        
        if not self._conn:
            # There are no connections to this interface so far; -> add it
            self._control.addInterface(self._iType.createContainer(
                self._tag, self._msgType, self._endpoint.uid, self._rosAddr))
        
        self._control.registerConnection(self._tag, interface.commID,
                                         interface.tag)
        self._conn.add(interface)
    
    def unregisterConnection(self, interface):
        """ Unregister a connection such that the other endpoint is no longer
            allowed to communicate with this interface.
            
            @param interface:   Interface to which the connection should be
                                destroyed.
            @type  interface:   core.interface.Interface
        """
        if interface not in self._conn:
            raise InvalidRequest('Can not remove a connection '
                                 'which is not here.')
        
        self._conn.remove(interface)
        
        if not self._conn:
            # There are no more references to this interface; -> remove it
            self._control.removeInterface(self._tag)
        else:
            self._control.unregisterConnection(
                self._tag, interface.commID, interface.tag)
    
    def delete(self):
        """ Removes the interface.
            
            Make sure to call this method, because this method makes sure that
            all necessary references are removed.
            
            Once this method is called this monitor can no longer be used.
        """
        self._endpoint.owner.unregisterInterface(self)
        self._endpoint.unregisterInterface(self)
        
        if self._conn:
            for interface in self._conn:
                interface.unregisterConnection(self)
            
            self._control.removeInterface(self._tag)
        
        if self._rosAddr:
            self._endpoint.freeAddr(self._rosAddr)
        
        self._endpoint = None
