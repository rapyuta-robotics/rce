#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     wrapper.py
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

# rce specific imports
from rce.util.name import validateName, IllegalName
from rce.core.error import InvalidRequest, AlreadyDead
from rce.slave.interface import Types


class _Wrapper(object):
    """ Base class for Wrapper classes, which are used to store additional
        information (of interest to the User) with the cloud engine
        representation of the objects.
    """
    def __init__(self, obj):
        """ Initialize the Wrapper around the object 'obj'.

            @param obj:         Object which should be wrapped.
            @type  obj:         rce.core.base.Proxy
        """
        self._obj = obj
        obj.notifyOnDeath(self._selfDied)

        self._cbs = set()

    def notifyOnDeath(self, cb):
        """ Method is used to forward 'notifyOnDeath' calls to the wrapped
            object. It is used to register a callback which will be called
            when the wrapped object died.

            @param cb:          Callback which should be registered. The
                                callback should take the died object as only
                                argument.
            @type  cb:          callable
        """
        assert callable(cb)

        try:
            self._cbs.add(cb)
        except AttributeError:
            raise AlreadyDead('{0} is already '
                              'dead.'.format(self.__class__.__name__))

    def dontNotifyOnDeath(self, cb):
        """ Method is used to forward 'dontNotifyOnDeath' calls to the wrapped
            object. It is used to unregister a callback which should have been
            called when the wrapped object died.

            @param cb:          Callback which should be unregistered.
            @type  cb:          callable
        """
        try:
            self._cbs.remove(cb)
        except AttributeError:
            pass

    def _selfDied(self, _):
        for cb in self._cbs:
            cb(self)

        self._cbs = None

    def destroy(self):
        """ Method should be called to destroy the Wrapper and will take care
            of destroying the wrapped object.
        """
        self._obj.destroy()
        self._obj = None


class Robot(_Wrapper):
    """ Wrapper for a Robot object. The underlying object is a Robot namespace.
    """
    def __init__(self, namespace):
        """ Initialize the Robot wrapper.

            @param namespace:   Namespace of the Robot object which should be
                                wrapped.
            @type  namespace:   rce.core.robot.Robot
        """
        super(Robot, self).__init__(namespace)

        self._interfaces = {}

    def getConnectInfo(self):
        """ Get the information necessary to the robot to establish a WebSocket
            connection.

            @return:            The authentication key and address which are
                                used for the WebSocket connection.
            @rtype:             twisted.internet.defer.Deferred
        """
        d = self._obj.getWebsocketAddress()
        d.addCallback(lambda addr: addr)
        return d

    def addInterface(self, iTag, iType, clsName):
        """ Add an interface to the Robot object.

            @param iTag:        Tag which is used to identify the interface in
                                subsequent requests.
            @type  iTag:        str

            @param iType:       Type of the interface. The type consists of a
                                prefix and a suffix.
                                 - Valid prefixes are:
                                     ServiceClient, ServiceProvider,
                                     Publisher, Subscriber
                                 - Valid suffixes are:
                                     Converter, Forwarder
            @type  iType:       str

            @param clsName:     Message type/Service type consisting of the
                                package and the name of the message/service,
                                i.e. 'std_msgs/Int32'.
            @type  clsName:     str
        """
        try:
            validateName(iTag)
        except IllegalName as e:
            raise InvalidRequest('Interface tag is invalid: {0}'.format(e))

        if iTag in self._interfaces:
            raise InvalidRequest("Can not use the same interface tag '{0}' "
                                 'in the same robot twice.'.format(iTag))

        try:
            iType = Types.encode(iType)
        except TypeError:
            raise InvalidRequest('Interface type is invalid.')

        interface = self._obj.createInterface(iType, clsName, iTag)
        interface = Interface(interface, iType, clsName)
        self._interfaces[iTag] = interface
        interface.notifyOnDeath(self._interfaceDied)

    def removeInterface(self, iTag):
        """ Remove an interface from the Robot object.

            @param iTag:        Tag which is used to identify the interface
                                which should be removed.
            @type  iTag:        str
        """
        try:
            self._interfaces.pop(iTag).destroy()
        except KeyError:
            raise InvalidRequest('Can not remove a non existent interface '
                                 "'{0}' from the robot.".format(iTag))

    def getInterface(self, iTag):
        """ Return the wrapped interface instance matching the given tag.

            @param iTag:        Tag which is used to identify the interface
                                which should be returned.
            @type  iTag:        str

            @return:            Wrapped interface instance which was requested.
            @rtype:             rce.core.user.Interface
        """
        try:
            return self._interfaces[iTag]
        except KeyError:
            raise InvalidRequest('Can not get a non existent interface '
                                 "'{0}' from the robot.".format(iTag))

    def _interfaceDied(self, interface):
        if self._interfaces:
            for key, value in self._interfaces.iteritems():
                if value == interface:
                    del self._interfaces[key]
                    break
        else:
            print('Received notification for dead Interface, '
                  'but Robot is already destroyed.')

    def destroy(self):
        """ Method should be called to destroy the robot and will take care of
            destroying all objects owned by this Robot as well as deleting all
            circular references.
        """
        for interface in self._interfaces.itervalues():
            interface.dontNotifyOnDeath(self._interfaceDied)

        self._interfaces = None

        super(Robot, self).destroy()


class Container(_Wrapper):
    """ Wrapper for a Container object. The underlying object is a Environment
        namespace.
    """
    def __init__(self, namespace, container):
        """ Initialize the Container wrapper.

            @param namespace:   Namespace of the Container object which should
                                be wrapped.
            @type  namespace:   rce.core.environment.Environment

            @param container:   Reference to Container.
            @type  container:   rce.core.container.Container
        """
        super(Container, self).__init__(namespace)

        self._container = container
        container.notifyOnDeath(self._containerDied)

        self._nodes = {}
        self._parameters = {}
        self._interfaces = {}

    def addNode(self, nTag, pkg, exe, args, name, namespace):
        """ Add a node to the ROS environment inside the container.

            @param nTag:        Tag which is used to identify the node in
                                subsequent requests.
            @type  nTag:        str

            @param pkg:         Name of ROS package where the node can be
                                found.
            @type  pkg:         str

            @param exe:         Name of executable (node) which should be
                                launched.
            @type  exe:         str

            @param args:        Additional arguments which should be used for
                                the launch.
            @type  args:        str

            @param name:        Name of the node under which the node should be
                                launched.
            @type  name:        str

            @param namespace:   Namespace in which the node should be started
                                in the environment.
            @type  namespace:   str
        """
        try:
            validateName(nTag)
        except IllegalName:
            raise InvalidRequest('Node tag is not a valid.')

        if nTag in self._nodes:
            raise InvalidRequest("Can not use the same node tag '{0}' in the "
                                 'same container twice.'.format(nTag))

        node = self._obj.createNode(pkg, exe, args, name, namespace)
        self._nodes[nTag] = node
        node.notifyOnDeath(self._nodeDied)

    def removeNode(self, nTag):
        """ Remove a node from the ROS environment inside the container.

            @param nTag:        Tag which is used to identify the ROS node
                                which should removed.
            @type  nTag:        str
        """
        try:
            self._nodes.pop(nTag).destroy()
        except KeyError:
            raise InvalidRequest('Can not remove a non existent node '
                                 "'{0}' from the container.".format(nTag))

    def addParameter(self, name, value):
        """ Add a parameter to the ROS environment inside the container.

            @param name:        Name of the parameter which should be added.
                                It is also used to identify the parameter in
                                subsequent requests.
            @type  name:        str

            @param value:       Value of the parameter which should be added.
            @type  value:       str, int, float, bool, list
        """
        if not name:
            raise InvalidRequest('Parameter name is not a valid.')

        if name in self._parameters:
            raise InvalidRequest("Can not use the same parameter name '{0}' "
                                 'in the same container twice.'.format(name))

        parameter = self._obj.createParameter(name, value)
        self._parameters[name] = parameter
        parameter.notifyOnDeath(self._parameterDied)

    def removeParameter(self, name):
        """ Remove a parameter from the ROS environment inside the container.

            @param name:        Name of the parameter which should be removed.
            @type  name:        str
        """
        try:
            self._parameters.pop(name).destroy()
        except KeyError:
            raise InvalidRequest('Can not remove a non existent node '
                                 "'{0}' from the container.".format(name))

    def addInterface(self, iTag, iType, clsName, addr):
        """ Add an interface to the ROS environment inside the container.

            @param iTag:        Tag which is used to identify the interface in
                                subsequent requests.
            @type  iTag:        str

            @param iType:       Type of the interface. The type has to be of
                                the form:
                                    {prefix}Interface
                                whit valid prefixes:
                                    ServiceClient, ServiceProvider,
                                    Publisher, Subscriber
            @type  iType:       str

            @param clsName:     Message type/Service type consisting of the
                                package and the name of the message/service,
                                i.e. 'std_msgs/Int32'.
            @type  clsName:     str

            @param addr:        ROS name/address which the interface should
                                use.
            @type  addr:        str
        """
        try:
            validateName(iTag)
        except IllegalName:
            raise InvalidRequest('Interface tag is not a valid.')

        if iTag in self._interfaces:
            raise InvalidRequest("Can not use the same interface tag '{0}' "
                                 'in the same container twice.'.format(iTag))

        try:
            iType = Types.encode(iType)
        except TypeError:
            raise InvalidRequest('Interface type is invalid (Unknown prefix).')

        interface = self._obj.createInterface(iType, clsName, addr)
        interface = Interface(interface, iType, clsName)
        self._interfaces[iTag] = interface
        interface.notifyOnDeath(self._interfaceDied)

    def removeInterface(self, iTag):
        """ Remove an interface from the ROS environment inside the container.

            @param iTag:        Tag which is used to identify the interface
                                which should be removed.
            @type  iTag:        str
        """
        try:
            self._interfaces.pop(iTag).destroy()
        except KeyError:
            raise InvalidRequest('Can not remove a non existent interface '
                                 "'{0}' from the container.".format(iTag))

    def getInterface(self, iTag):
        """ Return the wrapped interface instance matching the given tag.

            @param iTag:        Tag which is used to identify the interface
                                which should be returned.
            @type  iTag:        str

            @return:            Wrapped interface instance which was requested.
            @rtype:             rce.core.user.Interface
        """
        try:
            return self._interfaces[iTag]
        except KeyError:
            raise InvalidRequest('Can not get a non existent interface '
                                 "'{0}' from the container.".format(iTag))

    def getConnectInfo(self):
        """ Get connection information for the given container for rosproxy
            calls.
        """
        d = self._obj.getAddress()
        d.addCallback(lambda addr: 'http://{0}:{1}/'.format(addr.host,
                                                            addr.port + 2000))
        return d

    def _containerDied(self, container):
        if self._container:
            assert container == self._container
            self._container = None
            self.destroy()
        else:
            print('Received notification for dead Container, '
                  'but Container is already destroyed.')

    def _nodeDied(self, node):
        if self._nodes:
            for key, value in self._nodes.iteritems():
                if value == node:
                    del self._nodes[key]
                    break
        else:
            print('Received notification for dead Node, '
                  'but Container is already destroyed.')

    def _parameterDied(self, parameter):
        if self._parameters:
            for key, value in self._parameters.iteritems():
                if value == parameter:
                    del self._parameters[key]
                    break
        else:
            print('Received notification for dead Parameter, '
                  'but Container is already destroyed.')

    def _interfaceDied(self, interface):
        if self._interfaces:
            for key, value in self._interfaces.iteritems():
                if value == interface:
                    del self._interfaces[key]
                    break
        else:
            print('Received notification for dead Interface, '
                  'but Container is already destroyed.')

    def destroy(self):
        """ Method should be called to destroy the container and will take care
            of destroying all objects owned by this Container as well as
            deleting all circular references.
        """
        for node in self._nodes.itervalues():
            node.dontNotifyOnDeath(self._nodeDied)

        for parameter in self._parameters.itervalues():
            parameter.dontNotifyOnDeath(self._parameterDied)

        for interface in self._interfaces.itervalues():
            interface.dontNotifyOnDeath(self._interfaceDied)

        self._nodes = None
        self._parameters = None
        self._interfaces = None

        if self._container:
            self._container.dontNotifyOnDeath(self._containerDied)
            self._container.destroy()

        super(Container, self).destroy()


class Interface(_Wrapper):
    """ Wrapper for a Container object. The underlying object is an Interface.
    """
    def __init__(self, interface, iType, clsName):
        """ Initialize the Interface wrapper.

            @param interface:   Interface which should be wrapped.
            @type  interface:   rce.core.network.Interface

            @param iType:       Type of the interface encoded as an integer.
                                Refer to rce.slave.interface.Types for more
                                information.
            @type  iType:       int

            @param clsName:     Message type/Service type consisting of the
                                package and the name of the message/service,
                                i.e. 'std_msgs/Int32'.
            @type  clsName:     str
        """
        super(Interface, self).__init__(interface)

        self.iType = iType
        self.clsName = clsName

    @property
    def obj(self):
        """ Reference to the wrapped Interface instance. """
        return self._obj
