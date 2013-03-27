#!/usr/bin/env python
# -*- coding: utf-8 -*-
#     
#     rce-core/rce/master/user.py
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
from uuid import uuid4
from hashlib import md5

# twisted specific imports
from twisted.spread.pb import Referenceable, Error
from twisted.internet.defer import DeferredList

# Custom imports
from rce.master.base import AlreadyDead
from rce.slave.interface import Types
from rce.util.name import validateName


class InvalidRequest(Error):
    """ Exception is raised if the request can not be processed.
    """


class User(Referenceable):
    """ Represents a User. It has references to all objects the User is
        currently using and all requests from the outside will be going through
        here. For this to happen, each Robot object in the robot processes has
        a remote reference to on of the User instances in the Master.
    """
    def __init__(self, realm, userID):
        """ Initialize the User.
            
            @param realm:       The realm of the RoboEarth Cloud Engine master.
            @type  realm:       rce.master.core.RoboEarthCloudEngine
            
            @param userID:      The user ID associated with the user.
            @type  userID:      str
        """
        self._realm = realm
        self._userID = userID
        
        self._robots = {}
        self._containers = {}
        self._connections = {}
    
    @property
    def userID(self):
        """ User ID of this User. """
        return self._userID
    
    @property
    def robots(self):
        """ Robots owned by this User. """
        return self._robots

    @property
    def containers(self):
        """" Containers used by this User. """
        return self._containers

    def createRobot(self, robotID):
        """ Create a new Robot object.
            
            @param robotID:     The robot ID associated with the new Robot
                                object which should be created.
            @type  robotID:     str
            
            @return:            The authentication key and address which are
                                used for the websocket connection.
            @rtype:             twisted::Deferred
            
            @raise:             rce.util.name.IllegalName,
                                rce.master.user.InvalidRequest
        """
        validateName(robotID)
        
        if (robotID in self._robots or robotID in self._containers):
            raise InvalidRequest('ID is already used for a container '
                                 'or robot.')
        
        uid = uuid4().hex
        robot = self._realm.createRobot(self, robotID, uid)
        robot = Robot(robot, uid)
        self._robots[robotID] = robot
        robot.notifyOnDeath(self._robotDied)
        return robot.getConnectInfo()
    
    def remote_createContainer(self, tag):
        """ Create a new Container object.
            
            @param tag:         Tag which is used to identify the container
                                in subsequent requests.
            @type  tag:         str
            
            @raise:             rce.util.name.IllegalName,
                                rce.master.user.InvalidRequest
        """
        validateName(tag)
        
        if tag in self._containers or tag in self._robots:
            raise InvalidRequest('Tag is already used for a container '
                                 'or robot.')
        
        namespace, remote_container = self._realm.createContainer(self._userID)
        container = Container(namespace, remote_container)
        self._containers[tag] = container
        container.notifyOnDeath(self._containerDied)
        
        m = 'Container {0} successfully created.'.format(tag)
        d = DeferredList([namespace(), remote_container()],
                         fireOnOneErrback=True, consumeErrors=True)
        return d.addCallback(lambda _: m)
    
    def remote_destroyContainer(self, tag):
        """ Destroy a Container object.
            
            @param tag:         Tag which is used to identify the container
                                which should be destroyed.
            @type  tag:         str
        """
        try:
            container = self._containers.pop(tag)
        except KeyError:
            raise InvalidRequest('Can not destroy non existent container.')
        
        container.dontNotifyOnDeath(self._containerDied)
        container.destroy()
        
        # TODO: Return some info about success/failure of request
    
    def remote_addNode(self, cTag, nTag, pkg, exe, args='', name='', nspc=''):
        """ Add a node to a ROS environment.
            
            @param cTag:        Tag which is used to identify the ROS
                                environment to which the node should be added.
            @type  cTag:        str
            
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
                                the launch. Can contain the directives
                                $(find PKG) or $(env VAR). Other special
                                characters as '$' or ';' are not allowed.
            @type  args:        str
            
            @param name:        Name of the node under which the node should be
                                launched.
            @type  name:        str
            
            @param namespace:   Namespace in which the node should be started
                                in the environment.
            @type  namespace:   str
            
            @raise:             rce.util.name.IllegalName,
                                rce.master.user.InvalidRequest
        """
        try:
            self._containers[cTag].addNode(nTag, pkg, exe, args, name, nspc)
        except KeyError:
            raise InvalidRequest('Can not add Node, because Container {0} '
                                 'does not exist.'.format(cTag))
        
        # TODO: Return some info about success/failure of request
    
    def remote_removeNode(self, cTag, nTag):
        """ Remove a node from a ROS environment.
            
            @param cTag:        Tag which is used to identify the ROS
                                environment from which the node should be
                                removed.
            @type  cTag:        str
            
            @param nTag:        Tag which is used to identify the ROS node
                                which should removed.
            @type  nTag:        str
        """
        try:
            self._containers[cTag].removeNode(nTag)
        except KeyError:
            raise InvalidRequest('Can not remove Node, because Container {0} '
                                 'does not exist.'.format(cTag))
        
        # TODO: Return some info about success/failure of request
    
    def remote_addParameter(self, cTag, name, value):
        """ Add a parameter to a ROS environment.
            
            @param cTag:        Tag which is used to identify the ROS
                                environment to which the parameter should be
                                added.
            @type  cTag:        str
            
            @param name:        Name of the parameter which should be added.
                                It is also used to identify the parameter in
                                subsequent requests.
            @type  name:        str
            
            @param value:       Value of the parameter which should be added.
                                String values can contain the directives
                                $(find PKG) or $(env VAR).
            @type  value:       str, int, float, bool, list
            
            @raise:             rce.util.name.IllegalName,
                                rce.master.user.InvalidRequest
        """
        try:
            self._containers[cTag].addParameter(name, value)
        except KeyError:
            raise InvalidRequest('Can not add Parameter, because Container '
                                 '{0} does not exist.'.format(cTag))
        
        # TODO: Return some info about success/failure of request
    
    def remote_removeParameter(self, cTag, name):
        """ Remove a parameter from a ROS environment.
            
            @param cTag:        Tag which is used to identify the ROS
                                environment from which the parameter should be
                                removed.
            @type  cTag:        str
            
            @param name:        Name of the parameter which should be removed.
            @type  name:        str
        """
        try:
            self._containers[cTag].removeParameter(name)
        except KeyError:
            raise InvalidRequest('Can not remove Parameter, because Container '
                                 '{0} does not exist.'.format(cTag))
        
        # TODO: Return some info about success/failure of request
    
    def remote_addInterface(self, eTag, iTag, iType, clsName, addr=''):
        """ Add an interface to an endpoint, i.e. a ROS environment or a 
            Robot object.
            
            @param eTag:        Tag which is used to identify the endpoint to
                                which the interface should be added; either
                                a container tag or robot ID.
            @type  eTag:        str
                            
            @param iTag:        Tag which is used to identify the interface in
                                subsequent requests.
            @type  iTag:        str
            
            @param iType:       Type of the interface. The type consists of a
                                prefix and a suffix.
                                 - Valid prefixes are:
                                     ServiceClient, ServiceProvider,
                                     Publisher, Subscriber
                                 - Valid suffixes are:
                                     Interface, Converter, Forwarder
            @type  iType:       str
            
            @param clsName:     Message type/Service type consisting of the
                                package and the name of the message/service,
                                i.e. 'std_msgs/Int32'.
            @type  clsName:     str
            
            @param addr:        ROS name/address which the interface should
                                use. Only necessary if the suffix of @param
                                iType is 'Interface'.
            @type  addr:        str
            
            @raise:             rce.util.name.IllegalName,
                                rce.master.user.InvalidRequest
        """
        if iType.endswith('Converter') or iType.endswith('Forwarder'):
            try:
                self._robots[eTag].addInterface(iTag, iType, clsName)
            except KeyError:
                raise InvalidRequest('Can not add Interface, because Robot '
                                     '{0} does not exist.'.format(eTag))
        elif iType.endswith('Interface'):
            try:
                self._containers[eTag].addInterface(iTag, iType, clsName, addr)
            except KeyError:
                raise InvalidRequest('Can not add Interface, because '
                                     'Container {0} does not '
                                     'exist.'.format(eTag))
        else:
            raise InvalidRequest('Interface type is invalid (Unknown suffix).')
        
        # TODO: Return some info about success/failure of request
    
    def remote_removeInterface(self, eTag, iTag):
        """ Remove an interface from an endpoint, i.e. a ROS environment or a 
            Robot object.
            
            @param eTag:        Tag which is used to identify the endpoint from
                                which the interface should be removed; either
                                a container tag or robot ID.
            @type  eTag:        str
            
            @param iTag:        Tag which is used to identify the interface
                                which should be removed.
            @type  iTag:        str
        """
        self._getEndpoint(eTag).removeInterface(iTag)
        
        # TODO: Return some info about success/failure of request
    
    def remote_addConnection(self, tagA, tagB):
        """ Create a connection between two interfaces.
            
            @param tagX:        Tag which is used to identify the interface
                                which should be connected. It has to be of the
                                form:
                                    [endpoint tag]/[interface tag]
                                For example:
                                    testRobot/logPublisher
            @type  tagX:        str
        """
        eTagA, iTagA = tagA.split('/', 2)
        eTagB, iTagB = tagB.split('/', 2)
        
        ifA = self._getEndpoint(eTagA).getInterface(iTagA)
        ifB = self._getEndpoint(eTagB).getInterface(iTagB)
        
        if ifA.clsName != ifB.clsName:
            raise InvalidRequest('Can not connect two interfaces with '
                                 'different message/service type.')
        
        if not Types.connectable(ifA.iType, ifB.iType):
            raise InvalidRequest('Can not connect an interface of type {0} '
                                 'and an interface of type '
                                 '{1}.'.format(Types.decode(ifA.iType),
                                               Types.decode(ifB.iType)))
        
        key = int(md5(tagA).hexdigest(), 16)^int(md5(tagB).hexdigest(), 16)
        
        if key in self._connections:
            raise InvalidRequest('Can not add the same connection twice.')
        
        connection = self._realm.createConnection(ifA.obj, ifB.obj)
        self._connections[key] = connection
        connection.notifyOnDeath(self._connectionDied)
        
        # TODO: Return some info about success/failure of request
    
    def remote_removeConnection(self, tagA, tagB):
        """ Destroy a connection between two interfaces.
            
            @param tagX:        Tag which is used to identify the interface
                                which should be disconnected. It has to be of
                                the form:
                                    [endpoint tag]/[interface tag]
                                For example:
                                    testRobot/logPublisher
            @type  tagX:        str
        """
        key = int(md5(tagA).hexdigest(), 16)^int(md5(tagB).hexdigest(), 16)
        
        try:
            connection = self._connections.pop(key)
        except KeyError:
                raise InvalidRequest('Can not disconnect two unconnected '
                                     'interfaces.')
        
        connection.dontNotifyOnDeath(self._connectionDied)
        connection.destroy()
        
        # TODO: Return some info about success/failure of request
    
    def _getEndpoint(self, tag):
        if tag in self._robots:
            return self._robots[tag]
        elif tag in self._containers:
            return self._containers[tag]
        else:
            raise InvalidRequest('Can not get a non existent endpoint '
                                 "'{0}'.".format(tag))
    
    def _containerDied(self, container):
        if self._containers:
            for key, value in self._containers.iteritems():
                if value == container:
                    del self._containers[key]
                    break
        else:
            print('Received notification for dead Container, '
                  'but User is already destroyed.')
    
    def _robotDied(self, robot):
        if self._robots:
            for key, value in self._robots.iteritems():
                if value == robot:
                    del self._robots[key]
                    break
        else:
            print('Received notification for dead Robot, '
                  'but User is already destroyed.')
    
    def _connectionDied(self, connection):
        if self._connections:
            for key, value in self._connections.iteritems():
                if value == connection:
                    del self._connections[key]
                    break
        else:
            print('Received notification for dead Connection, '
                  'but User is already destroyed.')
    
    def destroy(self):
        """ Method should be called to destroy the user and will take care of
            destroying all objects owned by this User as well as deleting all
            circular references.
        """
        for connection in self._connections.itervalues():
            connection.dontNotifyOnDeath(self._connectionDied)
        
        for container in self._containers.itervalues():
            container.dontNotifyOnDeath(self._containerDied)
        
        for robot in self._robots.itervalues():
            robot.dontNotifyOnDeath(self._robotDied)
        
        for container in self._containers.itervalues():
            container.destroy()
        
        for robot in self._robots.itervalues():
            robot.destroy()
        
        self._connections = None
        self._containers = None
        self._robots = None


class _Wrapper(object):
    """ Base class for Wrapper classes, which are used to store additional
        information (of interest to the User) with the cloud engine
        representation of the objects.
    """
    def __init__(self, obj):
        """ Initialize the Wrapper around the object 'obj'.
            
            @param obj:         Object which should be wrapped.
            @type  obj:         rce.master.base.Proxy
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
    def __init__(self, namespace, key):
        """ Initialize the Robot wrapper.
            
            @param namespace:   Namespace of the Robot object which should be
                                wrapped.
            @type  namespace:   rce.master.robot.Robot
            
            @param key:         Key which is used by the robot to authenticate
                                the connection between the robot and the cloud
                                engine.
            @type  key:         str
        """
        super(Robot, self).__init__(namespace)
        
        self._key = key
        self._interfaces = {}
    
    def getConnectInfo(self):
        """ Get the information necessary to the robot to establish a websocket
            connection.
            
            @return:            The authentication key and address which are
                                used for the websocket connection.
            @rtype:             twisted::Deferred          
        """
        d = self._obj.getWebsocketAddress()
        d.addCallback(lambda addr: (self._key, addr))
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
            
            @raise:             rce.util.name.IllegalName,
                                rce.master.user.InvalidRequest
        """
        validateName(iTag)
        
        if iTag in self._interfaces:
            raise InvalidRequest("Can not use the same interface tag '{0}' "
                                 'in the same robot twice.'.format(iTag))
        
        modifier = 4 if iType.endswith('Forwarder') else 0
        
        try:
            iType = Types.encode(iType)
        except TypeError:
            raise InvalidRequest('Interface type is invalid (Unknown prefix).')
        
        interface = self._obj.createInterface(iType+modifier, clsName, iTag)
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
            @rtype:             rce.master.user.Interface
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
            @type  namespace:   rce.master.environment.Environment
            
            @param container:   Reference to Container.
            @type  container:   rce.master.container.Container
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
            
            @raise:             rce.util.name.IllegalName,
                                rce.master.user.InvalidRequest
        """
        validateName(nTag)
        
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
            
            @raise:             rce.util.name.IllegalName,
                                rce.master.user.InvalidRequest
        """
        validateName(iTag)
        
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
            @rtype:             rce.master.user.Interface
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
                                                            addr.port+2000))
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
            @type  interface:   rce.master.network.Interface
            
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
