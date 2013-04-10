#!/usr/bin/env python
# -*- coding: utf-8 -*-
#     
#     view.py
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
#     \author/s: Dominique Hunziker, Mayank Singh
#     
#     

# Python specific imports
from uuid import uuid4
from hashlib import md5

# twisted specific imports
from twisted.spread.pb import Referenceable, Avatar
from twisted.internet.defer import DeferredList
from twisted.spread.pb import Viewable

# Custom imports
from rce.error import InvalidRequest, AlreadyDead
from rce.slave.interface import Types
from rce.util.name import validateName
from rce.master.wrapper import Robot, Container, Interface

class RobotView(Viewable):
    """
    """
    def view_createContainer(self, clientAvatar, tag):
        """ Create a new Container object.
            
            @param tag:         Tag which is used to identify the container
                                in subsequent requests.
            @type  tag:         str
        """
        validateName(tag)
        
        if tag in clientAvatar._containers or tag in clientAvatar._robots:
            raise InvalidRequest('Tag is already used for a container '
                                 'or robot.')
        
        namespace, remote_container = clientAvatar._realm.createContainer(clientAvatar._userID)
        container = Container(namespace, remote_container)
        clientAvatar._containers[tag] = container
        container.notifyOnDeath(clientAvatar._containerDied)
        
        m = 'Container {0} successfully created.'.format(tag)
        d = DeferredList([namespace(), remote_container()],
                         fireOnOneErrback=True, consumeErrors=True)
        return d.addCallback(lambda _: m)
    
    def view_destroyContainer(self, clientAvatar, tag):
        """ Destroy a Container object.
            
            @param tag:         Tag which is used to identify the container
                                which should be destroyed.
            @type  tag:         str
        """
        try:
            container = clientAvatar._containers.pop(tag)
        except KeyError:
            raise InvalidRequest('Can not destroy non existent container.')
        
        container.dontNotifyOnDeath(clientAvatar._containerDied)
        container.destroy()
        
        # TODO: Return some info about success/failure of request
    
    def view_addNode(self, clientAvatar, cTag, nTag, pkg, exe, args='', name='', nspc=''):
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
        """
        try:
            clientAvatar._containers[cTag].addNode(nTag, pkg, exe, args, name, nspc)
        except KeyError:
            raise InvalidRequest('Can not add Node, because Container {0} '
                                 'does not exist.'.format(cTag))
        
        # TODO: Return some info about success/failure of request
    
    def view_removeNode(self, clientAvatar, cTag, nTag):
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
            clientAvatar._containers[cTag].removeNode(nTag)
        except KeyError:
            raise InvalidRequest('Can not remove Node, because Container {0} '
                                 'does not exist.'.format(cTag))
        
        # TODO: Return some info about success/failure of request
    
    def view_addParameter(self, clientAvatar, cTag, name, value):
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
        """
        try:
            clientAvatar._containers[cTag].addParameter(name, value)
        except KeyError:
            raise InvalidRequest('Can not add Parameter, because Container '
                                 '{0} does not exist.'.format(cTag))
        
        # TODO: Return some info about success/failure of request
    
    def view_removeParameter(self, clientAvatar, cTag, name):
        """ Remove a parameter from a ROS environment.
            
            @param cTag:        Tag which is used to identify the ROS
                                environment from which the parameter should be
                                removed.
            @type  cTag:        str
            
            @param name:        Name of the parameter which should be removed.
            @type  name:        str
        """
        try:
            clientAvatar._containers[cTag].removeParameter(name)
        except KeyError:
            raise InvalidRequest('Can not remove Parameter, because Container '
                                 '{0} does not exist.'.format(cTag))
        
        # TODO: Return some info about success/failure of request
    
    def view_addInterface(self, clientAvatar, eTag, iTag, iType, clsName, addr=''):
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
        """
        if iType.endswith('Converter') or iType.endswith('Forwarder'):
            try:
                clientAvatar._robots[eTag].addInterface(iTag, iType, clsName)
            except KeyError:
                raise InvalidRequest('Can not add Interface, because Robot '
                                     '{0} does not exist.'.format(eTag))
        elif iType.endswith('Interface'):
            try:
                clientAvatar._containers[eTag].addInterface(iTag, iType, clsName, addr)
            except KeyError:
                raise InvalidRequest('Can not add Interface, because '
                                     'Container {0} does not '
                                     'exist.'.format(eTag))
        else:
            raise InvalidRequest('Interface type is invalid (Unknown suffix).')
        
        # TODO: Return some info about success/failure of request
    
    def view_removeInterface(self, clientAvatar, eTag, iTag):
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
        clientAvatar._getEndpoint(eTag).removeInterface(iTag)
        
        # TODO: Return some info about success/failure of request
    
    def view_addConnection(self, clientAvatar, tagA, tagB):
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
        
        ifA = clientAvatar._getEndpoint(eTagA).getInterface(iTagA)
        ifB = clientAvatar._getEndpoint(eTagB).getInterface(iTagB)
        
        if ifA.clsName != ifB.clsName:
            raise InvalidRequest('Can not connect two interfaces with '
                                 'different message/service type.')
        
        if not Types.connectable(ifA.iType, ifB.iType):
            raise InvalidRequest('Can not connect an interface of type {0} '
                                 'and an interface of type '
                                 '{1}.'.format(Types.decode(ifA.iType),
                                               Types.decode(ifB.iType)))
        
        key = hash(tagA)^hash(tagB)
        
        if key in clientAvatar._connections:
            raise InvalidRequest('Can not add the same connection twice.')
        
        connection = clientAvatar._realm.createConnection(ifA.obj, ifB.obj)
        clientAvatar._connections[key] = connection
        connection.notifyOnDeath(clientAvatar._connectionDied)
        
        # TODO: Return some info about success/failure of request
    
    def view_removeConnection(self, clientAvatar, tagA, tagB):
        """ Destroy a connection between two interfaces.
            
            @param tagX:        Tag which is used to identify the interface
                                which should be disconnected. It has to be of
                                the form:
                                    [endpoint tag]/[interface tag]
                                For example:
                                    testRobot/logPublisher
            @type  tagX:        str
        """
        key = hash(tagA)^hash(tagB)
        
        try:
            connection = clientAvatar._connections.pop(key)
        except KeyError:
                raise InvalidRequest('Can not disconnect two unconnected '
                                     'interfaces.')
        
        connection.dontNotifyOnDeath(clientAvatar._connectionDied)
        connection.destroy()
        
        # TODO: Return some info about success/failure of request
        

class NormalConsoleView(Viewable):
    def view_update_user(self, clientAvatar, username, password, old_password):
        """ Remote call to edit user information.

            @param username:       The username
            @type  username:       string
            
            @param password:       The password
            @type  password:       string

            @param old_password:   Old password
            @type  old_password:   string
        """
        clientAvatar._realm._checker.passwd(username, password, old_password)
    
    def view_list_containers(self, clientAvatar):
        """ Remote call to list containers under the logged-in user.
        """
        return clientAvatar.containers.keys()

    def view_get_rosapi_connect_info(self, clientAvatar, tag):
        """ Remote call to get rosapi request URL and key for a particular
            container.
            
            @param tag:    Tag associated with a container.
            @type  tag:    string
        """
        try:
            container = clientAvatar._containers[tag]
        except KeyError:
            raise InternalError('No such container')
        else:
            uid = uuid4().hex
            container._obj.registerConsole(clientAvatar.userID, uid)
            d = container.getConnectInfo()
            d.addCallback(lambda addr: (addr, uid))
            return d
    
    def view_list_robots(self, clientAvatar):
        """ List Robots under the logged in user.
        """
        return clientAvatar.robots.keys()


class AdminConsoleView(Viewable):
    def view_machine_containers(self, clientAvatar, machineIP):
        """ Remote call to list containers in a machine with given IP.
        
            @param machineIP:    IP of master server
            @type  machineIP:    string
        """
        # TODO: Can not return rce.master.container.Container instances need
        #       some conversion into string, tuple, or some other base type
        try:
            return (machine for machine in clientAvatar._realm._balancer._machines
                    if machineIP == machine.IP).next()._containers
        except StopIteration:
            raise InternalError('No such machine')

    def view_list_machines(self, clientAvatar):
        """ Remote call to list machine IPs.
        """
        return [machine.IP for machine in clientAvatar._realm._balancer._machines]
    
    def view_stats_machine(self, clientAvatar, machineIP):
        """ Remote call to list stats of machine with given IP.
        
            @param machineIP:    IP of master server.
            @type  machineIP:    string
        """
        try:
            machine = (machine for machine in clientAvatar._realm._balancer._machines
                       if machineIP == machine.IP).next()
            return {'active':machine.active, 'capacity':machine.capacity}
        except StopIteration:
            raise InternalError('No such machine')

    def view_add_user(self, clientAvatar, username, password):
        """ Remote call to add user.
        
            @param username:    The username
            @type  username:    string

            @param password:    The password
            @type  password:    string
        """
        clientAvatar._realm._checker.addUser(username, password)

    def view_remove_user(self, clientAvatar, username):
        """ Remote call to remove user.

            @param username:    The username
            @type  username:    string
        """
        clientAvatar._realm._checker.removeUser(username)

    def view_update_user(self, clientAvatar, username, password):
        """ Remote call to edit user information.

            @param username:       The username
            @type  username:       string
            
            @param password:       The password
            @type  password:       string

            @param old_password:   Old password
            @type  old_password:   string
        """
        clientAvatar._realm._checker.passwd(username, password, True)

    def view_list_users(self, clientAvatar):
        """ Remote call to list all users logged into RoboEarthCloudEngine.
        """
        return clientAvatar._realm._users.keys()

    def view_list_containers_by_user(self, clientAvatar, userID):
        """ Remote call to list containers under a given user.
            
            @param userID:    The username
            @type  userID:    string
        """
        user = clientAvatar._realm._getUser(userID)
        return user.containers.keys()

    def view_list_robots_by_user(self, clientAvatar, userID):
        """ List robots under the user specified.
        
            @param userID:    The username
            @type  userID:    string
        """
        user = clientAvatar._realm._getUser(userID)
        return user.robots.keys()
