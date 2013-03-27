#!/usr/bin/env python
# -*- coding: utf-8 -*-
#     
#     rce-core/rce/master/console.py
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
#     \author/s: Dhananjay Sathe, Mayank Singh
#     
#     

# Python specific imports
from uuid import uuid4
from collections import defaultdict

# zope specific imports
from zope.interface import implements

# twisted specific imports
from twisted.cred.portal import IRealm
from twisted.spread.pb import IPerspective, Avatar

# Custom imports
from rce.error import InternalError


class InterfaceConnection2way(object):
    """ A simple two-way dictionary implementation
    """
    def __init__(self):
        self._d = defaultdict(list)

    def __getitem__(self, key):
        ret = self._d.get(key)
        if ret:
            return ret
        else:
            raise KeyError

    def __setitem__(self, key, val):
        self._d[key].append(val)
        self._d[val].append(key)

    def keys(self):
        return self._d.keys()

    def iterkeys(self):
        return self._d.iterkeys()

    def get(self, key):
        self._d.get(key)

    def items(self):
        return self._d.items()

    def iteritems(self):
        return self._d.iteritems()

    def values(self):
        return self._d.values()

    def itervalues(self):
        return self._d.itervalues()


class UnauthorisedLogon(Exception):
    """ Exception to be raised when the user login fails
        due to incorrect user credentials.
    """


class ConsoleDummyRealm(object):
    """ A dummy realm that routes console users to the RoboEarthCloudEngine
        Realm so as not to involve it's requestAvatar method.
    """
    implements(IRealm)
    
    def __init__(self, rce):
        self._rce = rce

    def requestAvatar(self, avatarId, mind, *interfaces):
        if IPerspective not in interfaces:
            raise NotImplementedError('RoboEarthCloudEngine only '
                                      'handles IPerspective.')

        user = self._rce._getUser(avatarId)
        avatar = UserAvatar(user, self._rce._console)

        return IPerspective, avatar, lambda: None


class UserAvatar(Avatar):
    """ Avatar to be returned when user logs in via console.
    """
    def __init__(self, user, console):
        """ Initializer for user avatar.
        
            @param user:    User object
            @type  user:    rce.master.user.User
            
            @param console: Reference to Console Proxy object
            @type  console: rce.master.console.Console
        """
        self.user = user
        self.console = console

    def perspective_list_machines(self):
        """ Remote call to list machine IPs.
        """
        return self.console._list_machines_byIP()
    
    def perspective_stats_machine(self, machineIP):
        """ Remote call to list stats of machine with given IP.
        
            @param machineIP:    IP of master server.
            @type  machineIP:    string
        """
        return self.console._list_machine_stats(machineIP)
    
    def perspective_machine_containers(self, machineIP):
        """ Remote call to list containers in a machine with given IP.
        
            @param machineIP:    IP of master server
            @type  machineIP:    string
        """
        # TODO: Can not return rce.master.container.Container instances need
        #       some conversion into string, tuple, or some other base type
        return self.console._list_machine_containers(machineIP)
    
    def perspective_add_user(self, username, password):
        """ Remote call to add user.
        
            @param username:    The username
            @type  username:    string
    
            @param password:    The password
            @type  password:    string
        """
        self.console._add_user(username, password)
    
    def perspective_remove_user(self, username):
        """ Remote call to remove user.

            @param username:    The username
            @type  username:    string
        """
        self.console._remove_user(username)

    def perspective_update_user(self, username, new_password, control_mode):
        """ Remote call to change user credentials.

            @param username:        username
            @type  username:        str
            
            @param new_password:    new password
            @type  new_password:    str
            
            @param control_mode:    in user mode:  old password
                                    in admin mode: True
            @type  control_mode:    in user mode:  str
                                    in admin mode: bool
        """
        self.console._change_password(username, new_password, control_mode)
    
    def perspective_list_users(self):
        """ Remote call to list all users logged into RoboEarthCloudEngine.
        """
        return self.console._list_userID()

    def perspective_start_container(self, tag):
        """ Remote call to start container given the tag.
        
            @param tag:    Tag associated with a container.
            @type  tag:    string
        """
        self.user.remote_createContainer(tag)
    
    def perspective_stop_container(self, tag):
        """ Remote call to stop container given the tag.
        
            @param tag:    Tag associated with a container.
            @type  tag:    string
        """
        self.user.remote_destroyContainer(tag)

    def perspective_list_containers(self):
        """ Remote call to list containers under the logged-in user.
        """
        return self.console._list_user_containers(self.user)

    def perspective_list_containers_by_user(self, userID):
        """ Remote call to list containers under a given user.
            
            @param userID:    The username
            @type  userID:    string
        """
        user = self.console._root._getUser(userID)
        return self.console._list_user_containers(user)

    def perspective_get_rosapi_connect_info(self, tag):
        """ Remote call to get rosapi request URL and key for a particular
            container.
            
            @param tag:    Tag associated with a container.
            @type  tag:    string
        """
        try:
            container = self.user._containers[tag]
        except KeyError:
            raise InternalError('No such container')
        else:
            uid = uuid4().hex
            container._obj.registerConsole(self.user.userID, uid)
            d = container.getConnectInfo()
            d.addCallback(lambda addr: (addr, uid))
            return d
    
    def perspective_start_node(self, cTag, nTag, pkg, exe, args=''):
        """ Remote call to add a node to a ROS environment.
        """
        self.user.remote_addNode(cTag, nTag, pkg, exe, args)
    
    def perspective_stop_node(self, cTag, nTag):
        """ Remote call to remove a node from a ROS environment.
        """
        self.user.remote_removeNode(cTag, nTag)
    
    def perspective_add_parameter(self, cTag, name, value):
        """ Remote call to add a parameter to a ROS environment.
        """
        self.user.remote_addParameter(cTag, name, value)
    
    def perspective_remove_parameter(self, cTag, name):
        """ Remote call to remove a parameter from a ROS environment.
        """
        self.user.remote_removeParameter(cTag, name)
    
    def perspective_add_interface(self, eTag, iTag, iType, iCls, addr=''):
        """ Remote call to add an interface.
        """
        self.user.remote_addInterface(eTag, iTag, iType, iCls, addr)

    def perspective_remove_interface(self, eTag, iTag):
        """ Remote call to remove an interface.
        """
        self.user.remote_removeInterface(eTag, iTag)
    
    def perspective_add_connection(self, tag1, tag2):
        """ Remote call to add a connection between two interfaces.

            @param tag1:    Tag for first interface
            @type  tag1:    string

            @param tag2:    Tag for second interface
            @type  tag2:    string
        """
        self.user.remote_addConnection(tag1, tag2)
        
    def perspective_remove_connection(self, tag1, tag2):
        """ Remote call to remove connection between two interfaces.

            @param tag1:    Tag for first interface
            @type  tag1:    string
            @param tag2:    Tag for second interface
            @type  tag2:    string
        """
        self.user.remote_removeConnection(tag1, tag2)
    
    def perspective_list_robots(self):
        """ List Robots under the logged in user.
        """
        return self.console._list_user_robots(self.user)

    def perspective_list_robots_by_user(self, userID):
        """ List robots under the user specified.
        
            @param userID:    The username
            @type  userID:    string
        """
        user = self.console._root._getUser(userID)
        return self.console._list_user_robots(user)


class Console(object):
    """
    The selective class that helps implement the console abilities
    """
    def __init__(self, root):
        """ Initialize the Console Handler.
            
            @param root:        Reference to top level of data structure.
            @type  root:        rce.master.core.RoboEarthCloudEngine
        """
        self._root = root
    
    # The following method is only accessible to the Admin user
#    def _list_machines(self): # TODO: Method is never used
#        """ Gets a list of all machines that are available in the Cloud Engine.
#            
#            This should be only accessible to the top level admin for security
#            reasons.
#        
#            @return:        List of machines .
#            @rtype:         List(rce.master.machine.Machine)
#        """
#        return self._root._balancer._machines
    
    def _list_machines_byIP(self):
        """ Gets a list of all machines that are available in the Cloud Engine.
            
            This should be only accessible to the top level admin for security
            reasons.
        
            @return:        List of machines by IPv4 addresses
            @rtype:         List(str)
        """
#        return [machine.IP for machine in self._list_machines()]
        return [machine.IP for machine in self._root._balancer._machines]
    
    def _list_machine_containers(self, machineIP):
        """ Gets a list of all containers available in the Machine.
            
            This should be only accessible to the top level admin for security
            reasons
        
            @param machine:    Machine for which the containers need to be
                               listed.
            @type  machine:    rce.master.machine.Machine
        
            @return:           List of containers.
            @rtype:            List(rce.master.container.Container)
        """
        try:
#            return (machine for machine in self._list_machines()
#                    if machineIP == machine.IP).next()._containers
            return (machine for machine in self._root._balancer._machines
                    if machineIP == machine.IP).next()._containers
        except StopIteration:
            raise InternalError('No such machine')

    def _list_machine_stats(self, machineIP):
        """ Gets some useful facts about the Machine.
            
            This should be only accessible to the top level admin for security
            reasons.
            
            @param machine:    Machine for which the containers need to be
                               listed.
            @type  machine:    rce.master.machine.Machine
            
            @return:           List of containers.
            @rtype:            List(rce.master.container.Container)
        """
        try:
#            machine = (machine for machine in self._list_machines()
#                       if machineIP == machine.IP).next()
            machine = (machine for machine in self._root._balancer._machines
                       if machineIP == machine.IP).next()
            return {'active':machine.active, 'capacity':machine.capacity}
        except StopIteration:
            raise InternalError('No such machine')
    
#    def _list_machine_users(self, machine): # TODO: Method is never used
#        """ List users using a machine's resources.
#            
#            This should be only accessible to the top level admin for security
#            reasons.
#            
#            @param machine:    Machine for which the containers need to be
#                               listed.
#            @type  machine:    rce.master.machine.Machine
#            
#            @return:           Counter containing the users and the number of
#                               containers they are running .
#            @rtype:            List collections.Counter
#        """
#        return machine._users.keys()
    
    def _list_userID(self):
        """ Gets a list of all users that are logged into the Cloud Engine.
            
            This should be only accessible to the top level admin for security
            reasons.
            
            @return:            List of users.
            @rtype:             list(str)
        """
        return self._root._users.keys()

    def _add_user(self, username, password):
        """ Add a user to the RoboEarth Cloud Engine

            @param username:     Username of the user to be added
            @type  username:     str
    
            @param password:     Password of the user to be added
            @type  passowrd:     str
    
            @return:             Results to True if succeeded.
            @rtype:              boolean
        """
        self._root._checker.addUser(username, password)

    def _remove_user(self, username, password):
        """ Remove a user from the RoboEarth Cloud Engine

            @param username:     Username of the user to be removed
            @type  username:     str
    
            @return:             Results to True if succeeded.
            @rtype:              boolean
        """
        self._root._checker.removeUser(username)

    def _change_password(self, username, new_password, control_mode):
        """ Change the password of a user of the RoboEarth Cloud Engine

            @param username:        Username of the user to be added
            @type  username:        str
            
            @param new_password:    New Password
            @type  new_password:    str
            
            @param control_mode:    in user mode:  old password
                                    in admin mode: True
            @type  control_mode:    in user mode:  str
                                    in admin mode: bool
    
            @return:                 Results to True if succeeded
            @rtype:                  boolean
        """
        self._root._checker.passwd(username, new_password, control_mode)
    
    def _list_user_robots(self, user):
        """ Retrieve a list of all the robots owned by the user.
        
            @param user:         User logged into the cloud engine.
            @type  user:         rce.master.user.User
            
            @return:             List of Robot uuids.
            @rtype:              list(str)
        """
        return user._robots.keys()
    
    def _list_user_containers(self, user):
        """ Retrieve a list of all the containers owned by the user.
        
            @param user:         User logged into the cloud engine.
            @type  user:         rce.master.user.User
            
            @return:             List of Container tags.
            @rtype:              list(str)
        """
        return user._containers.keys()
        
    def _list_user_connections(self, user):
        """ Retrieve a list of all the connections made by the user.
        
            @param user:         User logged into the cloud engine.
            @type  user:         rce.master.user.User
            
            @return:             List of hashed connection keys to
                                 rce.master.network.Connection
            @rtype:              list(str)
        """
        return user._connections.keys()
    
#    def _retrieve_robot_byUUID(self, user, robotID): # TODO: Method is never used
#        """Gets a Robot given the key and user
#        
#            @param user:         User logged into the cloud engine.
#            @type  user:         rce.master.user.User
#            
#            @param robotID:      Unique ID of robot.
#            @type  robotID:      str
#            
#            @return:             Robot object
#            @rtype:              rce.master.user.Robot
#        """
#        return user._robots.get(robotID)
    
    def _retrieve_container_byTag(self, user, tag):
        """Gets a container given the tag
        
            @param user:         User logged into the cloud engine.
            @type  user:         rce.master.user.User
            
            @param tag:          Tag of the container.
            @type  tag:          str
            
            @return:             Container object
            @rtype:              rce.master.user.Container
        """
        return user._containers.get(tag)
    
#    def _retrieve_connection_byHash(self, user, hash_key): # TODO: Method is never used
#        """Gets a connection given the hash key.
#        
#            @param user:         User logged into the cloud engine.
#            @type  user:         rce.master.user.User
#            
#            @param hash_key:     hash_key assosicated to the connection object
#                                 to be fetched
#            @type  hash_key:     str
#            
#            @return:             Container object
#            @rtype:              rce.master.user.Container
#        
#        """
#        return user._connection.get(hash_key)

    # TODO : Please read
    # The above functions expose a bunch of basic objects form the cloud engine,
    # the rest of these methods use the exposed objects in convenient wrappers,
    # feel free to edit and modify these as required alternatively one could
    #use the above to generate the required behavior.

    def generate_user_graph(self, user):
        """Generates a multidict representation of the users connect graph
        across the cloud engine. Useful for visualizing and debugging a users
        activities and connections.
        """
        #build the physical container graph
        def get_container_tree(tag):
            container = self._retrieve_container_byTag(user, tag)
            nodes = container._nodes.keys()
            parameters = container._parameters.keys()
            interfaces = container._interfaces.keys()
            return {tag:{'nodes':nodes, 'parameters':parameters,
                         'interfaces':interfaces}}

        machines = defaultdict(list)
        for tag, container in user._containers.iteritems():
            machines[container._machine.IP].append(get_container_tree(tag))

        #build the robots graph
        robots = {}
        for uuid, robot in user._robots.iteritems():
            robots[uuid] =  robot._interfaces.keys()

        # build the connections map
        connections = InterfaceConnection2way()
        for connection in user._connections.itervalues():
            connections[connection._connectionA._interface._uid] = \
                connection._connectionB._interface._uid

        return {'userID':user._userID, 'network':machines,
                'robots':robots, 'connections': connections}
