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
from twisted.internet.defer import DeferredList
from twisted.spread.pb import Viewable

# rce specific imports
from rce.util.name import validateName, IllegalName
from rce.core.error import InvalidRequest
from rce.core.wrapper import Container
from rce.slave.interface import Types


class ControlView(Viewable):
    """ View implementing all control actions which a user can perform to
        interact with the cloud engine.
    """
    def view_createContainer(self, user, tag, data={}):
        """ Create a new Container object.

            @param user:        User for which the container will be created.
            @type  user:        rce.core.user.User

            @param tag:         Tag which is used to identify the container
                                in subsequent requests.
            @type  tag:         str

            @param data:        Extra data used to configure the container.
            @type  data:        dict
        """
        try:
            validateName(tag)
        except IllegalName as e:
            raise InvalidRequest('Container tag is invalid: {0}'.format(e))

        if tag in user.containers or tag in user.robots:
            raise InvalidRequest('Tag is already used for a container '
                                 'or robot.')

        namespace, remote_container = user.realm.createContainer(user.userID,
                                                                 data)
        container = Container(namespace, remote_container)
        user.containers[tag] = container
        container.notifyOnDeath(user.containerDied)

        m = 'Container {0} successfully created.'.format(tag)
        d = DeferredList([namespace(), remote_container()],
                         fireOnOneErrback=True, consumeErrors=True)
        return d.addCallback(lambda _: m)

    def view_destroyContainer(self, user, tag):
        """ Destroy a Container object.

            @param user:        User for which the container will be destroyed.
            @type  user:        rce.core.user.User

            @param tag:         Tag which is used to identify the container
                                which should be destroyed.
            @type  tag:         str
        """
        try:
            container = user.containers.pop(tag)
        except KeyError:
            raise InvalidRequest('Can not destroy non existent container.')

        container.dontNotifyOnDeath(user.containerDied)
        container.destroy()

        # TODO: Return some info about success/failure of request

    def view_addNode(self, user, cTag, nTag, pkg, exe, args='', name='',
                     namespace=''):
        """ Add a node to a ROS environment.

            @param user:        User for which the node will be added.
            @type  user:        rce.core.user.User

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
            user.containers[cTag].addNode(nTag, pkg, exe, args, name, namespace)
        except KeyError:
            raise InvalidRequest('Can not add Node, because Container {0} '
                                 'does not exist.'.format(cTag))

        # TODO: Return some info about success/failure of request

    def view_removeNode(self, user, cTag, nTag):
        """ Remove a node from a ROS environment.

            @param user:        User for which the node will be destroyed.
            @type  user:        rce.core.user.User

            @param cTag:        Tag which is used to identify the ROS
                                environment from which the node should be
                                removed.
            @type  cTag:        str

            @param nTag:        Tag which is used to identify the ROS node
                                which should removed.
            @type  nTag:        str
        """
        try:
            user.containers[cTag].removeNode(nTag)
        except KeyError:
            raise InvalidRequest('Can not remove Node, because Container {0} '
                                 'does not exist.'.format(cTag))

        # TODO: Return some info about success/failure of request

    def view_addParameter(self, user, cTag, name, value):
        """ Add a parameter to a ROS environment.

            @param user:        User for which the parameter will be added.
            @type  user:        rce.core.user.User

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
            user.containers[cTag].addParameter(name, value)
        except KeyError:
            raise InvalidRequest('Can not add Parameter, because Container '
                                 '{0} does not exist.'.format(cTag))

        # TODO: Return some info about success/failure of request

    def view_removeParameter(self, user, cTag, name):
        """ Remove a parameter from a ROS environment.

            @param user:        User for which the parameter will be destroyed.
            @type  user:        rce.core.user.User

            @param cTag:        Tag which is used to identify the ROS
                                environment from which the parameter should be
                                removed.
            @type  cTag:        str

            @param name:        Name of the parameter which should be removed.
            @type  name:        str
        """
        try:
            user.containers[cTag].removeParameter(name)
        except KeyError:
            raise InvalidRequest('Can not remove Parameter, because Container '
                                 '{0} does not exist.'.format(cTag))

        # TODO: Return some info about success/failure of request

    def view_addInterface(self, user, eTag, iTag, iType, clsName, addr=''):
        """ Add an interface to an endpoint, i.e. a ROS environment or a
            Robot object.

            @param user:        User for which the interface will be added.
            @type  user:        rce.core.user.User

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
                user.robots[eTag].addInterface(iTag, iType, clsName)
            except KeyError:
                raise InvalidRequest('Can not add Interface, because Robot '
                                     '{0} does not exist.'.format(eTag))
        elif iType.endswith('Interface'):
            try:
                user.containers[eTag].addInterface(iTag, iType, clsName, addr)
            except KeyError:
                raise InvalidRequest('Can not add Interface, because '
                                     'Container {0} does not '
                                     'exist.'.format(eTag))
        else:
            raise InvalidRequest('Interface type is invalid (Unknown suffix).')

        # TODO: Return some info about success/failure of request

    def view_removeInterface(self, user, eTag, iTag):
        """ Remove an interface from an endpoint, i.e. a ROS environment or a
            Robot object.

            @param user:        User for which the interface will be destroyed.
            @type  user:        rce.core.user.User

            @param eTag:        Tag which is used to identify the endpoint from
                                which the interface should be removed; either
                                a container tag or robot ID.
            @type  eTag:        str

            @param iTag:        Tag which is used to identify the interface
                                which should be removed.
            @type  iTag:        str
        """
        user.getEndpoint(eTag).removeInterface(iTag)

        # TODO: Return some info about success/failure of request

    def view_addConnection(self, user, tagA, tagB):
        """ Create a connection between two interfaces.

            @param user:        User for which the connection will be created.
            @type  user:        rce.core.user.User

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

        ifA = user.getEndpoint(eTagA).getInterface(iTagA)
        ifB = user.getEndpoint(eTagB).getInterface(iTagB)

        if ifA.clsName != ifB.clsName:
            raise InvalidRequest('Can not connect two interfaces with '
                                 'different message/service type.')

        if not Types.connectable(ifA.iType, ifB.iType):
            raise InvalidRequest('Can not connect an interface of type {0} '
                                 'and an interface of type '
                                 '{1}.'.format(Types.decode(ifA.iType),
                                               Types.decode(ifB.iType)))

        key = int(md5(tagA).hexdigest(), 16) ^ int(md5(tagB).hexdigest(), 16)

        if key in user.connections:
            raise InvalidRequest('Can not add the same connection twice.')

        connection = user.realm.createConnection(ifA.obj, ifB.obj)
        user.connections[key] = connection
        connection.notifyOnDeath(user.connectionDied)

        # TODO: Return some info about success/failure of request

    def view_removeConnection(self, user, tagA, tagB):
        """ Destroy a connection between two interfaces.

            @param user:        User for which the connection will be destroyed.
            @type  user:        rce.core.user.User

            @param tagX:        Tag which is used to identify the interface
                                which should be disconnected. It has to be of
                                the form:
                                    [endpoint tag]/[interface tag]
                                For example:
                                    testRobot/logPublisher
            @type  tagX:        str
        """
        key = int(md5(tagA).hexdigest(), 16) ^ int(md5(tagB).hexdigest(), 16)

        try:
            connection = user.connections.pop(key)
        except KeyError:
                raise InvalidRequest('Can not disconnect two unconnected '
                                     'interfaces.')

        connection.dontNotifyOnDeath(user.connectionDied)
        connection.destroy()

        # TODO: Return some info about success/failure of request


class MonitorView(Viewable):
    """ View implementing all monitor actions which a normal user can perform to
        interact with the cloud engine.
    """
    def view_update_user(self, user, new_pw, old_pw):
        """ Remote call to edit user information.

            @param user:        User for which the login information will be
                                updated.
            @type  user:        rce.core.user.User

            @param new_pw:      The new password
            @type  new_pw:      str

            @param old_pw:      Old password
            @type  old_pw:      str
        """
        user.realm._checker.passwd(user.userID, new_pw, old_pw)

    def view_list_containers(self, user):
        """ Remote call to list containers under the logged-in user.

            @param user:        User for which the running container should be
                                listed.
            @type  user:        rce.core.user.User

            @return:            List of the container tags of running
                                containers.
            @rtype:             [ str ]
        """
        return user.containers.keys()

    def view_list_robots(self, user):
        """ List Robots under the logged in user.

            @param user:        User for which the connected robots should be
                                listed.
            @type  user:        rce.core.user.User

            @return:            List of the robot IDs of connected robots.
            @rtype:             [ str ]
        """
        return user.robots.keys()

    def view_get_rosapi_connect_info(self, user, tag):
        """ Remote call to get ROSAPI request URL and key for a particular
            container.

            @param user:        User who owns the container for which the URL
                                is requested.
            @type  user:        rce.core.user.User

            @param tag:         Tag used to identify the container.
            @type  tag:         str

            @return:            URL which can be used to connect to the rosproxy
                                running inside the container as well as the key
                                necessary to use the rosproxy.
                                (type: (str, str))
            @rtype:             twisted.internet.defer.Deferred
        """
        try:
            container = user.containers[tag]
        except KeyError:
            raise InvalidRequest('Container {0} does not exist.'.format(tag))

        uid = uuid4().hex
        container._obj.registerConsole(user.userID, uid)
        d = container.getConnectInfo()
        d.addCallback(lambda addr: (addr, uid))
        return d


class AdminMonitorView(Viewable):
    """ View implementing all monitor actions which an admin user can perform to
        interact with the cloud engine.
    """
    def view_list_machines(self, user):
        """ Remote call to list machine IPs.
        """
        return [machine.IP for machine in user.realm._balancer._machines]

    def view_machine_containers(self, user, machineIP):
        """ Remote call to list containers in a machine with given IP.

            @param user:        User who requested the container list.
            @type  user:        rce.core.user.User

            @param machineIP:   IP of machine for which the running containers
                                should be listed.
            @type  machineIP:   str

            @return:            List of running containers.
            @rtype:             # TODO: Depends on modification below.
        """
        # TODO: Can not return rce.master.container.Container instances need
        #       some conversion into string, tuple, or some other base type
        try:
            return (machine for machine in user.realm._balancer._machines
                    if machineIP == machine.IP).next()._containers
        except StopIteration:
            raise InvalidRequest('No such machine.')

    def view_stats_machine(self, user, machineIP):
        """ Remote call to list stats of machine with given IP.

            @param user:        User who requested the stats.
            @type  user:        rce.core.user.User

            @param machineIP:   IP of machine for which the stats should be
                                listed.
            @type  machineIP:   str

            @return:            Stats of the machine.
            @rtype:             { str : int }
        """
        try:
            machine = (machine for machine in user.realm._balancer._machines
                       if machineIP == machine.IP).next()
            return {'active':machine.active, 'size':machine.size}
        except StopIteration:
            raise InvalidRequest('No such machine.')

    def view_list_users(self, user):
        """ Remote call to list all users currently logged into
            the RoboEarth Cloud Engine.

            @param user:        User who requested the listing of logged-in
                                users.
            @type  user:        rce.core.user.User

            @return:            List of the user IDs of logged-in users.
            @rtype:             [ str ]
        """
        return user.realm._users.keys()

    def view_add_user(self, user, username, password):
        """ Remote call to add user.

            @param user:        User who requested the addtion of a new user.
            @type  user:        rce.core.user.User

            @param username:    The username of the new user.
            @type  username:    str

            @param password:    The password of the new user.
            @type  password:    str
        """
        user.realm._checker.addUser(username, password)

    def view_remove_user(self, user, username):
        """ Remote call to remove user.

            @param user:        User who requested the removal of the user.
            @type  user:        rce.core.user.User

            @param username:    The username of the user which should be
                                removed.
            @type  username:    str
        """
        user.realm._checker.removeUser(username)

    def view_update_user(self, user, username, password):
        """ Remote call to edit user information.

            @param user:        User who requested the addtion of a new user.
            @type  user:        rce.core.user.User

            @param username:    The username of the user which should be
                                updated.
            @type  username:    str

            @param password:    The new password for the user which should be
                                updated.
            @type  password:    str
        """
        user.realm._checker.passwd(username, password, True)

    def view_list_containers_by_user(self, user, userID):
        """ Remote call to list containers under a given user.

            @param user:        User who requested the listing of the containers
                                of a particular user.
            @type  user:        rce.core.user.User

            @param userID:      The username of the user for which the
                                containers should be listed.
            @type  userID:      str
        """
        return user._realm.getUser(userID).containers.keys()

    def view_list_robots_by_user(self, user, userID):
        """ List robots under the user specified.

            @param user:        User who requested the listing of the robots
                                of a particular user.
            @type  user:        rce.core.user.User

            @param userID:      The username of the user for which the robots
                                should be listed.
            @type  userID:      str
        """
        return user._realm.getUser(userID).robots.keys()
