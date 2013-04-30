#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     rce-core/rce/core/environment.py
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

# rce specific imports
from rce.util.error import InternalError
from rce.core.base import Proxy
from rce.core.network import Endpoint, Namespace, EndpointAvatar


class Node(Proxy):
    """ Representation of a node (ROS process) inside a ROS environment.
    """
    def __init__(self, namespace):
        """ Initialize the Node.

            @param namespace:   Namespace in which the node was created.
            @type  namespace:   rce.core.network.Namespace
        """
        super(Node, self).__init__()

        self._namespace = namespace
        namespace.registerNode(self)

    def destroy(self):
        """ Method should be called to destroy the node and will take care
            of deleting all circular references.
        """
        self._namespace.unregisterNode(self)
        self._namespace = None

        super(Node, self).destroy()


class Parameter(Proxy):
    """ Representation of a parameter inside a ROS environment.
    """
    def __init__(self, namespace):
        """ Initialize the Parameter.

            @param namespace:   Namespace in which the parameter was created.
            @type  namespace:   rce.core.network.Namespace
        """
        super(Parameter, self).__init__()

        self._namespace = namespace
        namespace.registerParameter(self)

    def destroy(self):
        """ Method should be called to destroy the parameter and will take
            care of deleting all circular references.
        """
        self._namespace.unregisterParameter(self)
        self._namespace = None

        super(Parameter, self).destroy()


class Environment(Namespace):
    """ Representation of a namespace which has a ROS environment assigned and
        is part of the cloud engine internal communication.
    """
    def __init__(self, endpoint):
        """ Initialize the Environment.

            @param endpoint:    Endpoint in which the environment was created.
            @type  endpoint:    rce.core.network.Endpoint
        """
        super(Environment, self).__init__(endpoint)

        self._nodes = set()
        self._parameters = set()

    def createNode(self, pkg, exe, args, name, nspace):
        """ Create a node (ROS process) inside the environment.

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

            @param nspace:      Namespace in which the node should be started
                                in the environment.
            @type  nspace:      str
        """
        node = Node(self)
        self.callRemote('createNode', pkg, exe, args, name,
                        nspace).chainDeferred(node)
        return node

    def createParameter(self, name, value):
        """ Create a parameter (in ROS parameter server) inside the
            environment.

            @param name:        Name of the parameter which should be added.
                                It is also used to identify the parameter in
                                subsequent requests.
            @type  name:        str

            @param value:       Value of the parameter which should be added.
            @type  value:       str, int, float, bool, list
        """
        parameter = Parameter(self)
        self.callRemote('createParameter', name, value).chainDeferred(parameter)
        return parameter

    def getAddress(self):
        """ Get the address of the endpoint of the environment namespace.
        """
        return self._endpoint.getAddress()

    def registerNode(self, node):
        assert node not in self._nodes
        self._nodes.add(node)

    def registerParameter(self, parameter):
        assert parameter not in self._parameters
        self._parameters.add(parameter)

    def unregisterNode(self, node):
        assert node in self._nodes
        self._nodes.remove(node)

    def unregisterParameter(self, parameter):
        assert parameter in self._parameters
        self._parameters.remove(parameter)

    def registerConsole(self, userID, key):
        """ Register a console user with the environment.
        """
        self._endpoint.registerConsole(userID, key)

    def destroyNode(self, remoteNode):
        """ Method should be called to destroy the node proxy referenced by the
            remote node namespace.

            @param remoteNode:  Reference to Node Namespace in Environment
                                process.
            @type  remoteNode:  twisted.spread.pb.RemoteReference
        """
        for node in self._nodes:
            if node.destroyExternal(remoteNode):
                break

    def destroyParameter(self, remoteParam):
        """ Method should be called to destroy the parameter proxy referenced by
            the remote parameter namespace.

            @param remoteParam: Reference to Parameter Namespace in Environment
                                process.
            @type  remoteParam: twisted.spread.pb.RemoteReference
        """
        for parameter in self._parameters:
            if parameter.destroyExternal(remoteParam):
                break

    def destroy(self):
        """ Method should be called to destroy the environment and will take
            care of destroying all objects owned by this Environment as well
            as deleting all circular references.
        """
        for node in self._nodes.copy():
            node.destroy()

        for parameter in self._parameters.copy():
            parameter.destroy()

        assert len(self._nodes) == 0
        assert len(self._parameters) == 0

        super(Environment, self).destroy()


class EnvironmentEndpoint(Endpoint):
    """ Representation of an endpoint which is a process that lives inside a
        container and is part of the cloud engine internal communication.
    """
    def __init__(self, network, container):
        """ Initialize the Environment Endpoint.

            @param network:     Network to which the endpoint belongs.
            @type  network:     rce.core.network.Network

            @param container:   Container in which the enpoint is living.
            @type  container:   rce.core.container.Container
        """
        super(EnvironmentEndpoint, self).__init__(network)

        self._container = container

    def getAddress(self):
        """ Get the address of the environment endpoint's internal
            communication server.

            @return:            Address of the environment endpoint's internal
                                communication server.
                                (type: twisted.internet.address.IPv4Address)
            @rtype:             twisted.internet.defer.Deferred
        """
        return self._container.getAddress()

    def registerConsole(self, userID, key):
        self.callRemote('addUsertoROSProxy', userID, key)

    def unregisterConsole(self, userID, key):
        self.callRemote('removeUserfromROSProxy', userID)

    def createNamespace(self):
        """ Create a Environment object in the environment endpoint.

            @return:            New Environment instance.
            @rtype:             rce.master.environment.Environment
                                (subclass of rce.core.base.Proxy)
        """
        if self._namespaces:
            raise InternalError('Can not have more than one namespace '
                                'in an Environment endpoint at a time.')

        return Environment(self)

    def registerRemoteEnvironment(self, remoteNamespace):
        """ Register a Namespace object of the endpoint.

            @param remoteNamespace: Reference to Environment namespace in
                                    Environment process.
            @type  remoteNamespace: twisted.spread.pb.RemoteReference
        """
        # TODO: Workaround for now...
        try:
            iter(self._namespaces).next().callback(remoteNamespace)
        except StopIteration:
            pass

    def destroyNode(self, remoteNode):
        """ Method should be called to destroy the node proxy referenced by the
            remote node namespace.

            @param remoteNode:  Reference to Node Namespace in Environment
                                process.
            @type  remoteNode:  twisted.spread.pb.RemoteReference
        """
        # TODO: Workaround for now...
        try:
            iter(self._namespaces).next().destroyNode(remoteNode)
        except StopIteration:
            pass

    def destroyParameter(self, remoteParam):
        """ Method should be called to destroy the parameter proxy referenced by
            the remote parameter namespace.

            @param remoteParam: Reference to Parameter Namespace in Environment
                                process.
            @type  remoteParam: twisted.spread.pb.RemoteReference
        """
        # TODO: Workaround for now...
        try:
            iter(self._namespaces).next().destroyParameter(remoteParam)
        except StopIteration:
            pass

    def destroy(self):
        """ Method should be called to destroy the endpoint and will take care
            of destroying all objects owned by this Endpoint as well as
            deleting all circular references.
        """
        print('Destroying Connection to Environment Process.')
        self._container = None
        super(EnvironmentEndpoint, self).destroy()


class EnvironmentEndpointAvatar(EndpointAvatar):
    """ Avatar for internal PB connection from an Environment Endpoint.
    """
    def perspective_setupNamespace(self, remoteNamespace):
        """ Register a namespace with the Master process.

            @param remoteNamespace: Reference to the Namesapce in the slave
                                    process.
            @type  remoteNamespace: twisted.spread.pb.RemoteReference
        """
        self._endpoint.registerRemoteEnvironment(remoteNamespace)

    def perspective_nodeDied(self, remoteNode):
        """ Notify that a remote node died.

            @param remoteNode:  Reference to the Node in the Environment
                                process.
            @type  remoteNode:  twisted.spread.pb.RemoteReference
        """
        self._endpoint.destroyNode(remoteNode)

    def perspective_parameterDied(self, remoteParameter):
        """ Notify that a remote parameter died.

            @param remoteParam: Reference to the Parameter in the Environment
                                process.
            @type  remoteParam: twisted.spread.pb.RemoteReference
        """
        self._endpoint.destroyParameter(remoteParameter)
