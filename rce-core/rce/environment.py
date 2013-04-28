#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     rce-core/rce/environment.py
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
import fcntl

# ROS specific imports
import rospy

# twisted specific imports
from twisted.python import log
from twisted.spread.pb import PBClientFactory

# rce specific imports
from rce.util.error import InternalError
from rce.util.loader import Loader
from rce.monitor.node import Node
from rce.monitor.parameter import Parameter
from rce.monitor.interface.environment import PublisherInterface, \
    SubscriberInterface, ServiceClientInterface, ServiceProviderInterface
from rce.slave.endpoint import Endpoint
from rce.slave.namespace import Namespace
from rce.slave.interface import Types


class Environment(Namespace):
    """ Representation of the namespace in the environment process, which is
        part of the cloud engine internal communication.
    """
    def __init__(self, endpoint):
        """ Initialize the Environment.

            @param endpoint:    Environment Client which is responsible for
                                monitoring the environment in this process.
            @type  endpoint:    rce.robot.EnvironmentClient
        """
        Namespace.__init__(self, endpoint)

        interface_map = {
            Types.encode('PublisherInterface') : PublisherInterface,
            Types.encode('SubscriberInterface') : SubscriberInterface,
            Types.encode('ServiceClientInterface') : ServiceClientInterface,
            Types.encode('ServiceProviderInterface') : ServiceProviderInterface
        }
        self._map.update(interface_map)

        self._nodes = set()
        self._parameters = set()

    def registerNode(self, node):
        assert node not in self._nodes
        self._nodes.add(node)

    def unregisterNode(self, node):
        assert node in self._nodes
        self._nodes.remove(node)
        self._endpoint.referenceDied('nodeDied', node)

    def registerParameter(self, parameter):
        assert parameter not in self._parameters
        self._parameters.add(parameter)

    def unregisterParameter(self, parameter):
        assert parameter in self._parameters
        self._parameters.remove(parameter)
        self._endpoint.referenceDied('parameterDied', parameter)

    def remote_createNode(self, pkg, exe, args, name, namespace):
        """ Create a Node object in the environment namespace and
            therefore in the endpoint.

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
        return Node(self, pkg, exe, args, name, namespace)

    def remote_createParameter(self, name, value):
        """ Create a Parameter object in the environment namespace and
            therefore in the endpoint.

            @param name:        Name of the parameter which should be added.
            @type  name:        str

            @param value:       Value of the parameter which should be added.
            @type  value:       str, int, float, bool, list
        """
        return Parameter(self, name, value)

    def remote_destroy(self):
        """ Method should be called to destroy the environment and will take
            care of destroying all objects owned by this Environment as well
            as deleting all circular references.
        """
        for node in self._nodes.copy():
            node.remote_destroy()

        for parameter in self._parameters.copy():
            parameter.remote_destroy()

        # Can not check here, because nodes are unregistered when the
        # node (process) exits and remote_destroy only requests to stop the
        # node (process)
        #assert len(self._nodes) == 0
        assert len(self._parameters) == 0

        Namespace.remote_destroy(self)


class EnvironmentClient(Endpoint):
    """ Environment client is responsible for the cloud engine components
        inside a container.
    """
    def __init__(self, reactor, commPort):
        """ Initialize the Environment Client.

            @param reactor:     Reference to the twisted reactor used in this
                                robot process.
            @type  reactor:     twisted::reactor

            @param commPort:    Port where the server for the cloud engine
                                internal communication will listen for incoming
                                connections.
            @type  commPort:    int
        """
        Endpoint.__init__(self, reactor, Loader(), commPort)

        self._dbFile = '/opt/rce/data/rosenvbridge.db' # TODO: Hardcoded?

    def createEnvironment(self, _):
        """ Create the Environment namespace.
        """
        if self._namespaces:
            raise InternalError('The environment can have only one namespace '
                                'at a time.')

        environment = Environment(self)
        return self._avatar.callRemote('setupNamespace', environment)

    def remote_addUsertoROSProxy(self, userID, key):
        """ Method to add username and key to rosproxy-environment bridge
            file that maintains list of users that can call functions of
            rosproxy.

            @param userID:      Username
            @type  userID:      str

            @param key:         Secret key
            @type  key:         str
        """
        # TODO: Should this be deferred to a separate thread due to flock,
        #       which is a blocking call?
        with open(self._dbFile, "a") as bridgefile:
            fcntl.flock(bridgefile.fileno(), fcntl.LOCK_EX)
            bridgefile.write('{0}:{1}\n'.format(userID, key))


def main(reactor, cred, masterIP, masterPort, commPort, uid):
    f = open('/opt/rce/data/env.log', 'w') # TODO: Use os.getenv('HOME') ?
    log.startLogging(f)

    rospy.init_node('RCE_Master')
    print 'connect to ', masterIP, masterPort

    factory = PBClientFactory()
    reactor.connectTCP(masterIP, masterPort, factory)

    client = EnvironmentClient(reactor, commPort)

    def terminate():
        reactor.callFromThread(client.terminate)
        reactor.callFromThread(reactor.stop)

    rospy.on_shutdown(terminate)

    def _err(reason):
        print(reason)
        terminate()

    d = factory.login(cred, (client, uid))
    d.addCallback(client.registerAvatar)
    d.addCallback(client.createEnvironment)
    d.addErrback(_err)

    reactor.run(installSignalHandlers=False)

    f.close()
