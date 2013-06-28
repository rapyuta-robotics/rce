#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     rce-core/rce/monitor/node.py
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
import os
import shlex
from uuid import uuid4

# twisted specific imports
from twisted.python import log
from twisted.internet.error import ProcessExitedAlready
from twisted.internet.protocol import ProcessProtocol
from twisted.spread.pb import Referenceable

# rce specific imports
from rce.monitor.common import ArgumentMixin


class NodeProtocol(ProcessProtocol):
    """ Node process protocol.

        It is used to monitor the health of a node and logging the stdout and
        stderr to files.
    """
    def __init__(self, monitor, out, err):
        self._monitor = monitor

        self._out = open(out, 'w')
        self._err = open(err, 'w')

        # Overwrite method from base class
        self.outReceived = self._out.write
        self.errReceived = self._err.write

    def connectionMade(self):
        self._monitor.started()

    def processEnded(self, reason):
        self._monitor.stopped(reason.value.exitCode)
        self._monitor = None

    def __del__(self):
        self._out.close()
        self._err.close()


class Node(Referenceable, ArgumentMixin):
    """ Representation of a ROS Node (process) inside an environment.
    """
    # CONFIG
    _STOP_ESCALATION = [('INT', 15), ('TERM', 2), ('KILL', None)]
    _LOG_DIR = '/opt/rce/data'  # TODO: After splitting process: '/home/ros'

    def __init__(self, owner, pkg, exe, args, name, namespace):
        """ Initialize and start the Node.

            @param owner:       Environment in which the node will be created.
            @type  owner:       rce.environment.Environment

            @param pkg:         Name of ROS package where the node can be
                                found.
            @type  pkg:         str

            @param exe:         Name of executable (node) which should be
                                launched.
            @type  exe:         str

            @param args:        Additional arguments which should be used for
                                the launch. Can contain the directives
                                $(find PKG) and/or $(env VAR). Other special
                                characters such as '$' or ';' are not allowed.
            @type  args:        str

            @param name:        Name of the node under which the node should be
                                launched.
            @type  name:        str

            @param namespace:   Namespace in which the node should be started
                                in the environment.
            @type  namespace:   str

            @raise:             rce.util.loader.ResourceNotFound
        """
        ArgumentMixin.__init__(self, owner.loader)

        owner.registerNode(self)
        self._owner = owner

        self._reactor = owner.reactor
        self._call = None
        self._protocol = None

        # Find and validate executable
        cmd = [self._loader.findNode(pkg, exe)]  # raises ResourceNotFound

        # Add arguments
        args = self.processArgument(args)

        # TODO: Is it necessary to limit the possible characters here?
#        for char in '$;':
#            if char in args:
#                raise ValueError('Argument can not contain special '
#                                 "character '{0}'.".format(char))

        cmd += shlex.split(args)

        # Process name and namespace argument
        if name:
            cmd.append('__name:={0}'.format(name))

        if namespace:
            cmd.append('__ns:={0}'.format(namespace))

        # Create protocol instance
        uid = uuid4().hex
        out = os.path.join(self._LOG_DIR,
                           '{0}-{1}-out.log'.format(uid, name or exe))
        err = os.path.join(self._LOG_DIR,
                           '{0}-{1}-err.log'.format(uid, name or exe))

        self._protocol = NodeProtocol(self, out, err)

        # Start node
        log.msg('Start Node {0}/{1} [pkg: {2}, exe: '
                '{3}].'.format(namespace, name or exe, pkg, exe))
        self._reactor.spawnProcess(self._protocol, cmd[0], cmd, env=os.environ)

        self._name = '{0}/{1}'.format(pkg, exe)

    def started(self):
        """ Callback for NodeProtocol to signal that the process has started.
        """

    def stopped(self, exitCode):
        """ Callback for NodeProtocol to signal that the process has died.
        """
        self._protocol = None

        if self._call:
            self._call.cancel()

        if exitCode:
            log.msg('Node ({0}) terminated with exit code: '
                    '{1}'.format(self._name, exitCode))

        if self._owner:
            self._owner.unregisterNode(self)
            self._owner = None

    def remote_destroy(self):
        """ Method should be called to stop/kill this node.
        """
        self._destroy()

    def _destroy(self, lvl=0):
        if not self._protocol:
            return

        cmd, delay = self._STOP_ESCALATION[lvl]

        try:
            self._protocol.transport.signalProcess(cmd)
        except ProcessExitedAlready:
            return

        if delay is not None:
            self._call = self._reactor.callLater(delay, self._destroy, lvl + 1)
        else:
            self._call = None
