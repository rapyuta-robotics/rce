#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     rce-core/rce/util/process.py
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

from twisted.python.failure import Failure
from twisted.internet.defer import Deferred
from twisted.internet.protocol import ProcessProtocol


class ExecutionError(Exception):
    """
    """


class _ProcessProtocol(ProcessProtocol):
    """ Protocol used to retrieve the exit code of a child process.
    """
    def __init__(self, cmd, deferred):
        self.cmd
        self.deferred = deferred

    def processEnded(self, reason):
        """ Method is called by the twisted framework once the child process has
            terminated.
        """
        if reason.value.exitCode == 0:
            self.deferred.callback("Command '{0}' successfully "
                                   'executed.'.format(self.cmd))
        else:
            e = ExecutionError("Execution of '{0}' failed: "
                               'Received exit code '
                               '{1}.'.format(self.cmd, reason.value.exitCode))
            self.deferred.errback(Failure(e))


def execute(cmd, env={}, path=None, reactor=None):
    """
    """
    deferred = Deferred()
    protocol = _ProcessProtocol(' '.join(cmd), deferred)

    try:
        reactor.spawnProcess(protocol, cmd[0], cmd, env, path)
    except OSError:
        e = ExecutionError('Command could not be executed.')
        deferred.errback(Failure(e))

    return deferred
