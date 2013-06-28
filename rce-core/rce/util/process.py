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
    """ Exception is raised in case the command could not be executed.
    """


class _ProcessProtocol(ProcessProtocol):
    """ Protocol used to retrieve the exit code of a child process.
    """
    def __init__(self, cmd, deferred):
        """ Initialize the ProcessProtocol which is used in the function
            'execute'.

            @param cmd:         Command which has been executed.
            @type  cmd:         str

            @param deferred:    Deferred which will be used to report the
                                return value of the command.
            @type  deferred:    twisted.internet.defer.Deferred
        """
        self.cmd = cmd
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


def execute(cmd, env=None, path=None, reactor=None):
    """ Execute a command using twisted's Process Protocol and returns a
        Deferred firing when the command as terminated.

        @param cmd:         Command which should be executed. It has to be a
                            tuple containing the executable as first argument
                            and all additional arguments which will be passed
                            to the executable.
        @type  cmd:         tuple

        @param env:         Can be used to use custom environment variables to
                            execute the command. If argument is omitted the
                            environment of os.environ is used.
        @type  env:         dict

        @param path:        Path which will be used as working directory to
                            execute the command. If argument is omitted the
                            current directory is used.
        @type  path:        str

        @param reactor:     Reference to twisted's reactor. If argument is
                            omitted the standard twisted reactor is imported and
                            used.
        @type  reactor:     twisted::reactor
    """
    deferred = Deferred()
    protocol = _ProcessProtocol(' '.join(cmd), deferred)

    try:
        reactor.spawnProcess(protocol, cmd[0], cmd, env, path)
    except OSError:
        e = ExecutionError('Command could not be executed.')
        deferred.errback(Failure(e))

    return deferred
