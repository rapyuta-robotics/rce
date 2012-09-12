#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       initMachine.py
#       
#       This file is part of the RoboEarth Cloud Engine framework.
#       
#       This file was originally created for RoboEearth
#       http://www.roboearth.org/
#       
#       The research leading to these results has received funding from
#       the European Union Seventh Framework Programme FP7/2007-2013 under
#       grant agreement no248942 RoboEarth.
#       
#       Copyright 2012 RoboEarth
#       
#       Licensed under the Apache License, Version 2.0 (the "License");
#       you may not use this file except in compliance with the License.
#       You may obtain a copy of the License at
#       
#       http://www.apache.org/licenses/LICENSE-2.0
#       
#       Unless required by applicable law or agreed to in writing, software
#       distributed under the License is distributed on an "AS IS" BASIS,
#       WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#       See the License for the specific language governing permissions and
#       limitations under the License.
#       
#       \author/s: Dominique Hunziker 
#       
#       

# Before we start to import everything check if the
# script can be executed
import os

if os.getuid() != 0:
    print('initServer has to be run as super user.')
    exit(1)

try:
    if os.environ['ROS_DISTRO'] != 'fuerte':
        print('ROS distribution has to be fuerte.')
        exit(1)
except KeyError:
    print('ROS environment is not properly set up.')
    exit(1)

# Python specific imports
import sys
from pwd import getpwnam

# twisted specific imports
from twisted.python import log
from twisted.internet.protocol import ProcessProtocol
from twisted.internet.defer import Deferred, DeferredList

# Custom imports
from core.machine import UIDClient

from settings import MASTER_UID_PORT, ROOT_SRC_DIR

class LoggerProtocol(ProcessProtocol):
    """ Simple ProcessProtocol which forwards the stdout of the subprocesses
        to the stdout of this process in a orderly fashion.
    """
    _STOP_ESCALATION = [ ( 'INT',    5),
                         ('TERM',    1),
                         ('KILL', None) ]
    
    def __init__(self, termDeferred):
        """ Initialize the LoggerProtocol.
        """
        self._buff = ''
        self._termDeferred = termDeferred
        self._escalation = 0
        self._stopCall = None
 
    def outReceived(self, data):
        """ Callback which is called by twisted when new data has arrived from
            subprocess.
        """
        self._buff += data
        lines = [line for line in self._buff.split('\n')]
        self._buff = lines[-1]
        
        for line in lines[:-1]:
            log.msg(line)
 
    def processEnded(self, reason):
        """ Callback which is called by twisted when the subprocess has ended.
        """
        if reason.value.exitCode != 0:
            log.msg(reason)
            
        self._escalation = -1
        
        if self._stopCall:
            self._stopCall.cancel()
        
        self._termDeferred.callback(None)
    
    def terminate(self, reactor):
        """ Method which is used to terminate the underlying process.
        """
        if self._escalation != -1 and self.transport:
            escalation = self._STOP_ESCALATION[self._escalation]
            self.transport.signalProcess(escalation[0])
            
            if escalation[1]:
                self._escalation += 1
                self._stopCall = reactor.callLater(escalation[1],
                                                   self.terminate, reactor)

def main(reactor, user, masterIP):
    log.startLogging(sys.stdout)
    
    if not os.path.isdir(ROOT_SRC_DIR):
        print('Root source directory is not a valid directory.')
        exit(1)
    
    containerExe = os.path.join(ROOT_SRC_DIR, 'container.py')
    relayExe = os.path.join(ROOT_SRC_DIR, 'relay.py')
    
    if not os.path.isfile(containerExe):
        print('Root source directory does not contain the file '
              '"container.py".')
        exit(1)
        
    if not os.access(containerExe, os.X_OK):
        print('File "container.py" in root source directory is not '
              'executable.')
        exit(1)
    
    if not os.path.isfile(relayExe):
        print('Root source directory does not contain the file "relay.py".')
        exit(1)
    
    if not os.access(relayExe, os.X_OK):
        print('File "relay.py" in root source directory is not executable.')
        exit(1)
    
    user = getpwnam(user)
    
    deferred = Deferred()
    
    containerDeferred = Deferred()
    containerProtocol = LoggerProtocol(containerDeferred)
    relayDeferred = Deferred()
    relayProtocol = LoggerProtocol(relayDeferred)
    termDeferreds = DeferredList([containerDeferred, relayDeferred])
    
    def callback(suffix):
        try:
            cmd = [containerExe, suffix, masterIP]
            reactor.spawnProcess(containerProtocol, cmd[0], cmd,
                                 env=os.environ) # uid=0, gid=0
            
            cmd = [relayExe, suffix, masterIP]
            reactor.spawnProcess(relayProtocol, cmd[0], cmd, env=os.environ,
                                 uid=user.pw_uid, gid=user.pw_gid)
        except Exception as e:
            log.msg(str(e))
            exit(1)
    
    def errback(errMsg):
        log.msg(errMsg)
        exit(1)
    
    deferred.addCallbacks(callback, errback)
    
    reactor.connectTCP(masterIP, MASTER_UID_PORT, UIDClient(deferred))
    
    def shutdown():
        containerProtocol.terminate(reactor)
        relayProtocol.terminate(reactor)
        return termDeferreds
    
    reactor.addSystemEventTrigger('before', 'shutdown', shutdown)
    
    reactor.run()

def _get_argparse():
    from argparse import ArgumentParser

    parser = ArgumentParser(prog='initMachine',
                            description='Script sets up a machine for the '
                                        'ReCloudEngine.')

    parser.add_argument('user', help='User which should be used to run the '
                                     'relay process.')
    parser.add_argument('masterIP', help='IP address of master process.')
    
    return parser

if __name__ == '__main__':
    from twisted.internet import reactor
    
    parser = _get_argparse()
    args = parser.parse_args()
    
    main(reactor, args.user, args.masterIP)
