#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       initSlave.py
#       
#       Copyright 2012 dominique hunziker <dominique.hunziker@gmail.com>
#       
#       This program is free software; you can redistribute it and/or modify
#       it under the terms of the GNU General Public License as published by
#       the Free Software Foundation; either version 2 of the License, or
#       (at your option) any later version.
#       
#       This program is distributed in the hope that it will be useful,
#       but WITHOUT ANY WARRANTY; without even the implied warranty of
#       MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#       GNU General Public License for more details.
#       
#       You should have received a copy of the GNU General Public License
#       along with this program; if not, write to the Free Software
#       Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#       MA 02110-1301, USA.
#       
#       

# twisted specific imports
from twisted.python import log
from twisted.internet.protocol import ProcessProtocol
from twisted.internet.defer import Deferred

# Python specific imports
import os, sys

# Custom imports
import settings
from Comm.UIDClient import UIDClientFactory #@UnresolvedImport

class LoggerProtocol(ProcessProtocol):
    """ Simple ProcessProtocol which forwards the stdout of the subprocesses to the
        stdout of this process in a orderly fashion.
    """
    def __init__(self):
        """ Initialize the LoggerProtocol.
        """
        self._buff = ''
 
    def outReceived(self, data):
        """ Callback which is called by twisted when new data has arrived from subprocess.
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

def main(reactor):
    log.startLogging(sys.stdout)
    
    deferred = Deferred()
    
    def callback(uid):
        cmd = [ '/usr/bin/python',
                os.path.join(settings.ROOT_DIR, 'Container.py'),
                uid ]
        reactor.spawnProcess(LoggerProtocol(), cmd[0], cmd, env=os.environ, uid=0, gid=0)
        
        cmd = [ '/usr/bin/python',
                os.path.join(settings.ROOT_DIR, 'Satellite.py'),
                uid,
                settings.IP_MASTER, settings.PORT_MASTER ]
        reactor.spawnProcess(LoggerProtocol(), cmd[0], cmd, env=os.environ, uid=1000, gid=1000)
    
    def errback(errMsg):
        print errMsg
        reactor.stop()
    
    deferred.addCallbacks(callback, errback)
    
    reactor.connectTCP(settings.IP_MASTER, settings.PORT_UID, UIDClientFactory(deferred))
    reactor.run()

if __name__ == '__main__':
    from twisted.internet import reactor
    main(reactor)
