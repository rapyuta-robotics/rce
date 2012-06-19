#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       initServer.py
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

# twisted specific imports
from twisted.python import log
from twisted.internet.protocol import ProcessProtocol
from twisted.internet.defer import Deferred, DeferredList

# Python specific imports
import sys
import shutil
from pwd import getpwnam

# Custom imports
import settings
from Comm.Message import MsgDef
from MasterUtil.UIDClient import UIDClientFactory

if settings.USE_SSL:
    from SSLUtil import createKeyCertPair, writeCertToFile, writeKeyToFile, RCEClientContext

class LoggerProtocol(ProcessProtocol):
    """ Simple ProcessProtocol which forwards the stdout of the subprocesses to the
        stdout of this process in a orderly fashion.
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
                self._stopCall = reactor.callLater(escalation[1], self.terminate, reactor)

def main(reactor, caFileName):
    log.startLogging(sys.stdout)
    
    if not os.path.isdir(settings.ROOT_SRC_DIR):
        print('Root source directory is not a valid directory.')
        exit(1)
    
    containerExe = os.path.join(settings.ROOT_SRC_DIR, 'Container.py')
    serverExe = os.path.join(settings.ROOT_SRC_DIR, 'Server.py')
    
    if not os.path.isfile(containerExe):
        print('Root source directory does not contain the file "Container.py".')
        exit(1)
        
    if not os.access(containerExe, os.X_OK):
        print('File "Container.py" in root source directory is not executable.')
        exit(1)
    
    if not os.path.isfile(serverExe):
        print('Root source directory does not contain the file "Server.py".')
        exit(1)
    
    if not os.access(serverExe, os.X_OK):
        print('File "Server.py" in root source directory is not executable.')
        exit(1)
    
    deferred = Deferred()
    
    containerDeferred = Deferred()
    containerProtocol = LoggerProtocol(containerDeferred)
    serverDeferred = Deferred()
    serverProtocol = LoggerProtocol(serverDeferred)
    termDeferreds = DeferredList([containerDeferred, serverDeferred])
    
    def callback((suffix, cert, key)):
        try:
            if settings.USE_SSL:
                # Create necessary directories
                serverPath = os.path.join(settings.SSL_DIR, 'server')
                containerPath = os.path.join(settings.SSL_DIR, 'container')
                
                # Change access and ownership of directories
                os.mkdir(serverPath)
                os.chmod(serverPath, 0700)
                os.chown(serverPath, 1000, 1000)
                
                os.mkdir(containerPath)
                os.chmod(containerPath, 0700)
                
                # Add clean up of directories
                def cleanUp(result):
                    shutil.rmtree(serverPath, True)
                    shutil.rmtree(containerPath, True)
                    return result
                
                termDeferreds.addCallbacks(cleanUp, cleanUp)
                
                # Save the received, signed certificate and key for the server (connection to the master)
                with open(os.path.join(serverPath, 'toMaster.cert'), 'w') as f:
                    f.write(cert)
                
                writeKeyToFile(key, os.path.join(serverPath, 'toMaster.key'))
                
                # Create a certificate to sign certificates for communication between server and container node
                (caCert, caKey) = createKeyCertPair('Machine')
                
                writeCertToFile(caCert, os.path.join(settings.SSL_DIR, 'Machine.cert'))
                
                # Create and save the certificate for the server (connection to the container)
                (cert, key) = createKeyCertPair( MsgDef.PREFIX_PUB_ADDR + suffix,
                                                 caCert,
                                                 caKey )
                
                writeCertToFile(cert, os.path.join(serverPath, 'toContainer.cert'))
                writeKeyToFile(key, os.path.join(serverPath, 'toContainer.key'))
                
                # Create and save the certificate for the container (connection to the server)
                (cert, key) = createKeyCertPair( MsgDef.PREFIX_PRIV_ADDR + suffix,
                                                 caCert,
                                                 caKey )
                
                writeCertToFile(cert, os.path.join(containerPath, 'toServer.cert'))
                writeKeyToFile(key, os.path.join(containerPath, 'toServer.key'))
                
                # Create a certificate to sign certificates for communication between server and environment nodes
                (caCert, caKey) = createKeyCertPair('Container')
                
                writeCertToFile(caCert, os.path.join(settings.SSL_DIR, 'Container.cert'))
                writeKeyToFile(caKey, os.path.join(containerPath, 'env.key'))
                
                # Create and save the certificate for the server (connection to environments)
                (cert, key) = createKeyCertPair( MsgDef.PREFIX_PUB_ADDR + suffix,
                                                 caCert,
                                                 caKey )
                
                writeCertToFile(cert, os.path.join(serverPath, 'toEnv.cert'))
                writeKeyToFile(key, os.path.join(serverPath, 'toEnv.key'))
            
            cmd = [containerExe, suffix]
            reactor.spawnProcess(containerProtocol, cmd[0], cmd, env=os.environ) # uid=0, gid=0
            
            user = getpwnam(settings.SERVER_USER)
            cmd = [serverExe, suffix, settings.IP_MASTER]
            reactor.spawnProcess(serverProtocol, cmd[0], cmd, env=os.environ, uid=user.pw_uid, gid=user.pw_gid)
        except Exception as e:
            log.msg(str(e))
            reactor.stop()
    
    def errback(errMsg):
        log.msg(errMsg)
        print errMsg
        reactor.stop()
    
    deferred.addCallbacks(callback, errback)
    
    if settings.USE_SSL:
        reactor.connectSSL( settings.IP_MASTER,
                            settings.PORT_UID,
                            UIDClientFactory(deferred),
                            RCEClientContext(caFileName) )
    else:
        reactor.connectTCP(settings.IP_MASTER, settings.PORT_UID, UIDClientFactory(deferred))
    
    def shutdown():
        containerProtocol.terminate(reactor)
        serverProtocol.terminate(reactor)
        return termDeferreds
    
    reactor.addSystemEventTrigger('before', 'shutdown', shutdown)
    
    reactor.run()

def _get_argparse():
    from argparse import ArgumentParser

    parser = ArgumentParser(prog='initServer',
                            description='Script which is used to set up a machine for the ReCloudEngine.')

    parser.add_argument('--cert', help='Path to the certificate file which is used to connect with the Master.', default='')
    
    return parser

if __name__ == '__main__':
    from twisted.internet import reactor
    
    parser = _get_argparse()
    args = parser.parse_args()
    
    if settings.USE_SSL and not os.path.isfile(args.cert):
        raise ValueError('Certificate path is not a valid file.')
    
    main(reactor, args.cert)
