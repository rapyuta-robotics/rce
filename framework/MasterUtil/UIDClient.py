#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       UIDClient.py
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

# twisted specific imports
from twisted.internet.error import ConnectionDone
from twisted.internet.protocol import ClientFactory
from twisted.protocols.basic import LineReceiver

# Custom imports
import settings
from Comm.CommUtil import validateSuffix

if settings.USE_SSL:
    from SSLUtil import createKey, createCertReq, dumpCertReq

class UIDClientProtocol(LineReceiver):
    """ Protocol which is used by a client to retrieve a new UID for a machine.
    """
    def __init__(self, deferred):
        """ Initialize UIDClientProtocol.
            
            @param deferred:    Deferred which should be called with the received UID.
            @type  deferred:    Deferred
        """
        self._deferred = deferred
        self._buff = []
        
        self._uid = None
        
        if settings.USE_SSL:
            self._key = createKey()
        else:
            self._key = None
    
    def lineReceived(self, line):
        """ Callback which is called by twisted when a line has been received.
        """
        if not self._uid:
            if not validateSuffix(line):
                raise ValueError('Received address suffix is not valid.')
            
            self._uid = line
            
            if settings.USE_SSL:
                self.transport.write(
                    '{0}{1}{1}'.format(
                        dumpCertReq(
                            createCertReq(
                                self._key,
                                'Server-{0}'.format(line)
                            )
                        ),
                        self.delimiter
                    )
                )
        else:
            self._buff.append(line)
    
    def connectionLost(self, reason):
        """ Callback which is called by twisted when the connection is lost.
        """
        if reason.check(ConnectionDone):
            self._deferred.callback((self._uid, '\n'.join(self._buff), self._key))
        else:
            self._deferred.errback(reason.getErrorMessage())

class UIDClientFactory(ClientFactory):
    """ Factory which is used by a client to retrieve a new UID for a machine.
    """
    def __init__(self, deferred):
        """ Initialize UIDClientFactory.
            
            @param deferred:    Deferred which should be called with the reveived UID.
            @type  deferred:    Deferred
        """
        self._deferred = deferred
    
    def buildProtocol(self, addr):
        """ Callback which is called by twisted when a new connection should be established.
        """
        return UIDClientProtocol(self._deferred)
