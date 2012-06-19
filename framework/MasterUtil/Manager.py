#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       Manager.py
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

# zope specific imports
from zope.interface import implements

# twisted specific imports
from twisted.python import log

# Python specific imports
from datetime import datetime, timedelta
import random

# Custom imports
import settings
from Exceptions import InternalError
from NodeManager import ManagerBase
from Comm.Message import MsgDef
from Interfaces import IUIDServer
from Type import ConnectDirectiveMessage, GetCommIDRequestMessage, GetCommIDResponseMessage, DelCommIDRequestMessage
from Processor import GetCommIDProcessor, DelCommIDProcessor

if settings.USE_SSL:
    from SSLUtil import parseCertReqStr, createCert, dumpCert

class MasterManager(ManagerBase):
    """ Manager which is used for master node who is responsible for the management
        of the servers nodes and is the connection point for the outside world.
    """
    implements(IUIDServer)
    
    def __init__(self, commManager, cert, key):
        """ Initialize the necessary variables for the MasterManager.
            
            @param commManager:     CommManager which should be used to communicate.
            @type  commManager:     CommManager
            
            @param cert:    Certificate which should be used to sign the new
                            certificates (only used for SSL communication).
            @type  cert:    crypto.X509
            
            @param key:     Key for certificate (only used for SSL communication).
            @type  key:     crypto.PKey
        """
        super(MasterManager, self).__init__(commManager)
        
        # List of all available servers
        # Key:   CommID
        # Value: IP address of the server
        self._servers = {}
        
        # Dictionary of currently reserved, but not yet confirmed UIDs
        # Key:   UID
        # Value: Timestamp with expiration date of UID
        self._tmpUIDs = {}
        
        # List of used commIDs
        self._commIDs = set()
        
        # Register Content Serializers
        self._commManager.registerContentSerializers([ ConnectDirectiveMessage(),
                                                       GetCommIDRequestMessage(),
                                                       GetCommIDResponseMessage(),
                                                       DelCommIDRequestMessage() ])
        
        # Register Message Processors
        self._commManager.registerMessageProcessors([ GetCommIDProcessor(self, commManager),
                                                      DelCommIDProcessor(self, commManager) ])
        
        if settings.USE_SSL:
            self._cert = cert
            self._key = key
    
    def getUID(self):
        """ Callback method which provides a new UID for a machine.
            
            @return:    Unique ID which should be used to build the CommID for Server and
                        Container Node.
            @rtype:     str
        """
        while 1:
            uid = ''.join(random.choice(MsgDef.ADDR_BASE) for _ in xrange(MsgDef.SUFFIX_LENGTH_ADDR))
            
            if '{0}{1}'.format(MsgDef.PREFIX_PUB_ADDR, uid) not in self._servers \
                and uid not in self._tmpUIDs:
                break
        
        self._tmpUIDs[uid] = datetime.now()
        
        return uid
    
    def getCertificate(self, req):
        """ Callback method which provides a new certificate for a machine.
            
            @param req:     X509Req object as a string.
            @type  req:     str
            
            @return:        Resulting X509 object as a string.
            @rtype:         str
        """
        try:
            return dumpCert(createCert(parseCertReqStr(req), self._cert, self._key))
        except AttributeError:
            raise InternalError('Tried to get a new SSL certificate, but configuration does not support SSL.')
    
    def checkUID(self, uid):
        """ Check if the given UID is in the list of not yet confirmed UIDs.
            
            @param uid:     UID which should be verified.
            @type  uid:     str
            
            @return:        True if the UID is valid; False otherwise
            @rtype:         bool
        """
        return uid in self._tmpUIDs
    
    def addServer(self, commID, ip):
        """ Callback method for the factory to register a server connection.
        """
        log.msg('Register server with ID: "{0}"; IP: "{1}"'.format(commID, ip))
        self._servers[commID] = ip
        del self._tmpUIDs[commID[MsgDef.PREFIX_LENGTH_ADDR:]]
    
    def removeServer(self, commID):
        """ Callback method for the factory to unregister a server connection.
        """
        if commID in self._servers:
            # First add the UID again to the list of not yet confirmed UIDs in case
            # the server just lost the connection and did not leave intentionally
            self._tmpUIDs[commID[MsgDef.PREFIX_LENGTH_ADDR:]] = datetime.now()
            del self._servers[commID]
    
    def getServers(self):
        """ Get a list of all available servers and their connection information.
            
            @return:    List of connection information of all available servers, where
                        connection information a dictionary is with the keys ip, commID.
            @rtype:     [ { str : str } ]
        """
        return [{ 'commID' : commID, 'ip' : ip } for commID, ip in self._servers.iteritems()]
    
    def getCommID(self):
        """ Callback method which provides a new CommID for a container.
            
            @return:    Unique ID which should be used as CommID for the container.
            @rtype:     str
        """
        while 1:
            commID = ''.join(random.choice(MsgDef.ADDR_BASE) for _ in xrange(MsgDef.ADDRESS_LENGTH))
            
            if commID not in self._commIDs:
                self._commIDs.add(commID)
                log.msg('Created CommID "{0}".'.format(commID))
                return commID
    
    def delCommID(self, commID):
        """ Callback method which provides a new CommID for a container.
            
            @param commID:  Unique ID which should be removed from list of used CommIDs.
            @type  commID:  str
        """
        try:
            self._commIDs.remove(commID)
        except KeyError:
            pass
        else:
            log.msg('Destroyed CommID "{0}".'.format(commID))
    
    def clean(self):
        """ Method is regularly called to free no longer claimed unique server IDs.
        """
        limit = datetime.now() - timedelta(seconds=settings.UID_TIMEOUT)
        
        for uid in [uid for uid, timestamp in self._tmpUIDs.iteritems() if timestamp > limit]:
            del self._tmpUIDs[uid]
