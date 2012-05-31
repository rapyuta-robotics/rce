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
import string

# Custom imports
import settings
from Comm.Message import MsgDef
from Comm.Interfaces import IUIDServer #@UnresolvedImport
from Type import ConnectDirectiveMessage, GetCommIDRequestMessage, GetCommIDResponseMessage, DelCommIDRequestMessage #@UnresolvedImport

class MasterManager(object):
    """ Manager which is used for master node who is responsible for the management
        of the satellites nodes and is the connection point for the outside world.
    """
    implements(IUIDServer)
    
    def __init__(self, commMngr):
        """ Initialize the necessary variables for the MasterManager.
            
            @param commMngr:    CommManager which should be used to communicate.
            @type  commMngr:    CommManager
        """
        # References used by the manager
        self._commMngr = commMngr
        
        # List of all available satellites
        # Key:   CommID
        # Value: IP address of the satellite
        self._satellites = {}
        
        # Dictionary of currently reserved, but not yet confirmed UIDs
        # Key:   UID
        # Value: Timestamp with expiration date of UID
        self._tmpUIDs = {}
        
        # List of used commIDs
        self._commIDs = set()
        
        # Register Content Serializers
        self._commMngr.registerContentSerializers([ ConnectDirectiveMessage(),
                                                    GetCommIDRequestMessage(),
                                                    GetCommIDResponseMessage(),
                                                    DelCommIDRequestMessage() ])
        # TODO: Check if all these Serializers are necessary
        
        # Register Message Processors
        #self._commMngr.registerMessageProcessors([ ConnectDirectiveProcessor(self) ])
        # TODO: Add all valid messages  
    
    def getUID(self):
        """ Callback method which provides a new UID for a machine.
            
            @return:    Unique ID which should be used to build the CommID for Satellite and
                        Container Node.
            @rtype:     str
        """
        while 1:
            uid = ''.join(random.choice(string.ascii_uppercase) for _ in xrange(MsgDef.SUFFIX_LENGTH_ADDR))
            
            if '{0}{1}'.format(MsgDef.PREFIX_SATELLITE_ADDR, uid) not in self._satellites \
                and uid not in self._tmpUIDs:
                break
        
        self._tmpUIDs[uid] = datetime.now()
        
        return uid
    
    def checkUID(self, uid):
        """ Check if the given UID is in the list of not yet confirmed UIDs.
            
            @param uid:     UID which should be verified.
            @type  uid:     str
            
            @return:        True if the UID is valid; False otherwise
            @rtype:         bool
        """
        return uid in self._tmpUIDs
    
    def addSatellite(self, commID, ip):
        """ Callback method for the factory to register a satellite connection.
        """
        log.msg('Register satellite with ID: "{0}"; IP: "{1}"'.format(commID, ip))
        self._satellites[commID] = ip
        del self._tmpUIDs[commID[MsgDef.PREFIX_LENGTH_ADDR:]]
    
    def removeSatellite(self, commID):
        """ Callback method for the factory to unregister a satellite connection.
        """
        if commID in self._satellites:
            # First add the UID again to the list of not yet confirmed UIDs in case
            # the satellite just lost the connection and did not leave intentionally
            self._tmpUIDs[commID[MsgDef.PREFIX_LENGTH_ADDR:]] = datetime.now()
            del self._satellites[commID]
    
    def getSatellites(self):
        """ Get a list of all available satellites and their connection information.
            
            @return:    List of connection information of all available satellites, where
                        connection information a dictionary is with the keys ip, commID.
            @rtype:     [ { str : str } ]
        """
        return [{ 'commID' : commID, 'ip' : ip } for commID, ip in self._satellites.iteritems()]
    
    def getCommID(self):
        """ Callback method which provides a new CommID for a container.
            
            @return:    Unique ID which should be used as CommID for the container.
            @rtype:     str
        """
        while 1:
            commID = ''.join(random.choice(string.ascii_uppercase) for _ in xrange(MsgDef.ADDRESS_LENGTH))
            
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
        """ Method is regularly called to free no longer claimed unique satellite IDs.
        """
        limit = datetime.now() - timedelta(seconds=settings.UID_TIMEOUT)
        
        for uid in [uid for uid, timestamp in self._tmpUIDs.iteritems() if timestamp > limit]:
            del self._tmpUIDs[uid]
