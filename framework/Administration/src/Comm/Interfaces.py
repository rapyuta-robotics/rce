#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       Interfaces.py
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

from zope.interface import Interface

class IPostInitTrigger(Interface):
    """ Interface which declares the necessary method which all post init triggers
        have to implement.
    """
    def trigger(origin, ip): #@NoSelf
        """ This method is called whenever the PostInitTrigger should be triggered.
            
            @param origin:  CommID of request origin.
            @type  origin:  str
            
            @param ip:      IP address of request origin.
            @type  ip:      str
        """

class IServerImplementation(Interface):
    """ Interface which declares the necessary methods which all server implementations
        have to implement.
    """
    def authOrigin(origin, ip, key): #@NoSelf
        """ Authenticate the origin using the attached key from the InitRequest.
            
            @param origin:  CommID of request origin.
            @type  origin:  str
            
            @param ip:      IP address of the request origin.
            @type  ip:      str
            
            @param key:     Key which was sent with the request.
            @type  key:     str
            
            @return:        Return True if the origin was successfully authenticated.
        """
    
    def getResponse(origin): #@NoSelf
        """ Generate the dictionary for the returned InitRequest message.
            
            This method has to be implemented.
            
            @param origin:  CommID of request origin.
            @type  origin:  str
            
            @return:        Dictionary containing all necessary fields for the InitRequest
                            message:    dest, origin, key
        """
    
    def saveState(content): #@NoSelf
        """ Save the state of the connection. This method is called after the response
            message has been successfully sent and before the connection is set to
            initialized.
            
            @param content:     Previously constructed message content.
            @type  content:     dict
        """
    
    def unregisterConnection(conn): #@NoSelf
        """ This method should be called to remove any references to the given connection.
            
            @param conn:    Protocol instance who should be unregistered.
            @type  conn:    ReappengineProtocol
        """

class IUIDServer(Interface):
    """ Interface which declares the necessary method which all UID server implementations
        have to implement.
    """
    def getUID(): #@NoSelf
        """ Return a unique ID which can be used as a communication ID.
                        
            @return:    Valid string which can be used to connect to the server.
        """
