#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       RoboearthBackend.py
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

from django.contrib.auth.backends import RemoteUserBackend
import httplib
import urllib

class RoboearthBackend(RemoteUserBackend):
    """ This backend is to be used in conjunction with the 'RemoteUserMiddleware'
        found in the middleware module of this package, and is used when the server
        is handling authentication outside of Django.
        
        The authentication is done via a POST request to a remote webserver.
    """
    _KEY = 'HC2s8q9aUXop1WnKYn4zJxCWUVimGLiOC0r9hUxRWSO8wtStUp'
    _HOST = 'localhost:75'
    _URI = '/login/'
    
    def authenticate(self, username=None, password=None):
        """ Authenticates a user using the remote webserver '_HOST'.
            Returns a User object if the authentication was successful
            and False otherwise.
        """
        body = urllib.urlencode([('key', RoboearthBackend._KEY), ('username', username), ('password', password)])
        headers = { 'Content-Type': 'application/x-www-form-urlencoded' }
        
        conn = httplib.HTTPConnection(RoboearthBackend._HOST)
        conn.request('POST', RoboearthBackend._URI, body, headers)
        r = conn.getresponse()
    
        if r.status == 200:
            return super(RoboearthBackend, self).authenticate(username)
        
        return None
