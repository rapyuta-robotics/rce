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

# zope specific imports
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
