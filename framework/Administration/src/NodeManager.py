#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       NodeManager.py
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

class ManagerBase(object):
    """ Base class for all node managers.
    """
    def __init__(self, commManager):
        """ Initialize the basic fields.
            
            @param commManager:     The CommManager instance which is used
                                    in this node for communication.
        """
        self._commManager = commManager
    
    @property
    def reactor(self):
        """ Reference to twisted reactor of this node. """
        return self._commManager.reactor
