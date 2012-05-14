#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       ProcessorBase.py
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

class ProcessorBase(object):
    """ Base class which declares the necessary methods which all processor subclasses
        have to overwrite.
        
        The Processor classes will be instantiated once and will be used for all
        received messages. 
    """
    IDENTIFIER = None
    
    def __init__(self, manager):
        """ @param manager:     Manager which is used in this node.
            @type  manager:     ReappengineManager
        """
        self.manager = manager
    
    def processMessage(self, msg):
        """ Process the message.
            
            All references which are necessary to process the message have to available
            as instance variables as the arguments of this method can not be changed.
        """
