#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       SemanticBase.py
#       
#       Copyright 2011 dominique hunziker <dominique.hunziker@gmail.com>
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

from __future__ import print_function

from Worker import ObjectWorker

class SemanticError(Exception):
    """ General Exception in connection with a Semantic.
    """
    pass

class SemanticBase(ObjectWorker):
    """ Generic base class for all Semantics interfaces. You should not
        instantiate this class directly.
    """
    
    DATA_FIELDS = { 'data' : [{ 'product' : None,
                    'type' : None,
                    'description' : None,
                    'additional_info' : None,
                    'language' : None }] }
    
    def run(self):
        """ Overwrites the necessary method from the base class.
            This method processes the raw data by calling the method
            process which has to be overwitten.
        """
        if len(self.data) == 0:
            self.warning('No data to process.')
            return
        
        self.process()
        
        self.appendData(self.result)
    
    def process(self):
        """ Overwrite this method, else a NotImplementedError is raised.
            This function is used to process the rawData from a Task
            object.
        """
        raise NotImplementedError('The method process for the used Semantics is not implemented.')
