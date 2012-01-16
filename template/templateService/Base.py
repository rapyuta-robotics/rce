#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       %Service%Base.py
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

from __future__ import print_function

###
### Select desired Worker type
###
from Worker import ObjectWorker as Worker
#from Worker import ThreadWorker as Worker

class %Service%Base(Worker):
    
    DATA_FIELDS={ ### Select the fields which are needed. ### }
    
    ###
    ### Example
    ###
    ### DATA_FIELDS = { 'number' : None, 'image' : { 'img' : None, 'name' : None } }
    ###
    ### -> self.number = ...
    ### -> self.image = { 'img' : ..., 'name' : ... }
    ###
    
    def run(self):
        ###
        ### Add your code here.
        ###
        ### Use self.appendData('data') to add some data which will be
        ### used to compile the response of the service.
        ###
