#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       ClientMsgTypes.py
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

""" Message Types of RCE Client Protocol:
        
        CS      Create a container
        CSR     Update on container status
        CH      Destroy a container
        
        CC      Change a ROS component (Node, Parameter, Interface)
        CI      register/unregister from an Interface
        
        CM      ROS Message
"""

CREATE_CONTAINER = 'CS'
STATUS_CONTAINER = 'CSR'
DESTROY_CONTAINER = 'CH'

CHANGE_COMPONENT = 'CC'
INTERFACE_STATE = 'CI'

MESSAGE = 'CM'
