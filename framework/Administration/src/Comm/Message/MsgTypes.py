#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       TypeBase.py
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

""" Message Types of Reappengine Protocol:
        
        AR  Initialization request
        AI  Routing information
        AC  Connection directives
        
        CS  Start container
        CH  Stop container
        CO  Container Status
        
        EC  Create environment
        ED  Destroy environment
        
        LI  Load Information
        
        RA  Add ROS component
        RR  Remove ROS component
        RM  ROS Message
        RB  Response from ROS
        RG  Get ROS Message
"""

INIT_REQUEST = 'AR'
ROUTE_INFO = 'AI'
CONNECT = 'AC'

CONTAINER_START = 'CS'
CONTAINER_STOP = 'CH'
CONTAINER_STATUS = 'CO'

ENV_CREATE = 'EC'
ENV_DESTROY = 'ED'

LOAD_INFO = 'LI'

ROS_ADD = 'RA'
ROS_REMOVE = 'RR'
ROS_MSG = 'RM'
ROS_RESPONSE = 'RB'
ROS_GET = 'RG'
