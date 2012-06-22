#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       MsgTypes.py
#       
#       This file is part of the RoboEarth Cloud Engine framework.
#       
#       This file was originally created for RoboEearth
#       http://www.roboearth.org/
#       
#       The research leading to these results has received funding from
#       the European Union Seventh Framework Programme FP7/2007-2013 under
#       grant agreement no248942 RoboEarth.
#       
#       Copyright 2012 RoboEarth
#       
#       Licensed under the Apache License, Version 2.0 (the "License");
#       you may not use this file except in compliance with the License.
#       You may obtain a copy of the License at
#       
#       http://www.apache.org/licenses/LICENSE-2.0
#       
#       Unless required by applicable law or agreed to in writing, software
#       distributed under the License is distributed on an "AS IS" BASIS,
#       WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#       See the License for the specific language governing permissions and
#       limitations under the License.
#       
#       \author/s: Dominique Hunziker 
#       
#       

""" Message Types of RCE Protocol:
        
        AR  Initialization request
        AI  Routing information
        AC  Connection directives
        
        DR  Database request
        DB  Database response
        
        CS  Start container
        CH  Stop container
        CO  Container Status
        
        EC  Create environment
        ED  Destroy environment
        
        IR  CommID request
        IB  CommID response
        ID  CommID delete
        
        LI  Load Information
        
        RA  Add ROS component
        RR  Remove ROS component
        RU  Add/Remove user to/from an Interface
        RM  ROS Message
        RB  Response from ROS
        RG  Get ROS Message
"""

INIT_REQUEST = 'AR'
ROUTE_INFO = 'AI'
CONNECT = 'AC'

DB_REQUEST = 'DR'
DB_RESPONSE = 'DB'

CONTAINER_START = 'CS'
CONTAINER_STOP = 'CH'
CONTAINER_STATUS = 'CO'

ENV_CREATE = 'EC'
ENV_DESTROY = 'ED'

ID_REQUEST = 'IR'
ID_RESPONSE = 'IB'
ID_DEL = 'ID'

LOAD_INFO = 'LI'

ROS_ADD = 'RA'
ROS_REMOVE = 'RR'
ROS_USER = 'RU'
ROS_MSG = 'RM'
ROS_RESPONSE = 'RB'
ROS_GET = 'RG'
