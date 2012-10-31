#!/usr/bin/env python
# -*- coding: utf-8 -*-
#     
#     cmd.py
#     
#     This file is part of the RoboEarth Cloud Engine framework.
#     
#     This file was originally created for RoboEearth
#     http://www.roboearth.org/
#     
#     The research leading to these results has received funding from
#     the European Union Seventh Framework Programme FP7/2007-2013 under
#     grant agreement no248942 RoboEarth.
#     
#     Copyright 2012 RoboEarth
#     
#     Licensed under the Apache License, Version 2.0 (the "License");
#     you may not use this file except in compliance with the License.
#     You may obtain a copy of the License at
#     
#     http://www.apache.org/licenses/LICENSE-2.0
#     
#     Unless required by applicable law or agreed to in writing, software
#     distributed under the License is distributed on an "AS IS" BASIS,
#     WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#     See the License for the specific language governing permissions and
#     limitations under the License.
#     
#     \author/s: Dominique Hunziker 
#     
#     


""" Message Content Identifier of Command Messages of RCE Protocol:
        
        -   Container (for removal)
        r   Robot (for removal)
        n   Node (for removal)
        i   Interface (for removal)
        p   Parameter (for removal)
        
        +   Container
        R   Robot
        
        N   Node

        Z   Standard Parameter
        Y   Standard Parameter Array
        X   FileParam
        
        V   Service Interface
        K   Service Provider Interface
        P   Publisher Interface
        C   Subscriber Interface
        
        W   Service Converter
        L   Service Provider Converter
        Q   Publisher Converter
        D   Subscriber Converter
        
        U   Service Forwarder
        I   Service Provider Forwarder
        O   Publisher Forwarder
        B   Subscriber Forwarder
        
        A   Connection
"""

RM_CONTAINER = '-'
RM_ROBOT = 'r'
RM_NODE = 'n'
RM_INTERFACE = 'i'
RM_PARAMETER = 'p'

CONTAINER = '+'
ROBOT = 'R'

NODE = 'N'

PARAM_STD = 'Z'
PARAM_ARR = 'Y'
PARAM_FILE = 'X'

INTERFACE_SRV = 'V'
INTERFACE_PRO = 'K'
INTERFACE_PUB = 'P'
INTERFACE_SUB = 'C'

CONVERTER_SRV = 'W'
CONVERTER_PRO = 'L'
CONVERTER_PUB = 'Q'
CONVERTER_SUB = 'D'

FORWARDER_SRV = 'U'
FORWARDER_PRO = 'I'
FORWARDER_PUB = 'O'
FORWARDER_SUB = 'B'

CONNECTION = 'A'
