#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       ComponentDefintion.py
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

""" Message Content Identifier of ROS Messages of Reappengine Protocol:
        
        n   Node (for removal)
        i   Interface (for removal)
        p   Parameter (for removal)
        
        N   Node

        I   IntParam
        S   StrParam
        F   FloatParam
        B   BoolParam
        X   FileParam
        
        V   ServiceInterface
        P   PublisherInterface
        C   SubscriberInterface
"""

RM_NODE = 'n'
RM_INTERFACE = 'i'
RM_PARAMETER = 'p'

NODE = 'N'

PARAM_INT = 'I'
PARAM_STR = 'S'
PARAM_FLOAT = 'F'
PARAM_BOOL = 'B'
PARAM_FILE = 'X'

INTERFACE_SRV = 'V'
INTERFACE_PUB = 'P'
INTERFACE_SUB = 'C'

