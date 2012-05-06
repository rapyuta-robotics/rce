#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       ContentDefintion.py
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

# Python specific imports
from struct import Struct

# Constants for (de-)serialization
STRUCT_I = Struct('!I')
LEN_I = STRUCT_I.size

class ContentDefinition(object):
    """ Message Content Identifier of ROS Messages of Reappengine Protocol:

            I   IntParam
            S   StrParam
            F   FloatParam
            B   BoolParam
            X   FileParam
            V   ServiceInterface
            P   PublisherInterface
            C   SubscriberInterface
            N   Node
    """
    __slots__ = ['PARAM_INT', 'PARAM_STR', 'PARAM_FLOAT', 'PARAM_BOOL', 'PARAM_FILE',
                 'INTERFACE_SRV', 'INTERFACE_PUB', 'INTERFACE_SUB', 'NODE']

    PARAM_INT = 'I'
    PARAM_STR = 'S'
    PARAM_FLOAT = 'F'
    PARAM_BOOL = 'B'
    PARAM_FILE = 'X'
    INTERFACE_SRV = 'V'
    INTERFACE_PUB = 'P'
    INTERFACE_SUB = 'C'
    NODE = 'N'
