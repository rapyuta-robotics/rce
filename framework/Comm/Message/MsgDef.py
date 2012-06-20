#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       MsgDef.py
#       
#       This file is part of the RoboEarth Cloud Engine framework.
#       
#       This file was originally created for RoboEearth - http://www.roboearth.org/
#       The research leading to these results has received funding from the European Union 
#       Seventh Framework Programme FP7/2007-2013 under grant agreement no248942 RoboEarth.
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
#       \author/s: 
#       
#       

# Python specific imports
from struct import Struct

# Custom imports
from settings import ADDR_BASE
from settings import ADDRESS_LENGTH
from settings import CHUNK_SIZE #@UnusedImport
from settings import MAX_LENGTH_MESSAGE as _MAX_LENGTH_MESSAGE

####################
# Character for special addresses
_PUB = '+'
_PRIV = '-'
_DB = '?'
_NEIGBOR = '$'

####################
# Short sanity checks

iter(ADDR_BASE)

if len(ADDR_BASE) < 5:
    raise ValueError('Address base is to small add some more characters.')

if ADDRESS_LENGTH < 5:
    raise ValueError('Address length is to small.')

if  _PUB == _PRIV:
    raise ValueError('Special char for public addresses is the same as the special char for private addresses.')

if  _PUB == _DB:
    raise ValueError('Special char for public addresses is the same as the special char for database address.')

if  _PUB == _NEIGBOR:
    raise ValueError('Special char for public addresses is the same as the special char for neighbor address.')

if  _PRIV == _DB:
    raise ValueError('Special char for private addresses is the same as the special char for database address.')

if  _PRIV == _NEIGBOR:
    raise ValueError('Special char for private addresses is the same as the special char for neighbor address.')

if  _DB == _NEIGBOR:
    raise ValueError('Special char for database address is the same as the special char for neighbor address.')

if _PUB in ADDR_BASE:
    raise ValueError('Special char for public addresses is already in chars for base addresses.')

if _PRIV in ADDR_BASE:
    raise ValueError('Special char for private addresses is already in chars for base addresses.')

if _DB in ADDR_BASE:
    raise ValueError('Special char for database address is already in chars for base addresses.')

if _NEIGBOR in ADDR_BASE:
    raise ValueError('Special char for neighbor address is already in chars for base addresses.')

####################
# Constants which are used to (de-)serialize header
I_STRUCT = Struct('!I')
I_LEN = I_STRUCT.size
MAX_INT = 2 ** (I_LEN * 8) - 1

# Message length limit to prevent overly large messages
# Absolute maximum is MAX_INT else the message length header field has an overflow
MAX_LENGTH = min(_MAX_LENGTH_MESSAGE, MAX_INT)

####################
# Necessary constants for the message
MSG_TYPE_LENGTH = 2

POS_DEST = I_LEN
POS_ORIGIN = POS_DEST + ADDRESS_LENGTH
POS_MSG_TYPE = POS_ORIGIN + ADDRESS_LENGTH
POS_MSG_NUMBER = POS_MSG_TYPE + MSG_TYPE_LENGTH
HEADER_LENGTH = POS_MSG_NUMBER + I_LEN

# Another sanity check
if 3 * HEADER_LENGTH > _MAX_LENGTH_MESSAGE:
    raise ValueError('Maximal message length is too small.')

####################
# Addresses
PREFIX_LENGTH_ADDR = ADDRESS_LENGTH / 2
SUFFIX_LENGTH_ADDR = ADDRESS_LENGTH - ADDRESS_LENGTH / 2

# Used prefix for public  nodes
PREFIX_PUB_ADDR = _PUB * PREFIX_LENGTH_ADDR

# Used prefix for private nodes
PREFIX_PRIV_ADDR = _PRIV * PREFIX_LENGTH_ADDR

# Used for DB node
DB_ADDR = PREFIX_PUB_ADDR + _DB * SUFFIX_LENGTH_ADDR

# Used to identify a message which is intended for direct neighbor
NEIGHBOR_ADDR = PREFIX_PRIV_ADDR + _NEIGBOR * SUFFIX_LENGTH_ADDR

# Used for master/load balancer node
MASTER_ADDR = PREFIX_PUB_ADDR + _PUB * SUFFIX_LENGTH_ADDR

# Used for ROS node launcher
LAUNCHER_ADDR = PREFIX_PRIV_ADDR + _PRIV * SUFFIX_LENGTH_ADDR

# List of all the special addresses
SPECIAL_ADDRS = [ MASTER_ADDR, LAUNCHER_ADDR, NEIGHBOR_ADDR, DB_ADDR ]
