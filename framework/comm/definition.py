#!/usr/bin/env python
# -*- coding: utf-8 -*-
#     
#     definition.py
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

# Python specific imports
import string
import random
from struct import Struct


####################
# Communication configuration
ADDR_BASE = string.ascii_uppercase
ADDRESS_LENGTH = 6
CHUNK_SIZE = 8192
_MAX_MESSAGE_LENGTH = 10000000 # ~10mbyte
MSG_QUEUE_TIMEOUT = 30


####################
# Character for special addresses
_PUB = '+'
_PRIV = '-'
_NEIGBOR = '$'


####################
# Short sanity checks
iter(ADDR_BASE)

if len(ADDR_BASE) < 5:
    raise ValueError('Address base is to small add some more characters.')

if ADDRESS_LENGTH < 5:
    raise ValueError('Address length is to small.')

if  _PUB == _PRIV:
    raise ValueError('Special char for public addresses is the same as the '
                     'special char for private addresses.')

if  _PUB == _NEIGBOR:
    raise ValueError('Special char for public addresses is the same as the '
                     'special char for neighbor address.')

if  _PRIV == _NEIGBOR:
    raise ValueError('Special char for private addresses is the same as the '
                     'special char for neighbor address.')

if _PUB in ADDR_BASE:
    raise ValueError('Special char for public addresses is already in chars '
                     'for base addresses.')

if _PRIV in ADDR_BASE:
    raise ValueError('Special char for private addresses is already in chars '
                     'for base addresses.')

if _NEIGBOR in ADDR_BASE:
    raise ValueError('Special char for neighbor address is already in chars '
                     'for base addresses.')


####################
# Constants which are used to (de-)serialize header
I_STRUCT = Struct('!I')
I_LEN = I_STRUCT.size
MAX_INT = 2 ** (I_LEN * 8) - 1

# Message length limit to prevent overly large messages
# Absolute maximum is MAX_INT else the message length header has an overflow
MAX_LENGTH = min(_MAX_MESSAGE_LENGTH, MAX_INT)


####################
# Necessary constants for the message
MSG_TYPE_LENGTH = 2

POS_DEST = I_LEN
POS_ORIGIN = POS_DEST + ADDRESS_LENGTH
POS_MSG_TYPE = POS_ORIGIN + ADDRESS_LENGTH
POS_MSG_NUMBER = POS_MSG_TYPE + MSG_TYPE_LENGTH
HEADER_LENGTH = POS_MSG_NUMBER + I_LEN

# Another sanity check
if 3 * HEADER_LENGTH > _MAX_MESSAGE_LENGTH:
    raise ValueError('Maximal message length is too small.')


####################
# Addresses
PREFIX_LENGTH = ADDRESS_LENGTH / 2
SUFFIX_LENGTH = ADDRESS_LENGTH - ADDRESS_LENGTH / 2

# Used prefix for public  nodes
PREFIX_PUB_ADDR = _PUB * PREFIX_LENGTH

# Used prefix for private nodes
PREFIX_PRIV_ADDR = _PRIV * PREFIX_LENGTH

# Used to identify a message which is intended for direct neighbor
NEIGHBOR_ADDR = PREFIX_PRIV_ADDR + _NEIGBOR * SUFFIX_LENGTH

# Used for master/load balancer node
MASTER_ADDR = _PUB * ADDRESS_LENGTH

# Used for ROS node launcher
LAUNCHER_ADDR = _PRIV * ADDRESS_LENGTH

# List of all the special addresses
SPECIAL_ADDRS = [ MASTER_ADDR, LAUNCHER_ADDR, NEIGHBOR_ADDR ]


####################
# Helper functions for addresses
def generateSuffix():
    """ Generate a random suffix.
    """
    return ''.join(random.choice(ADDR_BASE) for _ in xrange(SUFFIX_LENGTH))


def generateAddress():
    """ Generate a random suffix.
    """
    return ''.join(random.choice(ADDR_BASE) for _ in xrange(ADDRESS_LENGTH))


def validateSuffix(suffix):
    """ Validate the given suffix.
                        
        @param addr:    Suffix which should be validated.
        @type  addr:    str
        
        @return:    True if the suffix is valid; False otherwise.
        rtype:      bool
    """
    if len(suffix) != SUFFIX_LENGTH:
        return False
    
    for c in suffix:
        if c not in ADDR_BASE:
            return False
    
    return True


def validateAddress(addr, allowNeighbor=False):
    """ Validate the given address.
                        
        @param addr:    Address which should be validated.
        @type  addr:    str
        
        @param allowNeighbor:   Flag to indicate whether the special address
                                for a neighbor should return True or False.
        @type  allowNeighbor:   bool
        
        @return:    True if the address is valid; False otherwise.
        rtype:      bool
    """
    if len(addr) != ADDRESS_LENGTH:
        return False
    
    if addr == NEIGHBOR_ADDR:
        return allowNeighbor
    
    if addr in SPECIAL_ADDRS:
        return True
    
    prefix = addr[:PREFIX_LENGTH]
    
    if prefix in [PREFIX_PUB_ADDR, PREFIX_PRIV_ADDR]:
        addr = addr[PREFIX_LENGTH:]
    
    for c in addr:
        if c not in ADDR_BASE:
            return False
    
    return True
