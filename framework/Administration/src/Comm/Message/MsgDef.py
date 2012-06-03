#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       MsgDef.py
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

# Custom imports
import settings

####################
# Chunk size into which the message is split
CHUNK_SIZE = 8192

####################
# Constants which are used to (de-)serialize booleans / integers
B_STRUCT = Struct('!B')
B_LEN = B_STRUCT.size

I_STRUCT = Struct('!I')
I_LEN = I_STRUCT.size
MAX_INT = 2 ** (I_LEN * 8) - 1

####################
# Message length limit to prevent overly large messages
# Absolute maximum is MAX_INT else the message length header field has an overflow
MAX_LENGTH = min(settings.MAX_LENGTH_MESSAGE, MAX_INT)

####################
# Necessary constants for the message
ADDRESS_LENGTH = 6
MSG_TYPE_LENGTH = 2

POS_DEST = I_LEN
POS_ORIGIN = POS_DEST + ADDRESS_LENGTH
POS_MSG_TYPE = POS_ORIGIN + ADDRESS_LENGTH
POS_MSG_NUMBER = POS_MSG_TYPE + MSG_TYPE_LENGTH
HEADER_LENGTH = POS_MSG_NUMBER + I_LEN

####################
# Special addresses
PREFIX_LENGTH_ADDR = ADDRESS_LENGTH / 2
SUFFIX_LENGTH_ADDR = ADDRESS_LENGTH - ADDRESS_LENGTH / 2

# Used to identify a client which needs an address from server
#NEED_ADDR = '?' * ADDRESS_LENGTH

# Used to identify a message which is intended for direct neighbor
NEIGHBOR_ADDR = '!' * ADDRESS_LENGTH

# Used for master/load balancer node
MASTER_ADDR = '$' * ADDRESS_LENGTH

# Used for DB node
DB_ADDR = '?' * ADDRESS_LENGTH

LAUNCHER_ADDR = '*' * ADDRESS_LENGTH

# Used prefix for satellite nodes
PREFIX_SATELLITE_ADDR = '+' * PREFIX_LENGTH_ADDR

# Used prefix for container manager nodes
PREFIX_CONTAINER_ADDR = '-' * PREFIX_LENGTH_ADDR
