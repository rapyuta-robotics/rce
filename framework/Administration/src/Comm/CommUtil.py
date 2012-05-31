#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       CommUtil.py
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
import string

# Custom imports
from Message import MsgDef

def validateSuffix(suffix):
    """ Validate the given suffix.
                        
        @param addr:    Suffix which should be validated.
        @type  addr:    str
        
        @return:    True if the suffix is valid; False otherwise.
        rtype:      bool
    """
    if len(suffix) != MsgDef.SUFFIX_LENGTH_ADDR:
        return False
    
    for c in suffix:
        if c not in string.ascii_uppercase:
            return False
    
    return True

def validateAddress(addr, allowNeighbor=False):
    """ Validate the given address.
                        
        @param addr:    Address which should be validated.
        @type  addr:    str
        
        @param allowNeighbor:   Flag to indicate whether the special address for
                                a neighbor should return True or False.
        @type  allowNeighbor:   bool
        
        @return:    True if the address is valid; False otherwise.
        rtype:      bool
    """
    if len(addr) != MsgDef.ADDRESS_LENGTH:
        return False
    
    if addr == MsgDef.MASTER_ADDR:
        return True
    
    if addr == MsgDef.NEIGHBOR_ADDR:
        return allowNeighbor
    
    prefix = addr[:MsgDef.PREFIX_LENGTH_ADDR]
    
    if prefix == MsgDef.PREFIX_SATELLITE_ADDR:
        addr = addr[MsgDef.PREFIX_LENGTH_ADDR:]
    elif prefix == MsgDef.PREFIX_CONTAINER_ADDR:
        addr = addr[MsgDef.PREFIX_LENGTH_ADDR:] 
    
    for c in addr:
        if c not in string.ascii_uppercase:
            return False
    
    return True
