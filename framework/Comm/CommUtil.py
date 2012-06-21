#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       CommUtil.py
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
#       \author/s: Dominique Hunziker <dominique.hunziker@gmail.com> 
#       
#       

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
        if c not in MsgDef.ADDR_BASE:
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
    
    if addr == MsgDef.NEIGHBOR_ADDR:
        return allowNeighbor
    
    if addr in MsgDef.SPECIAL_ADDRS:
        return True
    
    prefix = addr[:MsgDef.PREFIX_LENGTH_ADDR]
    
    if prefix in [MsgDef.PREFIX_PUB_ADDR, MsgDef.PREFIX_PRIV_ADDR]:
        addr = addr[MsgDef.PREFIX_LENGTH_ADDR:]
    
    for c in addr:
        if c not in MsgDef.ADDR_BASE:
            return False
    
    return True
