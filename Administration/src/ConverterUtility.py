#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       ConverterUtility.py
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

import settings

def resolveReference(key):
    """ Resolve the reference.
        
        @param key:     Key which should be resolved.
        @type  key:     str
        
        @return:    The modified key for retrieving the data from files.
        @rtype:     str
        
        @raise:     ValueError if the given key does not match the reference
                    structure.
    """
    if settings.REFERENCE_PREFIX != key[:len(settings.REFERENCE_PREFIX)]:
        raise ValueError('Data field does not contain a reference to files.')
    
    return key[len(settings.REFERENCE_PREFIX):]

def buildReference(key, basename):
    """ Generate a reference.
        
        @param key:     Name of the field which is referenced.
        @type  key:     str
        
        @param basename:    Basename which should be used to generate the
                            reference.
        @type  basename:    str
        
        @return:        Reference which should be used.
        @rtype:         str
    """
    return '{0}{1}{2}'.format(settings.REFERENCE_PREFIX, basename, key)
