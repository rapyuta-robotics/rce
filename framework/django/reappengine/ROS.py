#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       setupROS.py
#       
#       Copyright 2011 dominique hunziker <dominique.hunziker@gmail.com>
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

# Copy of relevant bash commands to python

########################################################################
### Adjust the (ROS) environment here
########################################################################

# Custom path for ROS packages
_SOURCE_FOR_CUSTOM_PACKAGES = '/home/dominique/ROS'

# Default ROS configuration
_EXPORTS = [('ROS_ROOT', '/opt/ros/electric/ros'),
            ('PATH', '${ROS_ROOT}/bin:${PATH}'),
            ('PYTHONPATH', '${ROS_ROOT}/core/roslib/src:${PYTHONPATH}'),
            ('ROS_PACKAGE_PATH', '{0}:/opt/ros/electric/stacks'.format(_SOURCE_FOR_CUSTOM_PACKAGES)),
            ('ROS_HOSTNAME', 'localhost'),
            ('ROS_MASTER_URI', 'http://localhost:11311')]

########################################################################
### Do not change below here
########################################################################

import re
import os
import sys

def uniqify(pathList):
    """ Remove all duplicates from a list whilst keeping the ordering """
    newPathList = []
    
    for path in pathList:
        if path not in newPathList:
            newPathList.append(path)
    
    return newPathList

_regex = re.compile('\$\{(\w+)\}')

for (envVar, rawValue) in _EXPORTS:
    matches = _regex.finditer(rawValue)
    value = rawValue
    
    for match in matches:
        value = value.replace(match.group(), os.environ.get(match.group(1), ''))
        value = value.strip(':')
    
    os.environ[envVar] = ':'.join(uniqify(value.split(':')))
    
    # Special case for the PYTHONPATH variable:
    if envVar == 'PYTHONPATH':
        for path in value.split(':'):
            sys.path.append(path)
        
        sys.path = uniqify(sys.path)

########################################################################
