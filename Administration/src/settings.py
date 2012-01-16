#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       settings.py
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

# Custom converters for django/piston input to ROS messages.
CONVERTER_CLASSES = (
    'converters.ImageConverter.ImageConverter',
)

# Definition of reference from data to files
REFERENCE_PREFIX = 'REF:'

# Time in seconds after which inactive entries in the databases are removed.
TIMEOUT = 600

# Time in secondes after which unresponsive services are treated as dead.
WAIT_FOR_SERVICE_TIMEOUT = 2

# Scanner node
_scanner = {    'name' : 'ROS_Services/Scanner.py',
                'services' : [('Scanner', 'QueryScanner')] }

_webDB = {      'name' : 'ROS_Services/WebDB.py',
                'services' : [('WebDB', 'QueryWebDB')] }

_semantic = {   'name' : 'ROS_Services/Semantic.py',
                'services' : [('Semantic', 'QuerySemantic')] }

# Nodes which can be used
NODES = ( _scanner , _webDB, _semantic )

