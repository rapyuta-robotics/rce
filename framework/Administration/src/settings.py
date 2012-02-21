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

import roslib
import os.path

# Custom converters for django/piston input to ROS messages.
CONVERTER_CLASSES = (
    'converters.ImageConverter.ImageConverter',
)

# Definition of reference from data to files
REFERENCE_PREFIX = 'ReF'

# Time in seconds after which inactive entries in the databases are removed.
TIMEOUT = 600

# Time in secondes after which unresponsive services are treated as dead.
WAIT_FOR_SERVICE_TIMEOUT = 2

# Path to directory where temporary files to store the results should be kept
TMP_RESULT_DIR = '/var/www/tmp'

# Scanner node
_scanner = {    'name' : 'BarCodeService/Scanner.py',
                'services' : [('Scanner', 'QueryScanner')] }

_webDB = {      'name' : 'BarCodeService/WebDB.py',
                'services' : [('WebDB', 'QueryWebDB')] }

_semantic = {   'name' : 'BarCodeService/Semantic.py',
                'services' : [('Semantic', 'QuerySemantic')] }

pathTo_read_text = roslib.packages.get_pkg_dir('read_text')

_textReader = { 'name' : 'ReadTextService/ReadText',
                'services' : [('ReadText', 'QueryReadText')],
                'config' : [('correlation', 'file', os.path.join(pathTo_read_text, 'fonts/correlation.txt')),
                            ('wordList', 'file', os.path.join(pathTo_read_text, 'dictionary/full-dictionary'))] }

_test = {       'name' : 'Test/Test.py',
                'services' : [('test', 'QueryTest')] }

# Nodes which can be used
NODES = (   _scanner ,
            _webDB,
            _semantic,
            _textReader,
            _test
        )
