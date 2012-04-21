#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       MiscUtility.py
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

########################################################################

import string
import random

# Characters which are used to build the ID
_CHARS = string.ascii_letters

def generateID(length=10):
    """ Generate a (hopefully) unique ID consisting of the characters
        [a-zA-Z] and the given length.
    """
    return ''.join(random.choice(_CHARS) for x in xrange(length))

########################################################################

import os
import tempfile

def mktempfile(dir=None):
    """ Wrapper around tempfile.mkstemp function such that a file object
        is returned and not a int. Make sure to close the file object again.
    """
    (fd, fname) = tempfile.mkstemp(dir=dir)
    return (os.fdopen(fd, 'wb'), fname)
