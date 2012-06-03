#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       ParameterMonitor.py
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

# ROS specific imports
import rospy

# twisted specific imports
from twisted.python import log

# Python specific imports
import os
import tempfile

# Custom imports
from Exceptions import InternalError
import ComponentDefinition #@UnresolvedImport

class _ParamMonitor(object):
    """ Base class which provides the basic functionalities to handle incoming parameters.
    """
    def __init__(self, name, value):
        """ Add a parameter to the parameter server.
        """
        self._name = name
        
        try:
            if rospy.has_param(name):
                log.msg('Warning: Parameter already exists.')
            
            rospy.set_param(name, value)
        except rospy.ROSException as e:
            raise InternalError('ROS Parameter Server reported an error: {0}'.format(str(e)))
    
    def __del__(self):
        """ Remove a parameter from the parameter server.
        """
        try:
            rospy.delete_param(self._name)
        except rospy.ROSException:
            pass

class IntMonitor(_ParamMonitor):
    """ Class which allows to handle an integer parameter.
    """
    IDENTIFIER = ComponentDefinition.PARAM_INT

class StrMonitor(_ParamMonitor):
    """ Class which allows to handle a string parameter.
    """
    IDENTIFIER = ComponentDefinition.PARAM_STR

class FloatMonitor(_ParamMonitor):
    """ Class which allows to handle a float parameter.
    """
    IDENTIFIER = ComponentDefinition.PARAM_FLOAT

class BoolMonitor(_ParamMonitor):
    """ Class which allows to handle a bool parameter.
    """
    IDENTIFIER = ComponentDefinition.PARAM_BOOL

def _mktempfile(directory=None):
    """ Wrapper around tempfile.mkstemp function such that a file object
        is returned and not a int. Make sure to close the file object again.
    """
    (fd, fname) = tempfile.mkstemp(dir=directory)
    return (os.fdopen(fd, 'wb'), fname)

class FileMonitor(_ParamMonitor):
    """ Class which allows to handle a file parameter.
    """
    IDENTIFIER = ComponentDefinition.PARAM_FILE
    
    def __init__(self, name, value):
        """ Add a parameter to the parameter server.
        """
        self._fileCreated = False
        
        (f, fileName) = _mktempfile()
        f.write(value)
        f.close()

        if not os.path.isfile(fileName):
            raise InternalError('Could not find file.')

        self._fileCreated = True
        
        super(FileMonitor, self).__init__(name, fileName)
    
    def __del__(self):
        """ Remove a parameter from the parameter server.
        """
        if self._fileCreated:
            try:
                os.remove(self._value)
            except OSError:
                pass
        
        super(FileMonitor, self).__del__()
    