#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       ParameterMonitor.py
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
#       \author/s: 
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
import ComponentDefinition

class _ParamMonitor(object):
    """ Base class which provides the basic functionalities to handle incoming parameters.
    """
    def __init__(self, parameter):
        """ Add a parameter to the parameter server.
        """
        self._init(parameter.name, parameter.value)
    
    def _init(self, name, value):
        """ Internally used method by the constructor. Necessary for FileParam.
        """
        self._registered = False
        
        self._name = name
        
        try:
            if rospy.has_param(name):
                log.msg('Warning: Parameter already exists.')
            
            rospy.set_param(name, value)
            self._registered = True
        except rospy.ROSException as e:
            raise InternalError('ROS Parameter Server reported an error: {0}'.format(str(e)))
    
    @property
    def name(self):
        """ Name of the parameter. """
        return self._name
    
    def remove(self):
        """ Method which is used to remove the parameter from the parameter server.
        """
        if self._registered:
            try:
                rospy.delete_param(self._name)
            except rospy.ROSException:
                pass
            
            self._registered = False
        
    
    def __del__(self):
        """ Remove a parameter from the parameter server.
        """
        self.remove()

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
    
    def __init__(self, parameter):
        """ Add a parameter to the parameter server.
        """
        self._fileCreated = False
        
        (f, fileName) = _mktempfile()
        f.write(parameter.value)
        f.close()

        if not os.path.isfile(fileName):
            raise InternalError('Could not find file.')

        self._fileCreated = True
        
        self._init(parameter.name, fileName)
    
    def __del__(self):
        """ Remove a parameter from the parameter server.
        """
        if self._fileCreated:
            try:
                os.remove(self._value)
            except OSError:
                pass
        
        super(FileMonitor, self).__del__()
    
