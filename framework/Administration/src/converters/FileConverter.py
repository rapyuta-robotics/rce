#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       ImageConverter.py
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

# ROS specific imports
import roslib; roslib.load_manifest('Administration')
from Administration.msg import File

# Python specific imports
import cStringIO

# Custom imports
from ConverterUtility import getName, buildReference, resolveReference

class FileConverter(object):
    """ Convert files from Django style to ROS style and back.
    """
    MESSAGE_TYPE = 'Administration/File'
    
    def decode(self, rosMsgType, data, files):
        """ Convert a file stored in a cStringIO.StrinO object to a ROS
            compatible message (Administration.File).
        """
        fileObj = files[resolveReference(data)]
        
        if not isinstance(fileObj, cStringIO.OutputType):
            raise TypeError('Given object is not a cStringIO.StringO instance.')
        
        return File(content=fileObj.read(), name=getName(data))
    
    def encode(self, rosMsg, basename):
        """ Convert a ROS compatible message (Administration.File) to a
            cStringIO.StrinO object.
        """
        if not isinstance(rosMsg, File):
            raise TypeError('Given object is not a Administration.File instance.')
        
        # Save to StringIO
        fileObj = cStringIO.StringIO()
        fileObj.write(rosMsg.content)
        key = buildReference(rosMsg.name, basename)
        
        return ({ 'name' : rosMsg.name, 'content' : key }, { key : fileObj })
