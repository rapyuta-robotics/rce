#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       Interfaces.py
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

# zope specific imports
from zope.interface import Attribute, Interface

class IROSConverter(Interface):
    """ Interface which declares the necessary methods which all ROS message types
        converters have to implement.
    """
    MESSAGE_TYPE = Attribute("""
    Identifier which is used to select the ROS converter.
    """)
    
    def encode(rosMsg): #@NoSelf
        """ Generate dictionary from a ROS message.

            @param rosMsg:  The ROS message instance which should be converted.
            @type  rosMsg:  ROS message instance

            @return:    Dictionary containing the parsed message. The basic form does map
                        each field in the ROS message to a key / value pair in the returned
                        data dict. Binaries are added as StringIO instances.
            @rtype:     {}

            @raise:     TypeError, ValueError
        """
    
    def decode(rosMsgType, data): #@NoSelf
        """ Generate a ROS message from dictionary.

            @param rosMsgType:  String defining the ROS message type, i.e.
                                std_msgs/String.
            @type  rosMsgType:  str

            @param data:    Dictionary with keys matching the fields in
                            the desired ROS message.
                            Binary files should be included as StringIO instances.
            @param data:    { str : {} }

            @return:    ROS message of type @param rosMsgType containing the
                        given data.
            @rtype:     ROS message of type @param rosMsgType

            @raise:     TypeError, ValueError
        """
