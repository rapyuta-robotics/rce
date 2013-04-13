#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     rce-core/rce/util/converters/interfaces.py
#
#     This file is part of the RoboEarth Cloud Engine framework.
#
#     This file was originally created for RoboEearth
#     http://www.roboearth.org/
#
#     The research leading to these results has received funding from
#     the European Union Seventh Framework Programme FP7/2007-2013 under
#     grant agreement no248942 RoboEarth.
#
#     Copyright 2012 RoboEarth
#
#     Licensed under the Apache License, Version 2.0 (the "License");
#     you may not use this file except in compliance with the License.
#     You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
#     Unless required by applicable law or agreed to in writing, software
#     distributed under the License is distributed on an "AS IS" BASIS,
#     WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#     See the License for the specific language governing permissions and
#     limitations under the License.
#
#     \author/s: Dominique Hunziker
#
#

# zope specific imports
from zope.interface import Interface, Attribute


class ICustomROSConverter(Interface):
    """ Interface which declares the necessary methods which all ROS message
        types converters have to implement.
    """
    MESSAGE_TYPE = Attribute("""
    Identifier which is used to select the ROS converter.
    """)

    def encode(rosMsg): #@NoSelf
        """ Generate dictionary from a ROS message.

            @param rosMsg:  The ROS message instance which should be converted.
            @type  rosMsg:  genpy.message.Message

            @return:        Dictionary containing the parsed message. The basic
                            form does map each field in the ROS message to a
                            key / value pair in the returned data dict. Binaries
                            are added as StringIO instances.
            @rtype:         {}

            @raise:         TypeError, ValueError
        """

    def decode(data): #@NoSelf
        """ Generate a ROS message from dictionary.

            @param data:    Dictionary with keys matching the fields in the
                            desired ROS message.
                            Binary files should be included as StringIO
                            instances.
            @param data:    { str : {} }

            @return:        ROS message of type @param rosMsgType containing the
                            given data.
            @rtype:         ROS message of type @param rosMsgType

            @raise:         TypeError, ValueError,
                            rce.util.loader.ResourceNotFound
        """
