#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       Parser.py
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

# Custom imports
from ._NodeParser import NodeParser
from ._ParameterParser import IntParamParser, StrParamParser, FloatParamParser, BoolParamParser, FileParamParser
from ._InterfaceParser import ServiceInterfaceParser, PublisherInterfaceParser, SubscriberInterfaceParser

from Exceptions import InternalError

def createNodeParser(pkg, exe, params, interfaces):
    """ Creates a new node.

        @param pkg:     Package name where the node can be found.
        @type  pkg:     str

        @param exe:     Name of executable which should be launched.
        @type  exe:     str

        @param params:  Parameters which have to be used with this node.
                        They can be created using the function
                        'createParameter'.
        @type  params:  List of ParameterBaseFactory instances.

        @param interfaces:  Interfaces which this node provides.
                            They can be created using the function
                            'createInterface'.
        @type  interfaces:  List of InterfaceBaseFactory instances.

        @raise:     InternalError if there is a problem with the database.
    """
    return NodeParser(pkg, exe, params, interfaces)

def createParameterParser(name, paramType, opt=False, default=None):
    """ Creates a new parameter instance depending of paramType.

        @param name:    Name of the parameter
        @type  name:    str

        @param paramType:   String which represents the type of parameter
                            to create. Possible values are:
                                int, str, float, bool, file
        @type  paramType:   str

        @param opt:     Flag which indicates whether the configuration
                        parameter is optional or not
        @type  opt:     bool

        @param default: Default value for the case where the parameter
                        is optional

        @raise:     InternalError in case the opt flag is set but no
                    default value is given.
    """
    if paramType == 'int':
        return IntParamParser(name, opt, default)
    elif paramType == 'str':
        return StrParamParser(name, opt, default)
    elif paramType == 'float':
        return FloatParamParser(name, opt, default)
    elif paramType == 'bool':
        return BoolParamParser(name, opt, default)
    elif paramType == 'file':
        return FileParamParser(name, opt, default)
    else:
        raise InternalError('Could not identify the type of the configuration parameter.')

def createInterfaceParser(interfaceType, cls, interfaceName):
    """ Creates a new interface instance depending on interfaceType.

        @param interfaceType:   String which represents the type of
                                interface to create. Possible values are:
                                    pub, sub, srv
        @type  interfaceType:   str

        @param cls:     String representation of the class which is used
                        to communicate with the interface.
        @type  cls:     str

        @param interfaceName:   String representation of the name of
                                the interface.
        @type  interfaceName:   str

        @raise:     InternalError in case the interface type can not
                    be resolved.
    """
    if interfaceType == 'pub':
        interface = PublisherInterfaceParser(cls, interfaceName)
    elif interfaceType == 'sub':
        interface = SubscriberInterfaceParser(cls, interfaceName)
    elif interfaceType == 'srv':
        interface = ServiceInterfaceParser(cls, interfaceName)
    else:
        InternalError('Could not identify the type of the interface.')

    return interface
