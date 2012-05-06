#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       _InterfraceParser.py
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

# Python specific imports
from struct import error as StructError

try:
    from cStringIO import StringIO
except ImportError:
    from StringIO import StringIO

# Custom imports
from Exceptions import InternalError, SerializationError
from ContentDefinition import ContentDefinition, STRUCT_I

from ._ParserBase import ParserBase

class InterfaceBaseParser(ParserBase):
    """ Abstract base class which represents and provides a factory method
        for the interfaces to the nodes.
    """
    def __init__(self, cls, interfaceName):
        """ Constructor which initializes the basic fields of an interface.

            Should not be used directly. Instead use the provided function
            createInterface().

            @param cls:     String representation of the class which is
                            used to communicate with the interface.
            @type  cls:     str

            @param interfaceName:   String representation of the name of
                                    the interface.
            @type  interfaceName:   str

            @raise:     InternalError if the interfaceName or the cls is not
                        a valid string.
        """
        if isinstance(interfaceName, unicode):
            try:
                interfaceName = str(interfaceName)
            except UnicodeEncodeError:
                raise InternalError('The interface name {0} is not valid.'.format(interfaceName))

        if isinstance(cls, unicode):
            try:
                cls = str(cls)
            except UnicodeEncodeError:
                raise InternalError('The communication class {0} is not valid.'.format(cls))

        self.interfaceName = interfaceName
        self.cls = cls

    def serialize(self):
        """ Serialize the Interface instance.

            @raise:     SerializationError
        """
        buf = StringIO()

        try:
            buf.write(self.IDENTIFIER)
            buf.write(STRUCT_I.pack(len(self.interfaceName)))
            buf.write(self.interfaceName)
        except (StructError, TypeError) as e:
            raise SerializationError('Could not serialize Interface: {0}'.format(e))

        return buf.getvalue()

class ServiceInterfaceParser(InterfaceBaseParser):
    """ Used to create and parse a service interface.
    """
    IDENTIFIER = ContentDefinition.INTERFACE_SRV

    def __init__(self, cls, interfaceName):
        super(ServiceInterfaceParser, self).__init__(cls, interfaceName)

        self.reqCls = '{0}Request'.format(cls)
        self.rspCls = '{0}Response'.format(cls)

    def serialize(self):
        buf = StringIO()

        try:
            buf.write(self.IDENTIFIER)
            buf.write(STRUCT_I.pack(len(self.interfaceName)))
            buf.write(self.interfaceName)
            buf.write(STRUCT_I.pack(len(self.cls)))
            buf.write(self.cls)
        except (StructError, TypeError) as e:
            raise SerializationError('Could not serialize Interface: {0}'.format(e))

        return buf.getvalue()

    __init__.__doc__ = InterfaceBaseParser.__init__.__doc__
    serialize.__doc__ = InterfaceBaseParser.serialize.__doc__

class PublisherInterfaceParser(InterfaceBaseParser):
    """ Used to create and parse a publisher interface.
    """
    IDENTIFIER = ContentDefinition.INTERFACE_PUB

class SubscriberInterfaceParser(InterfaceBaseParser):
    """ Used to create and parse a subscriber interface.
    """
    IDENTIFER = ContentDefinition.INTERFACE_SUB
