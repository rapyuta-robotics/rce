#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       _NodeParser.py
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
import rosgraph.names

# Python specific imports
from struct import error as StructError

try:
    from cStringIO import StringIO
except ImportError:
    from StringIO import StringIO

# Custom imports
from Exceptions import InvalidRequest, InternalError, SerializationError
from ContentDefinition import ContentDefinition, STRUCT_I

from ._ParserBase import ParserBase

class NodeParser(ParserBase):
    """ Class which is used to create a node. This class is used
        to generate the necessary information to create a node
        and which is forwarded to the correct environment where
        a Node instance is created based on the sent information.
    """
    def __init__(self, pkg, exe, params, interfaces):
        """ Constructor which initializes the basic fields of a node.

            @param pkg:     Package name where the node can be found.
            @type  pkg:     str

            @param exe:     Name of executable which should be launched.
            @type  exe:     str

            @param params:  Parameters which have to be used with this node.
            @type  params:  List of ParameterBaseFactory instances.

            @param interfaces:  Interfaces which this node provides.
            @type  interfaces:  List of InterfaceBaseFactory instances.

            @raise:     InternalError if there is a problem with the database.
        """
        # First make sure, that the received string is a str object and not unicode
        if isinstance(pkg, unicode):
            try:
                pkg = str(pkg)
            except UnicodeEncodeError:
                raise InternalError('The package {0} is not valid.'.format(pkg))

        # First make sure, that the received string is a str object and not unicode
        if isinstance(exe, unicode):
            try:
                exe = str(exe)
            except UnicodeEncodeError:
                raise InternalError('The executable {0} is not valid.'.format(exe))

        self.pkg = pkg
        self.exe = exe
        self.params = params
        self.interfaces = interfaces

    def setNamespace(self, namespace):
        """ Set the ROS namespace in which the node should be created.

            @param namespace:   Namespace which should be used.
            @type  namespace:   str

            @raise:     InternalError if there is a problem with the namespace name.
                        InvalidRequest if the namespace is not valid.
        """
        # First make sure, that the received string is a str object and not unicode
        if isinstance(namespace, unicode):
            try:
                namespace = str(namespace)
            except UnicodeEncodeError:
                raise InternalError('The namespace {0} is not valid.'.format(namespace))

        # Validate namespace
        if not rosgraph.names.is_legal_name(namespace):
            raise InvalidRequest('The namespace {0} is not valid.'.format(namespace))

        self.namespace = namespace

        for param in self.params:
            param.setNamespace(namespace)

    def parse(self, config, files):
        """ Parses the given configuration to matching parameters.

            @param config:  Dictionary which contains all given parameters
                            as keys and their corresponding values as values.
            @type  config:  { str : ... }

            @param files:   Dictionary which contains the cStringIO.StringO
                            objects which contain the binary files sent with
                            the request.
            @type  files:   { str : cStringIO.StringO }

            @raise:     InvalidRequest in case of a parsing error or else
                        InternalError.
        """
        for param in self.params:
            param.parse(config, files)

    def serialize(self):
        """ Serialize the Node instance.

            @raise:     SerializationError
        """
        buf = StringIO()

        try:
            # Content identifier
            buf.write(ContentDefinition.NODE)

            # Package name
            buf.write(STRUCT_I.pack(len(self.pkg)))
            buf.write(self.pkg)

            # Executable name
            buf.write(STRUCT_I.pack(len(self.exe)))
            buf.write(self.exe)

            # ROS namespace
            buf.write(STRUCT_I.pack(len(self.namespace)))
            buf.write(self.namespace)

            # Parameters
            buf.write(STRUCT_I.pack(len(self.params)))

            for param in self.params:
                param = param.serialize()

                buf.write(STRUCT_I.pack(len(param)))
                buf.write(param)

            # Interfaces
            buf.write(STRUCT_I.pack(len(self.interfaces)))

            for interface in self.interfaces:
                interface = interface.serialize()

                buf.write(STRUCT_I.pack(len(interface)))
                buf.write(interface)
        except (StructError, TypeError) as e:
            raise SerializationError('Could not serialize Node: {0}'.format(e))

        return buf.getvalue()
