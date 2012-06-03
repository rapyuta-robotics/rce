#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       NodeParser.py
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

# Custom imports
from Exceptions import InvalidRequest, InternalError
from Node import Node #@UnresolvedImport

class NodeParser(object):
    """ Class which is used to parse the incoming request to start a node. This class is
        used to generate the necessary information to create a node, which is forwarded to
        the correct environment.
    """
    def __init__(self, pkg, exe, params):
        """ Initialize the NodeParser.

            @param pkg:     Package name where the node can be found.
            @type  pkg:     str

            @param exe:     Name of executable which should be launched.
            @type  exe:     str

            @param params:  Parameters which have to be used with this node.
            @type  params:  List of ParameterParser instances.

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

        self._pkg = pkg
        self._exe = exe
        self._params = params

    def parse(self, config):
        """ Parses the given configuration to matching parameters.

            @param config:  Dictionary which contains all given parameters as keys and
                            their corresponding values as values. Binary files are
                            represented as strings.
            @type  config:  { str : ... }
            
            @return:    Node instance which can be passed to the NodeMonitor.
            @rtype:     Node
            
            @raise:     InvalidRequest in case of a parsing error or else
                        InternalError.
        """
        namespace = config['namespaceID']     # TODO: Is this correct?
        
        # First make sure, that the received namespace is a str object and not unicode
        if isinstance(namespace, unicode):
            try:
                namespace = str(namespace)
            except UnicodeEncodeError:
                raise InternalError('The namespace {0} is not valid.'.format(namespace))

        # Validate namespace
        if not rosgraph.names.is_legal_name(namespace):
            raise InvalidRequest('The namespace {0} is not valid.'.format(namespace))
        
        return Node( self.pkg,
                     self.exe,
                     namespace,
                     [param.parse(config['parameters'], namespace) for param in self.params] )
