#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       _Node.py
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
import roslaunch.core

# Python specific imports
from struct import error as StructError

# Custom imports
from Exceptions import InternalError, SerializationError
from ContentDefinition import ContentDefinition, STRUCT_I, LEN_I

from _Serializer import deserialize as deserializeElement

from ._Parameter import IntParam, StrParam, FloatParam, BoolParam, FileParam
from ._Interface import ServiceInterface, PublisherInterface, SubscriberInterface

_MAP = { ContentDefinition.INTERFACE_SRV : ServiceInterface,
         ContentDefinition.INTERFACE_PUB : PublisherInterface,
         ContentDefinition.INTERFACE_SUB : SubscriberInterface,
         ContentDefinition.PARAM_INT     : IntParam,
         ContentDefinition.PARAM_STR     : StrParam,
         ContentDefinition.PARAM_FLOAT   : FloatParam,
         ContentDefinition.PARAM_BOOL    : BoolParam,
         ContentDefinition.PARAM_FILE    : FileParam }

class Node(object):
    """ Class which represents a running node. The node is created
        based on the received information from the NodeFactory.
    """
    # TODO: Use slots yes or no?
    # __slots__ = ['_pkg', '_exe', '_params', '_interfaces', '_namespace', '_process', 'running']

    def __init__(self, pkg, exe, namespace='', params=[], interfaces=[]):
        """ Initialize the Node instance.

            @param pkg:     Name of the package where this node can be found.
            @type  pkg:     str

            @param exe:     Name of the executable which is used.
            @type  exe:     str

            @param namespace:   Namespace in which this node should be launched.
            @type  namespace:   str

            @param params:  Parameters which have to be set for this node.
            @type  params:  [ Parameter ]

            @param interfaces:  Interfaces which this node provides.
            @type  interfaces:  [ Interface ]
        """
        self._pkg = pkg
        self._exe = exe
        self._namespace = namespace
        self._params = params
        self._interfaces = interfaces
        self._process = None
        self.running = False

    @classmethod
    def deserialize(cls, data):
        """ Deserialize the Node instance.
        """
        try:
            start = 0
            end = LEN_I
            length, = STRUCT_I.unpack(data[start:end])
            start = end
            end += length
            pkg = data[start:end]

            start = end
            end += LEN_I
            length, = STRUCT_I.unpack(data[start:end])
            start = end
            end += length
            exe = data[start:end]

            start = end
            end += LEN_I
            length, = STRUCT_I.unpack(data[start:end])
            start = end
            end += length
            namespace = data[start:end]

            params = []
            start = end
            end += LEN_I
            size, = STRUCT_I.unpack(data[start:end])

            for _ in xrange(size):
                start = end
                end += LEN_I
                length, = STRUCT_I.unpack(data[start:end])
                start = end
                end += length
                params.append(deserializeElement(data[start:end], _MAP))

            interfaces = []
            start = end
            end += LEN_I
            size, = STRUCT_I.unpack(data[start:end])

            for _ in xrange(size):
                start = end
                end += LEN_I
                length, = STRUCT_I.unpack(data[start:end])
                start = end
                end += length
                interfaces.append(deserializeElement(data[start:end], _MAP))
        except StructError as e:
            raise SerializationError('Could not deserialize Node: {0}'.format(e))

        return cls(pkg, exe, namespace, params, interfaces)

    def start(self, manager, uid):
        """ Add the necessary parameters to the parameter server, launch the
            node and start the necessary handlers for the interfaces.

            @param manager: The currently used manager instance.
            @type  manager: Instance of (sub-)class of ManagerBase

            @param uid:     ID to identify the node with.
            @type  uid:     str

            @raise:     InternalError if Node can not be launched.
        """
        if self._process:
            raise InternalError('Can not launch an already running node.')

        for param in self._params:
            param.setParam()

        self._process = manager.startNodeProcess(uid, roslaunch.core.Node(self._pkg, self._exe, name=uid, namespace=self._namespace, output='screen'))

        for interface in self._interfaces:
            try:
                interface.registerManager(manager)
                interface.start()
            except InternalError: ### TODO: Should be possible to continue here (is this ok?)
                self._process.stop()
                raise

        self.running = True

        return uid

    def isAlive(self):
        """ Check whether the node/process is alive.

            @raise:    InternalError if the node is not yet launched.
        """
        try:
            return self._process.is_alive()
        except AttributeError:
            raise InternalError('Node is not yet launched.')

    def stop(self):
        """ Stop the running handlers for the interfaces, stop the node
            and and remove the parameters from the parameter server.
        """
        for interface in self._interfaces:
            interface.stop()

        if self._process and self._process.is_alive():
            self._process.stop()

        self.running = False

    def __del__(self):
        """ Destructor.
        """
        if self.running:
            self.stop()
