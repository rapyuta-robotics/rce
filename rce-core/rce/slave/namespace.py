#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     rce-core/rce/slave/namespace.py
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
#     Copyright 2013 RoboEarth
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

# Python specific imports
from uuid import UUID

# twisted specific imports
from twisted.spread.pb import Referenceable

# rce specific imports
from rce.util.error import InternalError


class Namespace(Referenceable):
    """ Abstract base class for a Namespace in a slave process.
    """
    def __init__(self, endpoint):
        """ Initialize the Namespace.
        """
        self._endpoint = endpoint
        endpoint.registerNamespace(self)

        self._interfaces = {}
        self._map = {}

    @property
    def reactor(self):
        """ Reference to twisted::reactor. """
        return self._endpoint.reactor

    @property
    def loader(self):
        """ Reference to ROS components loader. """
        return self._endpoint.loader

    def registerInterface(self, interface):
        addr = interface.addr

        assert addr not in self._interfaces
        self._interfaces[addr] = interface

    def unregisterInterface(self, interface):
        addr = interface.addr

        assert addr in self._interfaces
        del self._interfaces[addr]
        self._endpoint.referenceDied('interfaceDied', interface)

    def remote_createInterface(self, uid, iType, msgType, addr):
        """ Create an Interface object in the namespace and therefore in
            the endpoint.

            @param uid:         Unique ID which is used to identify the
                                interface in the internal communication.
            @type  uid:         str

            @param iType:       Type of the interface encoded as an integer.
                                Refer to rce.slave.interface.Types for more
                                information.
            @type  IType:       int

            @param clsName:     Message type/Service type consisting of the
                                package and the name of the message/service,
                                i.e. 'std_msgs/Int32'.
            @type  clsName:     str

            @param addr:        Unique address which is used to identify the
                                interface in the external communication.
            @type  addr:        str

            @return:            New Interface instance.
            @rtype:             rce.slave.interface.Interface
        """
        try:
            cls = self._map[iType]
        except KeyError:
            raise InternalError('Interface type is not supported by this '
                                'namespace.')

        return cls(self, UUID(bytes=uid), msgType, addr)

    def remote_destroy(self):
        """ Method should be called to destroy the namespace and will take care
            of destroying all interfaces owned by this namespace as well as
            deleting all circular references.
        """
        for interface in self._interfaces.values():
            interface.remote_destroy()

        assert len(self._interfaces) == 0

        if self._endpoint:
            self._endpoint.unregisterNamespace(self)
            self._endpoint = None
