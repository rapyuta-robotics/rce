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

# twisted specific imports
from twisted.python import log
from twisted.spread.pb import Referenceable, \
    DeadReferenceError, PBConnectionLost


class Namespace(Referenceable):
    """ Abstract base class for a Namespace in a slave process.
    """
    def __init__(self):
        """ Initialize the Namespace.
        """
        self._interfaces = set()

    def registerInterface(self, interface):
        assert interface not in self._interfaces
        self._interfaces.add(interface)

    def unregisterInterface(self, interface):
        assert interface in self._interfaces
        self._interfaces.remove(interface)

    def remote_createInterface(self, *args, **kw):
        """ Remote callable method to create an interface in this namespace.

            Method has to be implemented!

            @return:            New Interface instance.
            @rtype:             rce.slave.interface.Interface
        """
        raise NotImplementedError("Method 'remote_createInterface' has"
                                  'to be implemented.')

    def remote_destroy(self):
        """ Method should be called to destroy the namespace and will take care
            of destroying all interfaces owned by this namespace as well as
            deleting all circular references.
        """
        for interface in self._interfaces.copy():
            interface.remote_destroy()

        assert len(self._interfaces) == 0

        if self._client._avatar:
            def eb(failure):
                if not failure.check(PBConnectionLost):
                    log.err(failure)

            try:
                self._client._avatar.callRemote('namespaceDied', self).addErrback(eb)
            except (DeadReferenceError, PBConnectionLost):
                pass
