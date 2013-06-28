#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     rce-core/rce/util/iaas.py
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

# TODO: This needs some work on specification:
#        - define interface
#        - create a class for each IAAS type implementing interface
class IaasHook(object):
    """ # TODO: Add doc
    """
    def disconnect(self):
        """ Method is called when shutting down the engine to relieve the hook.
        """
        # TODO: Should destroy all instances which have been started dynamically
        raise NotImplementedError

    def spin_up(self, count=1, type=None, specialRequest=None):
        """ Call to spin up more instances.

            @param count:           Number of instances to be spun up.
            @type  count:           int

            @param type:            Type (generally size) of instance requested
            @type  type:            TDB by implementation

            @param specialRequest:  Special request (gpu, cluster, hadoop)
            @type  specialRequest:  TDB by implementation
        """
        raise NotImplementedError

    def spin_down(self):
        """ # TODO: ???
        """
        raise NotImplementedError
