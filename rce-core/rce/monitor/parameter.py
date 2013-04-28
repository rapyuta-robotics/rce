#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     rce-core/rce/monitor/parameter.py
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

# ROS specific imports
import rospy

# twisted specific imports
from twisted.python import log
from twisted.spread.pb import Referenceable

# rce specific imports
from rce.util.error import InternalError
from rce.monitor.common import ArgumentMixin


class Parameter(Referenceable, ArgumentMixin):
    """ Representation of a Parameter inside an environment.
    """
    def __init__(self, owner, name, value):
        """ Add the Parameter to the parameter server.

            @param owner:       Environment in which the node will be created.
            @type  owner:       rce.environment.Environment

            @param name:        Name of the parameter which should be added.
            @type  name:        str

            @param value:       Value of the parameter which should be added.
                                Top-level string values can contain the
                                directives $(find PKG) and/or $(env VAR).
            @type  value:       str, int, float, bool, list
        """
        self._registered = False

        ArgumentMixin.__init__(self, owner.loader)

        owner.registerParameter(self)
        self._owner = owner

        if isinstance(value, basestring):
            value = self.processArgument(value)

        self._name = name

        try:
            if rospy.has_param(name):
                log.msg('Warning: Parameter already exists.')

            rospy.set_param(name, value)
            self._registered = True
        except rospy.ROSException as e:
            raise InternalError('ROS Parameter Server reported an error: '
                                '{0}'.format(e))

    def remote_destroy(self):
        """ Method should be called to delete the Parameter from the parameter
            server.
        """
        if self._registered:
            try:
                rospy.delete_param(self._name)
            except rospy.ROSException:
                pass

            self._registered = False

        if self._owner:
            self._owner.unregisterParameter(self)
            self._owner = None
