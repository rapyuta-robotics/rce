#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     rce-core/rce/monitor/common.py
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
import os
import re


class EnvironmentVariableNotFound(Exception):
    """ Exception is raised in case the requested environment variable was not
        found.
    """


class ArgumentMixin(object):
    """ Class which can be used as a mixin to add similar capabilities to
        dynamically modify arguments as for a ROS launch file.
    """
    _RE_FIND = re.compile('\\$\\( *find +(?P<pkg>[a-zA-Z][a-zA-z0-9_]*) *\\)')
    _RE_ENV = re.compile('\\$\\( *env +(?P<var>[a-zA-Z][a-zA-z0-9_]*) *\\)')

    def __init__(self, loader, *args, **kw):
        """ Initialize the argument mixin. It needs the manager instance as
            fist argument.

            @param loader:      Loader which can be used to import ROS
                                resources.
            @type  loader:      rce.util.loader.Loader
        """
        self._loader = loader

    def _replaceFind(self, match):
        """ Internally used method to replace found matches of _RE_FIND regular
            expression with corresponding package path.
        """
        path = self._loader.findPkgPath(match.group('pkg'))
        return '"{0}"'.format(path) if ' ' in path else path

    def _replaceEnv(self, match):
        """ Internally used method to replace found matches of _RE_ENV regular
            expression with corresponding environment variable.
        """
        var = match.group('var')

        try:
            return os.environ[var]
        except KeyError:
            raise EnvironmentVariableNotFound('Can not find environment '
                                              'variable: {0}'.format(var))

    def processArgument(self, value):
        """ Run the replacement methods over the argument if it is a string.

            @param value:       Value which should be modified if necessary.

            @return:            Value which should be used instead of the given
                                value.
        """
        if not isinstance(value, basestring):
            return value

        value = self._RE_FIND.subn(self._replaceFind, value)[0]
        value = self._RE_ENV.subn(self._replaceEnv, value)[0]

        return value
