#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     rce-core/rce/util/name.py
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
#     Copyright 2012 RoboEarth
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
import re

# twisted specific imports
from twisted.spread.pb import Error


# ASCII char followed by (alphanumeric, _)
_NAME_RE = re.compile('^[A-Za-z][\w_]*$')


class IllegalName(Error):
    """ Exception is raised in case the name is not legal.
    """


def validateName(name):
    """ Check if the name is legal, i.e. if it starts with an ASCII char
        followed by alphanumeric chars or '_'.

        @param name:    Name which should be checked.
        @type  name:    str

        @raise:         IllegalName, if the name is not valid.
    """
    if not name:
        raise IllegalName('Name can not be an empty string.')

    m = _NAME_RE.match(name)

    if m is None or m.group(0) != name:
        raise IllegalName('Name has to start with a letter followed by an '
                          'arbitrary number of alphanumeric characters or '
                          'underscores.')
