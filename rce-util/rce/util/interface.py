#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     rce-util/rce/util/interface.py
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

# zope specific imports
from zope.interface import verify
from zope.interface.exceptions import Invalid


class InterfaceError(Exception):
    """ This error occurs when zope throws an error during an object or a class
        verification.
    """
    pass


def verifyObject(interfaceCls, obj):
    """ Verifies if the object implements the specified interface. Raises an
        InterfaceError if this is not the case.

        @param interfaceCls:    Interface class which the object should
                                implement.
        @type  interfaceCls:    zope.interface.Interface

        @param obj:             Object which should be verified.

        @raise:                 rce.util.interface.InterfaceError
    """
    try:
        verify.verifyObject(interfaceCls, obj)
    except Invalid as e:
        raise InterfaceError('Verification of the object of type "{0}" '
                             'failed: {1}'.format(obj.__class__.__name__, e))


def verifyClass(interfaceCls, cls):
    """ Verifies if the class implements the specified interface. Raises an
        InterfaceError if this is not the case.

        @param interfaceCls:    Interface class which the class should
                                implement.
        @type  interfaceCls:    zope.interface.Interface

        @param cls:             Class which should be verified.

        @raise:                 rce.util.interface.InterfaceError
    """
    try:
        verify.verifyClass(interfaceCls, cls)
    except Invalid as e:
        raise InterfaceError('Verification of the class of type "{0}" '
                             'failed: {1}'.format(cls.__name__, e))
