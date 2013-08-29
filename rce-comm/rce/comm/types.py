#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     rce-comm/rce/comm/types.py
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

""" Message Types of RCE Client Protocol:

        CC      Create a container
        DC      Destroy a container

        CN      Change a (ROS) component (Node, Parameter, Interface)
        CX      Change connections between Interfaces

        DM      ROS Message

        ST      Status message
        ER      Error message


    Content Types of RCE Client Status Messages (ST):

        iu      Interface status update
"""

CREATE_CONTAINER = 'CC'
DESTROY_CONTAINER = 'DC'

CONFIGURE_COMPONENT = 'CN'
CONFIGURE_CONNECTION = 'CX'

DATA_MESSAGE = 'DM'

STATUS = 'ST'
ERROR = 'ER'

STATUS_INTERFACE = 'iu'
