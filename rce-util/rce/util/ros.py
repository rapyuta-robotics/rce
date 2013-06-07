#!/usr/bin/env python
# -*- coding: utf-8 -*-
#     
#     rce-util/rce/util/ros.py
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
from functools import wraps

# ROS specific imports
try:
    from rospy.core import get_node_uri
except ImportError:
    print('Can not import ROS Python client.')
    exit(1)


def decorator_has_connection(f):
    """ Decorator used to monkey patch the method 'has_connection' of the
        rospy Topic Implementation 'rospy.topics._TopicImpl.has_connection' to
        prevent publishers from sending messages to subscribers using the same
        endpoint ID as the publisher.
    """
    @wraps(f)
    def wrapper(self, endpoint_id):
        if endpoint_id == get_node_uri():
            return True
        
        return f(self, endpoint_id)
    
    return wrapper
