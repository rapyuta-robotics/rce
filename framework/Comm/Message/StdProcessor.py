#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       StdProcessor.py
#       
#       This file is part of the RoboEarth Cloud Engine framework.
#       
#       This file was originally created for RoboEearth - http://www.roboearth.org/
#       The research leading to these results has received funding from the European Union 
#       Seventh Framework Programme FP7/2007-2013 under grant agreement no248942 RoboEarth.
#       
#       Copyright 2012 RoboEarth
#       
#       Licensed under the Apache License, Version 2.0 (the "License");
#       you may not use this file except in compliance with the License.
#       You may obtain a copy of the License at
#       
#       http://www.apache.org/licenses/LICENSE-2.0
#       
#       Unless required by applicable law or agreed to in writing, software
#       distributed under the License is distributed on an "AS IS" BASIS,
#       WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#       See the License for the specific language governing permissions and
#       limitations under the License.
#       
#       \author/s: Dominique Hunziker <dominique.hunziker@gmail.com> 
#       
#       

# zope specific imports
from zope.interface import implements

# Custom imports
from Interfaces import IMessageProcessor
import MsgTypes

class StdProcessorBase(object):
    """ Base class for all standard processors.
    """
    implements(IMessageProcessor)
    
    def __init__(self, manager):
        """ @param manager:     CommManager which is used in this node.
            @type  manager:     CommManager
        """
        self.manager = manager

class RouteProcessor(StdProcessorBase):
    """ Message processor to update the routing information.
    """
    IDENTIFIER = MsgTypes.ROUTE_INFO
    
    def processMessage(self, msg):
        self.manager.router.updateRoutingInfo(msg.origin, msg.content)
