#!/usr/bin/env python
# -*- coding: utf-8 -*-
#     
#     interfaces.py
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

# zope specific imports
from zope.interface import Attribute, Interface


class IRobot(Interface):
    """
    """
    def createContainer(tag): #@NoSelf
        """
        """
    
    def destroyContainer(tag): #@NoSelf
        """
        """
    
    def addNode(cTag, nTag, pkg, exe, args='', name='', namespace=''): #@NoSelf
        """
        """
    
    def removeNode(cTag, nTag): #@NoSelf
        """
        """
    
    def addInterface(eTag, iTag, iType, clsName, addr=''): #@NoSelf
        """
        """
    
    def removeInterface(iTag): #@NoSelf
        """
        """
    
    def addParameter(cTag, name, value): #@NoSelf
        """
        """
    
    def removeParameter(cTag, name): #@NoSelf
        """
        """
    
    def addConnection(tagA, tagB): #@NoSelf
        """
        """
    
    def removeConnection(tagA, tagB): #@NoSelf
        """
        """
    
    def receivedFromClient(orig, dest, msgType, msgID, msg): #@NoSelf
        """
        """
    
    def registerConnectionToRobot(connection): #@NoSelf
        """
        """
    
    def unregisterConnectionToRobot(): #@NoSelf
        """
        """


class IRobotCredentials(Interface):
    """
    """
    userID = Attribute(""" ... """)
    robotID = Attribute(""" ... """)
    
    def checkKey(key): #@NoSelf
        """
        """
