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
    """ Interface which the Robot Avatar has to implement.
    """
    def createContainer(tag): #@NoSelf
        """ Create a new Container object.
            
            @param tag:         Tag which is used to identify the container
                                in subsequent requests.
            @type  tag:         str
        """
    
    def destroyContainer(tag): #@NoSelf
        """ Destroy a Container object.
            
            @param tag:         Tag which is used to identify the container
                                which should be destroyed.
            @type  tag:         str
        """
    
    def addNode(cTag, nTag, pkg, exe, args='', name='', namespace=''): #@NoSelf
        """ Add a node to a container / ROS environment.
            
            @param cTag:        Tag which is used to identify the container /
                                ROS environment to which the node should be
                                added.
            @type  cTag:        str
            
            @param nTag:        Tag which is used to identify the node in
                                subsequent requests.
            @type  nTag:        str

            @param pkg:         Name of ROS package where the node can be
                                found.
            @type  pkg:         str

            @param exe:         Name of executable (node) which should be
                                launched.
            @type  exe:         str
            
            @param args:        Additional arguments which should be used for
                                the launch. Can contain the directives
                                $(find PKG) and/or $(env VAR). Other special
                                characters such as '$' or ';' are not allowed.
            @type  args:        str
            
            @param name:        Name of the node under which the node should be
                                launched.
            @type  name:        str
            
            @param namespace:   Namespace in which the node should be started
                                in the environment.
            @type  namespace:   str
        """
    
    def removeNode(cTag, nTag): #@NoSelf
        """ Remove a node from a container / ROS environment.
            
            @param cTag:        Tag which is used to identify the container /
                                ROS environment from which the node should be
                                removed.
            @type  cTag:        str
            
            @param nTag:        Tag which is used to identify the ROS node
                                which should removed.
            @type  nTag:        str
        """
    
    def addInterface(eTag, iTag, iType, clsName, addr=''): #@NoSelf
        """ Add an interface to an endpoint, i.e. a ROS environment or a 
            Robot object.
            
            @param eTag:        Tag which is used to identify the endpoint to
                                which the interface should be added; either
                                a container tag or robot ID.
            @type  eTag:        str
                            
            @param iTag:        Tag which is used to identify the interface in
                                subsequent requests.
            @type  iTag:        str
            
            @param iType:       Type of the interface. The type consists of a
                                prefix and a suffix.
                                 - Valid prefixes are:
                                     ServiceClient, ServiceProvider,
                                     Publisher, Subscriber
                                 - Valid suffixes are:
                                     Interface, Converter, Forwarder
            @type  iType:       str
            
            @param clsName:     Message type/Service type consisting of the
                                package and the name of the message/service,
                                i.e. 'std_msgs/Int32'.
            @type  clsName:     str
            
            @param addr:        ROS name/address which the interface should
                                use. Only necessary if the suffix of @param
                                iType is 'Interface'.
            @type  addr:        str
        """
    
    def removeInterface(eTag, iTag): #@NoSelf
        """ Remove an interface from an endpoint, i.e. a ROS environment or a 
            Robot object.
            
            @param eTag:        Tag which is used to identify the endpoint from
                                which the interface should be removed; either
                                a container tag or robot ID.
            @type  eTag:        str
            
            @param iTag:        Tag which is used to identify the interface
                                which should be removed.
            @type  iTag:        str
        """
    
    def addParameter(cTag, name, value): #@NoSelf
        """ Add a parameter to a container / ROS environment.
            
            @param cTag:        Tag which is used to identify the container /
                                ROS environment to which the parameter should
                                be added.
            @type  cTag:        str
            
            @param name:        Name of the parameter which should be added.
                                It is also used to identify the parameter in
                                subsequent requests.
            @type  name:        str
            
            @param value:       Value of the parameter which should be added.
                                Top-level string values can contain the
                                directives $(find PKG) and/or $(env VAR).
            @type  value:       str, int, float, bool, list
        """
    
    def removeParameter(cTag, name): #@NoSelf
        """ Remove a parameter from a container / ROS environment.
            
            @param cTag:        Tag which is used to identify the container /
                                ROS environment from which the parameter should
                                be removed.
            @type  cTag:        str
            
            @param name:        Name of the parameter which should be removed.
            @type  name:        str
        """
    
    def addConnection(tagA, tagB): #@NoSelf
        """ Create a connection between two interfaces.
            
            @param tagX:        Tag which is used to identify the interface
                                which should be connected. It has to be of the
                                form:
                                    [endpoint tag]/[interface tag]
                                For example:
                                    testRobot/logPublisher
            @type  tagX:        str
        """
    
    def removeConnection(tagA, tagB): #@NoSelf
        """ Destroy a connection between two interfaces.
            
            @param tagX:        Tag which is used to identify the interface
                                which should be disconnected. It has to be of
                                the form:
                                    [endpoint tag]/[interface tag]
                                For example:
                                    testRobot/logPublisher
            @type  tagX:        str
        """
    
    def receivedFromClient(iTag, clsName, msgID, msg): #@NoSelf
        """ Process a data message which has been received from the robot
            client and send the message to the appropriate interface.
                            
            @param iTag:        Tag which is used to identify the interface to
                                which this message should be sent.
            @type  iTag:        str
            
            @param clsName:     Message type/Service type consisting of the
                                package and the name of the message/service,
                                i.e. 'std_msgs/Int32'.
            @type  clsName:     str
            
            @param msgID:       Message ID which can be used to get a
                                correspondence between request and response
                                message for a service call.
            @type  msgID:       str
            
            @param msg:         Message which should be sent. It has to be a
                                JSON compatible dictionary where part or the
                                complete message can be replaced by a StringIO
                                instance which is interpreted as binary data.
            @type  msg:         {str : {} / base_types / StringIO} / StringIO
        """
    
    def registerConnectionToRobot(connection): #@NoSelf
        """ Register the connection to the robot with this avatar.
            
            @param connection:  Connection which should be registered.
        """
    
    def unregisterConnectionToRobot(): #@NoSelf
        """ Unregister the connection to the robot with this avatar.
        """


class IRobotCredentials(Interface):
    """ Interface which the used robot credentials has to implement.
    """
    userID = Attribute(""" User ID of the robot owner. """)
    robotID = Attribute(""" Robot ID to identify the robot. """)
    
    def checkKey(key): #@NoSelf
        """ Check if the provided key matches the stored key which has been
            received from the client.
            
            @param key:         True value of the key which should be used to
                                verify the credentials.
            @type  key:         str
            
            @return:            True if the key matches; False otherwise.
        """
