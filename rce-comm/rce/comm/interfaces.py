#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     rce-comm/rce/comm/interfaces.py
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
from zope.interface import Interface


class IMasterRealm(Interface):
    """ Interface which the Master realm has to implement.
    """
    def requestURL(userID):  #@NoSelf
        """ Callback for Robot resource to retrieve the location of the Robot
            process to which a WebSocket connection should be established.

            @param userID:      User ID under which the robot will login.
                                (Can be used to do optimizations in distributing
                                the load.)
            @type  userID:      str

            @return:            The IP address of Robot process to which a
                                WebSocket connection should be established.
                                (type: str)
            @rtype:             twisted.internet.defer.Deferred
        """


class IRobotRealm(Interface):
    """ Interface which the Robot realm has to implement.
    """
    def login(userID, robotID, password):  #@NoSelf
        """ Callback for Robot connection to login and authenticate.

            @param userID:      User ID under which the robot is logging in.
            @type  userID:      str

            @param robotID:     Unique ID of the robot in the namespace of the
                                user under which the robot is logging in.
            @type  robotID:     str

            @param password:    Hashed password as hex-encoded string which is
                                used to authenticate the user.
            @type  password:    str

            @return:            Representation of the connection to the robot
                                which is used in the Robot process.
                                (type: rce.robot.Connection)
            @rtype:             twisted.internet.defer.Deferred
        """

    def registerWebsocketProtocol(connection, protocol):  #@NoSelf
        """ Register the client protocol with a Connection object.

            @param connection:  Connection where the protocol should be
                                registered.
            @type  connection:  rce.robot.Connection

            @param protocol:    Protocol which should be registered.
            @type  protocol:    rce.comm.interfaces.IServersideProtocol
        """

    def unregisterWebsocketProtocol(connection, protocol):  #@NoSelf
        """ Unregister the client protocol from a Connection object.

            @param connection:  Connection where the protocol should be
                                unregistered.
            @type  connection:  rce.robot.Connection

            @param protocol:    Protocol which should be unregistered.
            @type  protocol:    rce.comm.interfaces.IServersideProtocol
        """


class IProtocol(Interface):
    """ Interface which the Protocol has to implement on the server side.
    """
    def sendDataMessage(iTag, clsName, msgID, msg):  #@NoSelf
        """ Send a data message to the robot client.

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

    def sendInterfaceStatusUpdateMessage(iTag, status): #@NoSelf
        """ Send a status change to the client such that the client can start
            or stop its client-side interface implementation according to the
            status information.

            @param iTag:        Tag which is used to identify the interface
                                which changed its status.
            @type  iTag:        str

            @param status:      Boolean indicating whether the interface should
                                be active or not.
            @type  status:      bool
        """

    def sendErrorMessage(msg):  #@NoSelf
        """ Send an error message to the robot client.

            @param msg:         Error message which should be sent.
            @type  msg:         str
        """

    def dropConnection():  #@NoSelf
        """ Request that the protocol drops the connection to the client.
        """


class IRobot(Interface):
    """ Interface which the Robot Avatar has to implement.
    """
    def createContainer(tag, data={}):  #@NoSelf
        """ Create a new Container object.

            @param tag:         Tag which is used to identify the container
                                in subsequent requests.
            @type  tag:         str

            @param data:        Extra data which is used to configure the
                                container.
            @type  data:        dict
        """

    def destroyContainer(tag):  #@NoSelf
        """ Destroy a Container object.

            @param tag:         Tag which is used to identify the container
                                which should be destroyed.
            @type  tag:         str
        """

    def addNode(cTag, nTag, pkg, exe, args='', name='', namespace=''):  #@NoSelf
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

    def removeNode(cTag, nTag):  #@NoSelf
        """ Remove a node from a container / ROS environment.

            @param cTag:        Tag which is used to identify the container /
                                ROS environment from which the node should be
                                removed.
            @type  cTag:        str

            @param nTag:        Tag which is used to identify the ROS node
                                which should removed.
            @type  nTag:        str
        """

    def addInterface(eTag, iTag, iType, clsName, addr=''):  #@NoSelf
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

    def removeInterface(eTag, iTag):  #@NoSelf
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

    def addParameter(cTag, name, value):  #@NoSelf
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

    def removeParameter(cTag, name):  #@NoSelf
        """ Remove a parameter from a container / ROS environment.

            @param cTag:        Tag which is used to identify the container /
                                ROS environment from which the parameter should
                                be removed.
            @type  cTag:        str

            @param name:        Name of the parameter which should be removed.
            @type  name:        str
        """

    def addConnection(tagA, tagB):  #@NoSelf
        """ Create a connection between two interfaces.

            @param tagX:        Tag which is used to identify the interface
                                which should be connected. It has to be of the
                                form:
                                    [endpoint tag]/[interface tag]
                                For example:
                                    testRobot/logPublisher
            @type  tagX:        str
        """

    def removeConnection(tagA, tagB):  #@NoSelf
        """ Destroy a connection between two interfaces.

            @param tagX:        Tag which is used to identify the interface
                                which should be disconnected. It has to be of
                                the form:
                                    [endpoint tag]/[interface tag]
                                For example:
                                    testRobot/logPublisher
            @type  tagX:        str
        """


class IMessageReceiver(Interface):
    """ Interface which declares the necessary callback for the communication
        client/server.
    """
    def processReceivedMessage(iTag, clsName, msgID, msg):  #@NoSelf
        """ Process a data message which has been received from the server side.

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


class IClient(IMessageReceiver):
    """ Interface which declares additional necessary callback for the
        communication on the client side.
    """
    def processInterfaceStatusUpdate(iTag, status): #@NoSelf
        """ Process a interface status update message received from the server
            side.

            @param iTag:        Tag which is used to identify the interface
                                which changed its status.
            @type  iTag:        str

            @param status:      Boolean indicating whether the interface should
                                be active or not.
            @type  status:      bool
        """
