#!/usr/bin/env python
# -*- coding: utf-8 -*-
#     
#     user.py
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
from uuid import uuid4

# twisted specific imports
from twisted.spread.pb import Referenceable

# Custom imports
from rce.error import InvalidRequest
from rce.master.base import AlreadyDead
from rce.slave.interface import Types
from rce.util.name import isLegalName


class User(Referenceable):
    """
    """
    def __init__(self, realm, userID):
        """
        """
        self._realm = realm
        self._userID = userID
        
        self._robots = {}
        self._containers = {}
        self._connections = {}
    
    def createRobot(self, robotID):
        """
        """
        if not isLegalName(robotID):
            raise InvalidRequest('Robot ID is not a valid.')
        
        if (robotID in self._robots or robotID in self._containers):
            raise InvalidRequest('ID is already used for a container '
                                 'or robot.')
        
        uid = uuid4().hex
        robot = self._realm.createRobot(self, self._userID, robotID, uid)
        robot = Robot(robot, uid)
        self._robots[robotID] = robot
        robot.notifyOnDeath(self._robotDied)
        return robot.getConnectInfo()
    
    def remote_createContainer(self, tag):
        """
        """
        # TODO: Container tags can not be the same as robotIDs,
        #       verify this somewhere!
        
        if not isLegalName(tag):
            raise InvalidRequest('Container tag is not a valid.')
        
        if tag in self._containers or tag in self._robots:
            raise InvalidRequest('Tag is already used for a container '
                                 'or robot.')
        
        container = Container(self._realm.createContainer(tag))
        self._containers[tag] = container
        container.notifyOnDeath(self._containerDied)
    
    def remote_destroyContainer(self, tag):
        """
        """
        try:
            container = self._containers.pop(tag)
        except KeyError:
            raise InvalidRequest('Can not destroy non existent container.')
        
        container.dontNotifyOnDeath(self._containerDied)
        container.destroy()
    
    def remote_addNode(self, cTag, nTag, pkg, exe, args='', name='', nspc=''):
        """
        """
        try:
            self._containers[cTag].addNode(nTag, pkg, exe, args, name, nspc)
        except KeyError:
            raise InvalidRequest('Can not add Node, because Container {0} '
                                 'does not exist.'.format(cTag))
    
    def remote_removeNode(self, cTag, nTag):
        """
        """
        try:
            self._containers[cTag].removeNode(nTag)
        except KeyError:
            raise InvalidRequest('Can not remove Node, because Container {0} '
                                 'does not exist.'.format(cTag))
    
    def remote_addParameter(self, cTag, name, value):
        """
        """
        try:
            self._containers[cTag].addParameter(name, value)
        except KeyError:
            raise InvalidRequest('Can not add Parameter, because Container '
                                 '{0} does not exist.'.format(cTag))
    
    def remote_removeParameter(self, cTag, name):
        """
        """
        try:
            self._containers[cTag].removeParameter(name)
        except KeyError:
            raise InvalidRequest('Can not remove Parameter, because Container '
                                 '{0} does not exist.'.format(cTag))
    
    def remote_addInterface(self, eTag, iTag, iType, clsName, addr=''):
        """
        """
        if iType.endswith('Converter') or iType.endswith('Forwarder'):
            try:
                self._robots[eTag].addInterface(iTag, iType, clsName)
            except KeyError:
                raise InvalidRequest('Can not add Interface, because Robot '
                                     '{0} does not exist.'.format(eTag))
        elif iType.endswith('Interface'):
            try:
                self._containers[eTag].addInterface(iTag, iType, clsName, addr)
            except KeyError:
                raise InvalidRequest('Can not add Interface, because '
                                     'Container {0} does not '
                                     'exist.'.format(eTag))
        else:
            raise InvalidRequest('Interface type is invalid (Unknown suffix).')
    
    def remote_removeInterface(self, eTag, iTag):
        """
        """
        self._getEndpoint(eTag).removeInterface(iTag)
    
    def remote_addConnection(self, tagA, tagB):
        """
        """
        eTagA, iTagA = tagA.split('/', 2)
        eTagB, iTagB = tagB.split('/', 2)
        
        ifA = self._getEndpoint(eTagA).getInterface(iTagA)
        ifB = self._getEndpoint(eTagB).getInterface(iTagB)
        
        if ifA.msgType != ifB.msgType:
            raise InvalidRequest('Can not connect two interfaces with '
                                 'different message type.')
        
        if not Types.connectable(ifA.iType, ifB.iType):
            raise InvalidRequest('Can not connect an interface of type {0} '
                                 'and an interface of type '
                                 '{1}.'.format(Types.decode(ifA.iType),
                                               Types.decode(ifB.iType)))
        
        key = hash(tagA)^hash(tagB)
        
        if key in self._connections:
            raise InvalidRequest('Can not add the same connection twice.')
        
        self._connections[key] = self._realm.createConnection(ifA.obj, ifB.obj)
    
    def remote_removeConnection(self, tagA, tagB):
        """
        """
        key = hash(tagA)^hash(tagB)
        
        try:
            connection = self._connections.pop(key)
        except KeyError:
                raise InvalidRequest('Can not disconnect two unconnected '
                                     'interfaces.')
        
        connection.destroy()
    
    def _getEndpoint(self, tag):
        if tag in self._robots:
            return self._robots[tag]
        elif tag in self._containers:
            return self._containers[tag]
        else:
            raise InvalidRequest('There is no endpoint with tag '
                                 '{0}.'.format(tag))
    
    def _containerDied(self, container):
        if self._robots:
            for key, value in self._containers.iteritems():
                if value == container:
                    del self._containers[key]
                    break
        else:
            print('Received notification for dead Container, '
                  'but User is already destroyed.')
    
    def _robotDied(self, robot):
        if self._robots:
            for key, value in self._robots.iteritems():
                if value == robot:
                    del self._robots[key]
                    break
        else:
            print('Received notification for dead Robot, '
                  'but User is already destroyed.')
    
    def destroy(self):
        """
        """
#        for connection in self._connections.itervalues():
#            connection.dontNotifyOnDeath(self._connectionDied)
        
        for container in self._containers.itervalues():
            container.dontNotifyOnDeath(self._containerDied)
        
        for robot in self._robots.itervalues():
            robot.dontNotifyOnDeath(self._robotDied)
        
        for container in self._containers.itervalues():
            container.destroy()
        
        for robot in self._robots.itervalues():
            robot.destroy()
        
        self._connections = None
        self._containers = None
        self._robots = None


class _Wrapper(object):
    """
    """
    def __init__(self, obj):
        """
        """
        self._obj = obj
        obj.notifyOnDeath(self._selfDied)
        
        self._cbs = set()
    
    def notifyOnDeath(self, cb):
        """
        """
        assert callable(cb)
        
        try:
            self._cbs.add(cb)
        except AttributeError:
            raise AlreadyDead('{0} is already '
                              'dead.'.format(self.__class__.__name__))
    
    def dontNotifyOnDeath(self, cb):
        """
        """
        try:
            self._cbs.remove(cb)
        except AttributeError:
            pass
    
    def _selfDied(self, _):
        for cb in self._cbs:
            cb(self)
        
        self._cbs = None
    
    def destroy(self):
        """
        """
        self._obj.destroy()
        self._obj = None


class Robot(_Wrapper):
    """
    """
    def __init__(self, namespace, uid):
        """
        """
        super(Robot, self).__init__(namespace)
        
        self._uid = uid
        
        self._interfaces = {}
    
    def getConnectInfo(self):
        """
        """
        return self._obj.getIP().addCallback(lambda ip: (self._uid, ip))
    
    def addInterface(self, iTag, iType, clsName):
        """
        """
        if not isLegalName(iTag):
            raise InvalidRequest('Interface tag is not a valid.')
        
        if iTag in self._interfaces:
            raise InvalidRequest("Can not use the same interface tag '{0}' "
                                 'in the same robot twice.'.format(iTag))
        
        if iType.endswith('Forwarder'):
            modifier = 4
        else:
            modifier = 0
        
        try:
            iType = Types.encode(iType)
        except TypeError:
            raise InvalidRequest('Interface type is invalid (Unknown prefix).')
        
        
        interface = self._obj.createInterface(iType+modifier, clsName, iTag)
        interface = Interface(interface, iType, clsName)
        self._interfaces[iTag] = interface
        interface.notifyOnDeath(self._interfaceDied)
    
    def removeInterface(self, iTag):
        """
        """
        try:
            self._interfaces.pop(iTag).destroy()
        except KeyError:
            raise InvalidRequest('Can not remove a non existent interface '
                                 "'{0}' from the robot.".format(iTag))
    
    def getInterface(self, iTag):
        """
        """
        try:
            return self._interfaces[iTag]
        except KeyError:
            raise InvalidRequest('Can not get a non existent interface '
                                 "'{0}' from the robot.".format(iTag))
    
    def _interfaceDied(self, interface):
        if self._interfaces:
            for key, value in self._interfaces.iteritems():
                if value == interface:
                    del self._interfaces[key]
                    break
        else:
            print('Received notification for dead Interface, '
                  'but Robot is already destroyed.')
    
    def destroy(self):
        """
        """
        for interface in self._interfaces.itervalues():
            interface.dontNotifyOnDeath(self._interfaceDied)
        
        self._interfaces = None
        
        super(Robot, self).destroy()


class Container(_Wrapper):
    """
    """
    def __init__(self, namespace):
        """
        """
        super(Container, self).__init__(namespace)
        
        self._nodes = {}
        self._parameters = {}
        self._interfaces = {}
    
    def addNode(self, nTag, pkg, exe, args, name, namespace):
        """
        """
        if not isLegalName(nTag):
            raise InvalidRequest('Node tag is not a valid.')
        
        if nTag in self._nodes:
            raise InvalidRequest("Can not use the same node tag '{0}' in the "
                                 'same container twice.'.format(nTag))
        
        node = self._obj.createNode(pkg, exe, args, name, namespace)
        self._nodes[nTag] = node
        node.notifyOnDeath(self._nodeDied)
    
    def addParameter(self, name, value):
        """
        """
        if not name:
            raise InvalidRequest('Parameter name is not a valid.')
        
        if name in self._parameters:
            raise InvalidRequest("Can not use the same parameter name '{0}' "
                                 'in the same container twice.'.format(name))
        
        parameter = self._obj.createParameter(name, value)
        self._parameters[name] = parameter
        parameter.notifyOnDeath(self._parameterDied)
    
    def addInterface(self, iTag, iType, clsName, addr):
        """
        """
        if not isLegalName(iTag):
            raise InvalidRequest('Interface tag is not a valid.')
        
        if iTag in self._interfaces:
            raise InvalidRequest("Can not use the same interface tag '{0}' "
                                 'in the same container twice.'.format(iTag))
        
        try:
            iType = Types.encode(iType)
        except TypeError:
            raise InvalidRequest('Interface type is invalid (Unknown prefix).')
        
        interface = Interface(self._obj.createInterface(iType, clsName, addr),
                              iType, clsName)
        self._interfaces[iTag] = interface
        interface.notifyOnDeath(self._interfaceDied)
    
    def removeNode(self, nTag):
        """
        """
        try:
            self._nodes.pop(nTag).destroy()
        except KeyError:
            raise InvalidRequest('Can not remove a non existent node '
                                 "'{0}' from the container.".format(nTag))
    
    def removeParameter(self, name):
        """
        """
        try:
            self._parameters.pop(name).destroy()
        except KeyError:
            raise InvalidRequest('Can not remove a non existent node '
                                 "'{0}' from the container.".format(name))
    
    def removeInterface(self, iTag):
        """
        """
        try:
            self._interfaces.pop(iTag).destroy()
        except KeyError:
            raise InvalidRequest('Can not remove a non existent interface '
                                 "'{0}' from the container.".format(iTag))
    
    def getInterface(self, iTag):
        """
        """
        try:
            return self._interfaces[iTag]
        except KeyError:
            raise InvalidRequest('Can not get a non existent interface '
                                 "'{0}' from the container.".format(iTag))
    
    def _nodeDied(self, node):
        if self._nodes:
            for key, value in self._nodes.iteritems():
                if value == node:
                    del self._nodes[key]
                    break
        else:
            print('Received notification for dead Node, '
                  'but Container is already destroyed.')
    
    def _parameterDied(self, parameter):
        if self._parameters:
            for key, value in self._parameters.iteritems():
                if value == parameter:
                    del self._parameters[key]
                    break
        else:
            print('Received notification for dead Parameter, '
                  'but Container is already destroyed.')
    
    def _interfaceDied(self, interface):
        if self._interfaces:
            for key, value in self._interfaces.iteritems():
                if value == interface:
                    del self._interfaces[key]
                    break
        else:
            print('Received notification for dead Interface, '
                  'but Container is already destroyed.')
    
    def destroy(self):
        """
        """
        for node in self._nodes.itervalues():
            node.dontNotifyOnDeath(self._nodeDied)
        
        for parameter in self._parameters.itervalues():
            parameter.dontNotifyOnDeath(self._parameterDied)
        
        for interface in self._interfaces.itervalues():
            interface.dontNotifyOnDeath(self._interfaceDied)
        
        self._nodes = None
        self._parameters = None
        self._interfaces = None
        
        super(Container, self).destroy()


class Interface(_Wrapper):
    """
    """
    def __init__(self, interface, iType, msgType):
        """
        """
        super(Interface, self).__init__(interface)
        
        self.iType = iType
        self.msgType = msgType
    
    @property
    def obj(self):
        """ ... """
        return self._obj
    
