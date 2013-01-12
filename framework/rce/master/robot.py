#!/usr/bin/env python
# -*- coding: utf-8 -*-
#     
#     robot.py
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

# twisted specific imports
from twisted.internet.defer import succeed

# Custom imports
from rce.master.network import Endpoint, Namespace
from rce.master.base import Proxy


class RelayError(Exception):
    """
    """


class Distributor(object):
    """
    """
    def __init__(self):
        """
        """
        self._relays = set()
        self._iter = iter(self._relays)
    
    def registerRelay(self, relay):
        """
        """
        assert relay not in self._relays
        self._relays.add(relay)
    
    def unregisterRelay(self, relay):
        """
        """
        assert relay in self._relays
        self._relays.remove(relay)
    
    def getNextLocation(self, retry=False):
        """
        """
        try:
            return self._iter.next()
        except StopIteration:
            if retry:
                raise RelayError('There is no relay registered.')
        except RuntimeError:
            self._iter = iter(self._relays)
        
        return self.getNextLocation(True)
    
    def cleanUp(self):
        """
        """
        assert len(self._relays) == 0


class Robot(Namespace):
    """
    """
    def __init__(self, endpoint):
        """
        """
        super(Robot, self).__init__(endpoint)
    
    @Proxy.returnDeferred
    def getIP(self):
        """
        """
        return self.obj.broker.transport.getPeer().host


class RobotEndpoint(Endpoint):
    """
    """
    def __init__(self, network, distributor):
        """
        """
        super(RobotEndpoint, self).__init__(network)
        
        self._distributor = distributor
        distributor.registerRelay(self)
    
    def getAddress(self):
        """
        """
        return succeed(self.obj.broker.transport.getPeer())
    
    @Proxy.returnProxy(Robot)
    def createNamespace(self, user, userID, robotID, key):
        """
        """
        return self.obj.callRemote('createNamespace', user, userID,
                                   robotID, key)
    
    def destroy(self):
        """
        """
        if self._distributor:
            self._distributor.unregisterRelay(self)
            self._distributor = None
            
            super(RobotEndpoint, self).destroy()
        else:
            print('robot.RobotEndpoint destroy() called multiple times...')
