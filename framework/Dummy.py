#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       Dummy.py
#       
#       This file is part of the RoboEarth Cloud Engine framework.
#       
#       This file was originally created for RoboEearth
#       http://www.roboearth.org/
#       
#       The research leading to these results has received funding from
#       the European Union Seventh Framework Programme FP7/2007-2013 under
#       grant agreement no248942 RoboEarth.
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
#       \author/s: Dominique Hunziker 
#       
#       

from zope.interface import implements
from twisted.internet.defer import Deferred

import sys

from MasterUtil.UIDServer import IUIDServer, UIDServerFactory
from MasterUtil.UIDClient import UIDClientFactory

class UIDServer(object):
    implements(IUIDServer)
    
    def getUID(self):
        return 'Hello World!'

def mainServer(reactor, port):
    reactor.listenTCP(port, UIDServerFactory(UIDServer()))
    reactor.run()

def mainClient(reactor, ip, port):
    deferred = Deferred()
    
    def callback(arg):
        print arg
        reactor.stop()
    
    deferred.addCallbacks(callback, callback)
    
    reactor.connectTCP(ip, port, UIDClientFactory(deferred))
    reactor.run()

if __name__ == '__main__':
    from twisted.internet import reactor
    
    if len(sys.argv) == 2:
        mainServer(reactor, int(sys.argv[1]))
    elif len(sys.argv) == 3:
        mainClient(reactor, sys.argv[1], int(sys.argv[2]))
