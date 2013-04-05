#!/usr/bin/env python
# -*- coding: utf-8 -*-
#     
#     master.py
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
from twisted.python import log
from twisted.cred.credentials import UsernamePassword
from twisted.cred.portal import Portal
from twisted.spread.pb import PBServerFactory
from twisted.web.server import Site
from twisted.application.internet import TCPServer
from twisted.application.service import Application, Service, IServiceCollection

# Custom imports
from rce.client.protocol import MasterRobotAuthentication
from rce.util.network import getIP
from rce.master.core import RoboEarthCloudEngine
from rce.master.console import Console, ConsoleDummyRealm

# twisted specific imports
#from twisted.internet import reactor
from twisted.cred.checkers import InMemoryUsernamePasswordDatabaseDontUse

# Custom imports
from rce.util.cred import RCECredChecker, RCEInternalChecker
import settings


# Credentials checkers used in the cloud engine
extCred = RCECredChecker(settings.PASSWORD_FILE, settings.DEV_MODE)
intCred = RCEInternalChecker(settings.PASSWORD_FILE)


rce = RoboEarthCloudEngine(extCred, settings.INT_IF, settings.RCE_INTERNAL_PORT)
consolerealm = ConsoleDummyRealm(rce)
intCred.add_checker(rce.checkUIDValidity)
# Internal communication
p = Portal(rce, (intCred,))
internalserver = TCPServer(settings.MASTER_PORT, PBServerFactory(p))

# Client Connection
robotserver = TCPServer(settings.HTTP_PORT, Site(MasterRobotAuthentication(rce)))

pconsole = Portal(consolerealm, (extCred,))

#Console Connection
consoleserver = TCPServer(settings.RCE_CONSOLE_PORT, PBServerFactory(pconsole))
application = Application("master")
s = IServiceCollection(application)
internalserver.setServiceParent(s)
robotserver.setServiceParent(s)
consoleserver.setServiceParent(s)

class FooService(Service):

    def stopService(self):
        rce.preShutdown()
        #Service.stopService(self)
        rce.postShutdown()

foo = FooService()
foo.setName('foo')
foo.setServiceParent(s)
#reactor.addSystemEventTrigger('before', 'shutdown', rce.preShutdown)
#reactor.addSystemEventTrigger('after', 'shutdown', rce.postShutdown)

#reactor.run()
