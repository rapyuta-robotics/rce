#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     rce-core/rce/core/webUI.py
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
#     Copyright 2014 RoboEarth
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
import os
import json

# twisted specific imports
from twisted.web.resource import Resource
from twisted.web.static import File

# Autobahn specific imports
from autobahn.websocket import WebSocketServerProtocol, WebSocketServerFactory

# rce specific imports
import rce.core
from rce.util.resource import getModulePath


class WebUIFeedProtocol(WebSocketServerProtocol):
    def __init__(self, user):
        self._user = user

    def onOpen(self):
        self._user.registerListener(self)

    def onClose(self, *_):
        self._user.unregisterListener(self)

    def feedUpdate(self, update):
        self.sendMessage(json.dumps(update))


class WebUIFeedFactory(WebSocketServerFactory):
    def __init__(self, url, user, **kw):
        WebSocketServerFactory.__init__(self, url, **kw)

        self._user = user

    def buildProtocol(self, addr):
        protocol = WebUIFeedProtocol(self._user)
        protocol.factory = self
        return protocol


def getRootResource():
    fileSrc = os.path.join(getModulePath(rce.core), 'web')

    resource = Resource()
    resource.putChild('rapyuta.js', File(os.path.join(fileSrc, 'rapyuta.js')))
    resource.putChild('rapyuta.css', File(os.path.join(fileSrc, 'rapyuta.css')))
    resource.putChild('', File(os.path.join(fileSrc, 'index.html')))

    return  resource
