#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       echo_client.py
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
#       \author/s: Gajamohan Mohanarajah
#       
#       

from twisted.internet import reactor
from autobahn.websocket import WebSocketClientFactory, WebSocketClientProtocol, connectWS

class rceClientProtocol(WebSocketClientProtocol):
    def sendHello(self):
        self.sendMessage("Hello, world!")

    def onOpen(self):
        self.sendHello()

    def onMessage(self, msg, binary):
        print "Got echo: " + msg
        reactor.callLater(1, self.sendHello)

if __name__ == '__main__':
    factory = WebSocketClientFactory("ws://192.168.237.131:9000")
    factory.protocol = rceClientProtocol
    connectWS(factory)
    reactor.run()
