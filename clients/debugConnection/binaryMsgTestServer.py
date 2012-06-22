#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       binaryMsgTestServer.py
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
from autobahn.websocket import WebSocketServerFactory, WebSocketServerProtocol, listenWS
import uuid

class CloudEngineProtocol(WebSocketServerProtocol):
    def onMessage(self, msg, binary):
        if binary:
            #self.sendMessage('received binary message')
            #f = open('apple_copy.png','w')
            #f.write(msg[33:])
            #f.close()
            
            print 'UUID -> '+ msg[:32]
            #self.sendMessage(msg[32:], binary=True)
            tmpUUID = uuid.uuid4().hex
            print 'UUID <- '+ tmpUUID
            JSONImageMsg = '{"type":"CM","dest":"robotUniqueID","orig":"containerTag01","data":{"type":"sensor_msgs/Image","msgID":"msgID_0","interfaceTag":"Test/modifyImage","msg":{"data":'+tmpUUID+'}}}'
            self.sendMessage(JSONImageMsg)
            self.sendMessage(tmpUUID+msg[32:], binary=True)
            #self.sendMessage(msg, binary=True)
        else:
            #self.sendMessage('received non binary message')
            print msg
            
    def onOpen(self):
        print 'connection made.'
        

if __name__ == '__main__':
    factory = WebSocketServerFactory("ws://localhost:9000")
    factory.protocol = CloudEngineProtocol
    listenWS(factory)
    reactor.run()
