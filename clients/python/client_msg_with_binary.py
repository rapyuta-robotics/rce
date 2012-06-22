#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       client_msg_with_binary.py
#       
#       This file is part of the RoboEarth Cloud Engine clients.
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

import uuid
import json

class EchoClientProtocol(WebSocketClientProtocol):

    def sendBinary(self):
        f = open('chillie.png','r')
        data = f.read()
        f.close()
        self.sendMessage(self.tmpUID+data,binary=True)

    def onOpen(self):
        
        self.tmpUID = uuid.uuid4().hex
        
        # if part of data is being sent as binary key[-1:]=='*'
        msg = {"string1":"one","string2":"two","binData*":self.tmpUID}; 
        cmd_RM = {
            "type":"RM",
            "dest":["destination_container","interfaceID"],
            "orig":"origin_robot",
            "data":{
                "type":"my_msgs/2str1bin", 
                "msg":msg,
                "msgID":"mid"}
        }
        
        cmd_RM_js = json.dumps(cmd_RM)
        self.sendMessage(cmd_RM_js)
        reactor.callLater(2, self.sendBinary)
        
    def onMessage(self, msg, binary):
        print "received message from server!"
        

if __name__ == '__main__':
    factory = WebSocketClientFactory("ws://localhost:9000")
    factory.protocol = EchoClientProtocol
    connectWS(factory)
    print 'calling reactor.run()..'
    reactor.run()
    print 'Done'
