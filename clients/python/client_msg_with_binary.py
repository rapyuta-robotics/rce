#!/usr/bin/env python
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
    print 'calling reactor.run() ..'
    reactor.run()
    print 'Done'
