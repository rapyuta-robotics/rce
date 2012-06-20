#!/usr/bin/env python
from twisted.internet import reactor
from autobahn.websocket import WebSocketServerFactory, WebSocketServerProtocol, listenWS
import uuid
import json

class CloudEngineProtocol(WebSocketServerProtocol):
    def onMessage(self, msg, binary):
        if binary:
            # Attach a different UUID and send the same binary back
            print 'UUID -> '+ msg[:32]
            tmpUUID = uuid.uuid4().hex
            print 'UUID <- '+ tmpUUID
            JSONImageMsg = "{'type':'DM','dest':'robotUniqueID','orig':'containerTag01','data':{'type':'sensor_msgs/Image','msgID':'msgID_0','interfaceTag':'Test/modifyImage','msg':{'data':'+tmpUUID+'}}}"
            self.sendMessage(JSONImageMsg)
            self.sendMessage(tmpUUID+msg[32:], binary=True)

        else:
            #self.sendMessage('received non binary message')
            print '== Msg from client =='
            print msg
            data = json.loads(msg);
            
            if data['type'] == 'CC':
                cmd_CS = {
                    'type':'CS',
                    'dest':'robotUniqueID',
                    'orig':'$$$$$$',
                    'data':{data['data']['containerTag']:True} 
                    #'data':{'tag04':True} #Send wring tag 
                    }
                cmd_CS_js = json.dumps(cmd_CS)
                self.sendMessage(cmd_CS_js)
                print 'dummy container created'

    def onOpen(self):
        print 'connection made.'
        

if __name__ == '__main__':
    factory = WebSocketServerFactory('ws://localhost:9000')
    factory.protocol = CloudEngineProtocol
    listenWS(factory)
    reactor.run()
