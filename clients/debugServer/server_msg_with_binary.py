#!/usr/bin/env python
from twisted.internet import reactor
from autobahn.websocket import WebSocketServerFactory, WebSocketServerProtocol, listenWS

import json

class EchoServerProtocol(WebSocketServerProtocol):

    completelyReceivedMsgs = []
    incompleteMsgs = []
    
    def onMessage(self, msg, binary):
        if not binary:
            print 'Received a non binary message'
            msg_pydict = json.loads(msg)
            
            if msg_pydict['type'] == 'RM':
                print '-- type: data message'
                
                uidList = []
                for key in msg_pydict['data']['msg']:
                    if key[-1:] == '*':
                       uidList.append(msg_pydict['data']['msg'][key])
                    
                if len(uidList) == 0:
                    self.completelyReceivedMsgs.append({"sMsg":msg_pydict})
                else: 
                    tmpDict = {"sMsg":msg_pydict}
                    for uid in uidList:
                            tmpDict[uid]=None
                    self.incompleteMsgs.append(tmpDict)
                    
        else: 
            print 'Received a binary message'
            uid = msg[:32]
            data = msg[33:]
            print "-- uid: " + uid
            
            complete = False 
            msgDict_idx = 0
            
            # Assuming binary data arrives after the json string (TODO:Check Validity)
            if len(self.incompleteMsgs) == 0:
                pass
            else:
                try:
                    for idx,msgDict in enumerate(self.incompleteMsgs):
                        for key in msgDict:
                            if key == uid: 
                                msgDict[key]=data
                                msgDict_idx = idx
                                # check if msgDict is complete
                                complete = True
                                for key in msgDict:
                                    if msgDict[key]==None:
                                            complete=False
                                raise StopIteration()
                except StopIteration:
                    pass
                
                if complete:
                    self.completelyReceivedMsgs.append(self.incompleteMsgs.pop(msgDict_idx))
                    
            print 'finished handling binary msgs'
            print 'completelyReceivedMsgs: '
            print(self.completelyReceivedMsgs)
            print 'incompleteMsgs: '
            print(self.incompleteMsgs)
                            
            
if __name__ == '__main__':

	factory = WebSocketServerFactory("ws://localhost:9000")
	factory.protocol = EchoServerProtocol
	listenWS(factory)
	reactor.run()
