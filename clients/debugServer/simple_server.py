#!/usr/bin/env python
from twisted.internet import reactor
from autobahn.websocket import WebSocketServerFactory, WebSocketServerProtocol, listenWS

import json
import uuid
from Queue import Queue

from ROSProxy import ROSProxy

class Container(object):
    def __init__(self, connection):
        self.connection = connection
        self.containerID = None

class connectionManager(object):
    def __init__(self, protocol):
        self.proxy = ROSProxy()
        self.comntainerQueue = Queue()
        self.protocol()
        
    def createContainer(self, robotUID):
        pass
        
    
class CloudEngineProtocol(WebSocketServerProtocol):
    
	incoming_msg_count = 0 
	cmd_CSR = {"type":"CSR", "dest":None, "orig":"$$$$$$", "data":{"containerID":None}}

	def onConnect(self, _):
		pass
		  
    def connectionMade(self):
        self.connectionManager = connectionManager(self)
        
	def onMessage(self, msg, binary):
        # Debug 
        self.incoming_msg_count += 1
        print('received Message # '+str(self.incoming_msg_count))

		if not binary:
			cmd = json.loads(msg)
			if cmd['type']=="CS":
				print "received container creation method"
				self.cmd_CSR['dest']=cmd['orig']
				self.cmd_CSR['data']['containerID']=uuid.uuid4().hex
				self.sendMessage(json.dumps(self.cmd_CSR))
			else:
				self.sendMessage('received Message # '+str(self.incoming_msg_count))
		else:
			pass

if __name__ == '__main__':
	factory = WebSocketServerFactory("ws://localhost:9000")
	factory.protocol = CloudEngineProtocol
	listenWS(factory)
	reactor.run()