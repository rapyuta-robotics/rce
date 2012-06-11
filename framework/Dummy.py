'''
Created on May 21, 2012

@author: dominique
'''

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
