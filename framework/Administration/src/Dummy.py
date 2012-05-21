'''
Created on May 21, 2012

@author: dominique
'''

from zope.interface import implements

import sys

from Comm.UIDServer import IUIDServer, UIDServerFactory #@UnresolvedImport

class UIDServer(object):
    implements(IUIDServer)
    
    def getUID(self):
        return 'Hello World!'

def main(reactor, port):
    reactor.listenTCP(port, UIDServerFactory(UIDServer()))
    reactor.run()

if __name__ == '__main__':
    from twisted.internet import reactor
    main(reactor, int(sys.argv[1]))
