#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       client.py
#       
#       Copyright 2012 dominique hunziker <dominique.hunziker@gmail.com>
#       
#       This program is free software; you can redistribute it and/or modify
#       it under the terms of the GNU General Public License as published by
#       the Free Software Foundation; either version 2 of the License, or
#       (at your option) any later version.
#       
#       This program is distributed in the hope that it will be useful,
#       but WITHOUT ANY WARRANTY; without even the implied warranty of
#       MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#       GNU General Public License for more details.
#       
#       You should have received a copy of the GNU General Public License
#       along with this program; if not, write to the Free Software
#       Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#       MA 02110-1301, USA.
#       
#       



from twisted.internet.protocol import Factory, Protocol
from twisted.internet.endpoints import TCP4ClientEndpoint
from twisted.internet import reactor

import time
from threading import Thread

global start

class TestProtocolClient(Protocol):
    def __init__(self, parent):


    def connectionMade(self):


    def dataReceived(self, data):
        print data

        if 'Tschau' in data:
            print 'Duration: {0}'.format(time.time() - start)

    def _sendMsg(self, msg):
        print 'Send: {0}'.format(msg)
        self.transport.write("MESSAGE %s\n" % msg)

    def sendMessage(self, msg):
        reactor.callFromThread(self._sendMsg, msg)

class TestFactoryClient(Factory):
    def buildProtocol(self, addr):
        print 'Connected.'
        return TestProtocolClient()

class CommRunner(Thread):
    def __init__(self, callback):
        super(CommRunner, self).__init__()
        self._callback = callback

    def gotProtocol(self, p):
        print 'Got protocol: {0}'.format(p)
        reactor.callLater(0.1, p.sendMessage, 'Hello too!')

        self._callback(p)

    def reportError(self, p):
        print 'Got error: {0}'.format(p)

    def dbg(self):
        print 'Debug'

    def run(self):
        endpoint = TCP4ClientEndpoint(reactor, 'localhost', 10023)
        d = endpoint.connect(TestFactoryClient())
        d.addCallbacks(self.gotProtocol, self.reportError)
        reactor.run(installSignalHandlers=False)

def sendMessages(p):
    reactor.callLater(0.5, p.sendMessage, 'Wie gehts?')
    reactor.callLater(1.5, p.sendMessage, 'Was gibts?')
    reactor.callLater(2.5, p.sendMessage, 'Tschau')

def terminate():
    print 'terminate'
    reactor.callFromThread(reactor.stop)

if __name__ == '__main__':
    rospy.init_node('test')
    rospy.on_shutdown(terminate)

    global start
    start = time.time()

    runner = CommRunner(sendMessages)

    try:
        runner.run()
    finally:
        print 'Exiting...'
