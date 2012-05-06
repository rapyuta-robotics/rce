#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       server.py
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

from twisted.internet.protocol import Protocol, Factory
from twisted.internet.endpoints import SSL4ServerEndpoint
from twisted.internet.ssl import DefaultOpenSSLContextFactory
from twisted.internet.error import ConnectionDone

from twisted.internet import reactor

class TestProtocolServer(Protocol):
    def connectionMade(self):
        print 'Connection Made...'
        self.transport.write('Hello!\n')

    def connectionLost(self, reason):
        if reason.check(ConnectionDone):
            print 'Connection ended cleanly.'
        else:
            print 'Connection lost: {0}'.format(reason.getErrorMessage())

    def dataReceived(self, data):
        print data
        self.transport.write(data)

class TestFactoryServer(Factory):
    def buildProtocol(self, addr):
        return TestProtocolServer()

def main():
    endpoint = SSL4ServerEndpoint(reactor, 10023, DefaultOpenSSLContextFactory('key.pem', 'cert.pem'))
    endpoint.listen(TestFactoryServer())
    reactor.run()
    return 0

if __name__ == '__main__':
    main()

