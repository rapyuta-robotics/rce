#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     twisted_test.py
#
#     This file is part of the RoboEarth Cloud Engine framework.
#
#     This file was originally created for RoboEearth
#     http://www.roboearth.org/
#
#     The research leading to these results has received funding from
#     the European Union Seventh Framework Programme FP7/2007-2013 under
#     grant agreement no248942 RoboEarth.
#
#     Copyright 2013 RoboEarth
#
#     Licensed under the Apache License, Version 2.0 (the "License");
#     you may not use this file except in compliance with the License.
#     You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
#     Unless required by applicable law or agreed to in writing, software
#     distributed under the License is distributed on an "AS IS" BASIS,
#     WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#     See the License for the specific language governing permissions and
#     limitations under the License.
#
#     \author/s: Dominique Hunziker
#
#

# Python specific imports
import json

# twisted specific imports
from twisted.internet.defer import Deferred
from twisted.internet.protocol import Protocol, ClientFactory

# local imports
from base import PASSES, SIZES, LocalTest


class TwistedTest(LocalTest):
    def _activate(self):
        super(TwistedTest, self)._activate()
        print('Run twisted test...')
    
    def _sendReq(self):
        self._conn.sendMessage(self._str, self._resp)


class Connection(Protocol):
    def __init__(self, reactor, deferred):
        self._reactor = reactor
        self._deferred = deferred

        self._len = 0
        self._buf = ''
        self._cb = None

    @property
    def reactor(self):
        return self._reactor

    def sendMessage(self, msg, cb):
        assert self._cb is None
        self._cb = cb
        self._len = len(msg)
        self.transport.write(msg)

    def dataReceived(self, data):
        self._buf += data
        
        if len(self._buf) == self._len:
            assert self._cb is not None
            buf, self._buf = self._buf, ''
            cb, self._cb = self._cb, None
            cb(buf)
        elif len(self._buf) > self._len:
            raise ValueError('Message is too big.')

    def connectionMade(self):
        if self._deferred:
            self._deferred.callback(self)

    def connectionLost(self, reason):
        if self._deferred and not self._deferred.called:
            self._deferred.errback(self)


class ConnectionFactory(ClientFactory):
    def __init__(self, reactor, deferred):
        self._reactor = reactor
        self._deferred = deferred

    def buildProtocol(self, addr):
        p = Connection(self._reactor, self._deferred)
        p.factory = self
        return p


class Measurement(object):
    def __init__(self, runs):
        self._runs = runs

    def run(self, conn):
        self._tests = [TwistedTest(conn, '', 'twisted')]

        d = Deferred()

        for _ in xrange(self._runs):
            for test in self._tests:
                d.addCallback(test.run)

        d.addCallback(self._postProcess)
        d.callback(None)

        def stop(_):
            reactor.stop()
            print('\ndone')

        d.addCallback(stop)

    def _postProcess(self, _):
        with open('twisted.data', 'w') as f:
            f.write(json.dumps(SIZES))
            f.write('\n')

            for test in self._tests:
                f.write(str(test))
                f.write('\n')


def _get_argparse():
    from argparse import ArgumentParser

    parser = ArgumentParser(prog='twisted_test',
                            description='Run communication measurement for '
                                        'Twisted using a string message.')

    parser.add_argument('--passes', help='Number of passes to do.',
                        type=int, default=PASSES)
    parser.add_argument('ip', help='IP address of WebSocket Echo Server.',
                        type=str)

    return parser


def main(reactor, passes, ip):
    measurement = Measurement(passes)

    d = Deferred()
    d.addCallback(measurement.run)

    factory = ConnectionFactory(reactor, d)
    reactor.connectTCP(ip, 8000, factory)

    reactor.run()


if __name__ == '__main__':
    from twisted.internet import reactor

    args = _get_argparse().parse_args()

    main(reactor, args.passes, args.ip)
