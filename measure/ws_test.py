#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     ws_test.py
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

# Autobahn specific imports
from autobahn.websocket import WebSocketClientFactory, \
                               WebSocketClientProtocol, \
                               connectWS

# local imports
from base import PASSES, SIZES, LocalTest


class WebSocketTest(LocalTest):
    def __init__(self, conn, binary, testType):
        super(WebSocketTest, self).__init__(conn, None, testType)
        self._binary = binary

    def _activate(self):
        super(WebSocketTest, self)._activate()
        print('Run twisted test...')

    def _sendReq(self):
        self._conn.sendMessage(self._str, self._binary, self._resp)


class Connection(WebSocketClientProtocol):
    def __init__(self, reactor, deferred):
        self._reactor = reactor
        self._deferred = deferred
        self._cb = None

    @property
    def reactor(self):
        return self._reactor

    def sendMessage(self, msg, binary, cb):
        assert self._cb is None
        self._cb = cb
        WebSocketClientProtocol.sendMessage(self, msg, binary)

    def onMessage(self, msg, binary):
        assert self._cb is not None
        cb, self._cb = self._cb, None
        cb(msg)

    def onOpen(self):
        if self._deferred:
            self._deferred.callback(self)

    def onClose(self, wasClean, code, reason):
        if self._deferred and not self._deferred.called:
            self._deferred.errback(self)


class ConnectionFactory(WebSocketClientFactory):
    def __init__(self, url, reactor, deferred, **kw):
        WebSocketClientFactory.__init__(self, url, **kw)

        self._reactor = reactor
        self._deferred = deferred

    def buildProtocol(self, addr):
        p = Connection(self._reactor, self._deferred)
        p.factory = self
        return p


class Measurement(object):
    TYPES = ((False, 'text'), (True, 'binary'))

    def __init__(self, runs):
        self._runs = runs

    def run(self, conn):
        self._tests = [WebSocketTest(conn, b, t) for b, t in self.TYPES]

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
        with open('websocket.data', 'w') as f:
            f.write(json.dumps(SIZES))
            f.write('\n')

            for test in self._tests:
                f.write(str(test))
                f.write('\n')


def _get_argparse():
    from argparse import ArgumentParser

    parser = ArgumentParser(prog='ws_test',
                            description='Run communication measurement for '
                                        'WebSocket using a string message.')

    parser.add_argument('--passes', help='Number of passes to do.',
                        type=int, default=PASSES)
    parser.add_argument('ip', help='IP address of WebSocket Echo Server.',
                        type=str)

    return parser


def main(reactor, passes, ip):
    measurement = Measurement(passes)

    d = Deferred()
    d.addCallback(measurement.run)

    factory = ConnectionFactory('ws://{0}:9000'.format(ip), reactor, d)
    connectWS(factory)

    reactor.run()


if __name__ == '__main__':
    from twisted.internet import reactor

    args = _get_argparse().parse_args()

    main(reactor, args.passes, args.ip)
