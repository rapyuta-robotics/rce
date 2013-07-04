#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     rosbridge.py
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
from collections import defaultdict, Counter

# twisted specific imports
from twisted.internet.defer import Deferred

# Autobahn specific imports
from autobahn.websocket import WebSocketClientFactory, \
                               WebSocketClientProtocol, \
                               connectWS

# local imports
from base import PASSES, SIZES, LocalServiceTest, LocalTopicTest


class Interface(object):
    def __init__(self, conn, addr):
        self._conn = conn
        self._addr = '/{0}'.format(addr)


class CBInterface(Interface):
    _INTERFACE_TYPE = None

    def __init__(self, conn, addr, cb):
        super(CBInterface, self).__init__(conn, addr)

        self._cb = cb
        conn.registerCb(self._INTERFACE_TYPE, self._addr, cb)
        self._registered = True

    def unsubscribe(self):
        if self._registered:
            self._conn.unregisterCb(self._INTERFACE_TYPE, self._addr, self._cb)
            self._registered = False

    def __del__(self):
        self.unsubscribe()


class ServiceClient(CBInterface):
    _INTERFACE_TYPE = 'srv'

    def call(self, msg):
        self._conn.sendMessage(json.dumps({'op':'call_service',
                                           'service':self._addr,
                                           'args':msg}))


class Publisher(Interface):
    def __init__(self, conn, addr, msgType):
        super(Publisher, self).__init__(conn, addr)

        conn.sendMessage(json.dumps({'op':'advertise',
                                     'topic':self._addr,
                                     'type':msgType}))

    def publish(self, msg):
        self._conn.sendMessage(json.dumps({'op':'publish',
                                           'topic':self._addr,
                                           'msg':msg}))

    def __del__(self):
        self._conn.sendMessage(json.dumps({'op':'unadvertise',
                                           'topic':self._addr}))


class Subscriber(CBInterface):
    _INTERFACE_TYPE = 'tpc'

    def __init__(self, conn, addr, cb):
        super(Subscriber, self).__init__(conn, addr, cb)

        conn.sendMessage(json.dumps({'op':'subscribe',
                                     'topic':self._addr}))

    def unsubscribe(self):
        if self._registered:
            self._conn.sendMessage(json.dumps({'op':'unsubscribe',
                                               'topic':self._addr}))

        super(Subscriber, self).unsubscribe()


class Connection(WebSocketClientProtocol):
    def __init__(self, reactor, deferred):
        self._reactor = reactor
        self._deferred = deferred

        self._srvCb = defaultdict(Counter)
        self._tpcCb = defaultdict(Counter)

    @property
    def reactor(self):
        return self._reactor

    def serviceClient(self, addr, _, cb):
        return ServiceClient(self, addr, cb)

    def publisher(self, addr, msgType):
        return Publisher(self, addr, msgType)

    def subscriber(self, addr, _, cb):
        return Subscriber(self, addr, cb)

    def registerCb(self, iType, addr, cb):
        if iType == 'srv':
            self._srvCb[addr][cb] += 1
        elif iType == 'tpc':
            self._tpcCb[addr][cb] += 1
        else:
            raise TypeError('Invalid interface type ({0}).'.format(iType))


    def unregisterCb(self, iType, addr, cb):
        if iType == 'srv':
            cbs = self._srvCb[addr]
        elif iType == 'tpc':
            cbs = self._tpcCb[addr]
        else:
            raise TypeError('Invalid interface type ({0}).'.format(iType))

        cbs[cb] -= 1

        if not cbs[cb]:
            del cbs[cb]

    def onMessage(self, msg, binary):
        msg = json.loads(msg)

        if msg['op'] == 'service_response':
            data = msg.get('values', {})

            for cb in self._srvCb[msg['service']]:
                cb(data)
        elif msg["op"] == "publish":
            data = msg['msg']

            for cb in self._tpcCb[msg['topic']]:
                cb(data)

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
    TYPES = ((LocalServiceTest, 'stringEchoService'),
             (LocalTopicTest, ('stringEchoReq', 'stringEchoResp')))

    def __init__(self, runs):
        self._runs = runs

    def run(self, conn):
        self._tests = [cls(conn, iTag) for cls, iTag in self.TYPES]

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
        with open('rosbridge.data', 'w') as f:
            f.write(json.dumps(SIZES))
            f.write('\n')

            for test in self._tests:
                f.write(str(test))
                f.write('\n')


def _get_argparse():
    from argparse import ArgumentParser

    parser = ArgumentParser(prog='rosbridge',
                            description='Run communication measurement for '
                                        'rosbridge between a robot and a '
                                        'container using a string message.')

    parser.add_argument('--passes', help='Number of passes to do.',
                        type=int, default=PASSES)
    parser.add_argument('ipROSbridge', help='IP address of rosbridge.',
                        type=str)

    return parser


def main(reactor, passes, ip):
    measurement = Measurement(passes)

    d = Deferred()
    d.addCallback(measurement.run)

    factory = ConnectionFactory('ws://{0}:9090'.format(ip), reactor, d)
    connectWS(factory)

    reactor.run()


if __name__ == '__main__':
    from twisted.internet import reactor

    args = _get_argparse().parse_args()

    main(reactor, args.passes, args.ipROSbridge)
