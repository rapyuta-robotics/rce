#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     base.py
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
import time
import random
import string
import json

# twisted specific imports
from twisted.internet.defer import Deferred


PASSES = 3

# Default
SIZES = [3, 10, 20, 42, 88, 183, 379, 784, 1623, 3359, 6951, 14384, 29763,
         61584, 127427, 263665, 545559, 1128837, 2335721, 4832930, 10000000]

# Amazon R2C
#SIZES = [3, 10, 20, 42, 88, 183, 379, 784, 1623, 3359,
#         6951, 7938, 9310, 11216, 13864, 17544, 22658, 29763,
#         61584, 127427, 263665, 545559, 1128837, 2335721, 4832930, 10000000]


def delay(response, delta, reactor):
    d = Deferred()
    reactor.callLater(delta, d.callback, response)
    return d


class TestBase(object):
    def __init__(self, conn, iTag, testType):
        self._conn = conn
        self._iTag = iTag
        self._testType = testType

        self._data = []

    def _activate(self):
        pass

    def _deactivate(self):
        pass

    def _run(self):
        raise NotImplementedError

    def run(self, _):
        self._deferred = Deferred()
        self._conn.reactor.callLater(1, self._activate)
        self._conn.reactor.callLater(2, self._run)
        return self._deferred

    def _done(self):
        self._deactivate()
        self._deferred.callback(None)

    def __str__(self):
        return json.dumps({'type' : self._testType, 'data' : self._data})


class RemoteTest(TestBase):
    def _activate(self):
        self._srv = self._conn.serviceClient(self._iTag, 'Test/StringTest',
                                             self._process)

    def _deactivate(self):
        self._srv = None

    def _run(self):
        self._srv.call({'testType' : self._testType})

    def _process(self, msg):
        self._data.append(msg['times'])
        self._conn.reactor.callLater(0, self._done)


class LocalTest(TestBase):
    def _activate(self):
        self._data.append([])
        self._str = None
        self._ready = False
        self._dropCnt = 0

    def _run(self):
        self._req()

    def _req(self):
        count = len(self._data[-1])

        if count >= len(SIZES):
            self._conn.reactor.callLater(0, self._done)
            return

        if SIZES[count] > 10:
            sample = ''.join(random.choice(string.lowercase)
                             for _ in xrange(10))
            rep = int(SIZES[count] / 10)
            tail = 'A' * (SIZES[count] % 10)

            self._str = sample * rep + tail
        else:
            self._str = ''.join(random.choice(string.lowercase)
                                for _ in xrange(SIZES[count]))

        self._time = time.time()
        self._sendReq()

    def _resp(self, data):
        stop = time.time()

        if not self._ready:
            if self._dropCnt > 3:
                self._data[-1] = [-1.0] * len(SIZES)
                self._conn.reactor.callLater(0, self._done)
                return

            if (not self._str
                or len(self._str) != len(data) or self._str[:10] != data[:10]):
                self._dropCnt += 1
                return

            self._ready = True

        if len(self._str) != len(data) or self._str[:10] != data[:10]:
            delta = -1
        else:
            delta = (stop - self._time) * 1000

        self._data[-1].append(delta)
        self._req()

    def _sendReq(self):
        raise NotImplementedError


class LocalServiceTest(LocalTest):
    def __init__(self, conn, iTag):
        super(LocalServiceTest, self).__init__(conn, iTag, 'service')

    def _activate(self):
        super(LocalServiceTest, self)._activate()

        self._srv = self._conn.serviceClient(self._iTag, 'Test/StringEcho',
                                             self._resp)

    def _deactivate(self):
        self._srv = None

    def _sendReq(self):
        self._srv.call({'data' : self._str})

    def _resp(self, resp):
        super(LocalServiceTest, self)._resp(resp['data'])


class LocalTopicTest(LocalTest):
    def __init__(self, conn, iTag):
        super(LocalTopicTest, self).__init__(conn, iTag, 'topic')

    def _activate(self):
        super(LocalTopicTest, self)._activate()

        self._pub = self._conn.publisher(self._iTag[0], 'std_msgs/String')
        self._sub = self._conn.subscriber(self._iTag[1], 'std_msgs/String',
                                          self._resp)

    def _deactivate(self):
        self._pub = None
        self._sub.unsubscribe()
        self._sub = None

    def _sendReq(self):
        self._pub.publish({'data' : self._str})

    def _resp(self, resp):
        super(LocalServiceTest, self)._resp(resp['data'])
