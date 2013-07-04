#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     center.py
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
import string
import random
import time
from threading import Event

# ROS specific imports
import roslib; roslib.load_manifest('Test')
import rospy
from std_msgs.msg import String

# rce test specific imports
from Test.srv import StringEcho


SIZES = [3, 10, 20, 42, 88, 183, 379, 784, 1623, 3359, 6951, 14384, 29763,
         61584, 127427, 263665, 545559, 1128837, 2335721, 4832930, 10000000]


class TestCenter(object):
    _TIMEOUT = 15

    def __init__(self):
        self._times = []

        self._counter = 0
        self._event = None
        self._pub = None
        self._str = None
        self._time = None

        rospy.Subscriber('stringEchoResp', String, self._resp)

    def _runService(self):
        try:
            rospy.wait_for_service('stringEchoService', timeout=5)
        except rospy.ROSException:
            return [-1.0] * len(SIZES)

        times = []

        serviceFunc = rospy.ServiceProxy('stringEchoService', StringEcho)

        for size in SIZES:
            if size > 10:
                sample = ''.join(random.choice(string.lowercase)
                                 for _ in xrange(10))
                s = sample * int(size / 10) + 'A' * (size % 10)
            else:
                s = ''.join(random.choice(string.lowercase)
                            for _ in xrange(size))

            start = time.time()
            response = serviceFunc(s)
            end = time.time()

            if response.data != s:
                times.append(-1.0)
            else:
                times.append((end - start) * 1000)

        return times

    def _req(self):
        if self._counter >= len(SIZES):
            self._event.set()
        else:
            if SIZES[self._counter] > 10:
                sample = ''.join(random.choice(string.lowercase)
                                 for _ in xrange(10))
                rep = int(SIZES[self._counter] / 10)
                tail = 'A' * (SIZES[self._counter] % 10)

                self._str = sample * rep + tail
            else:
                self._str = ''.join(random.choice(string.lowercase)
                                    for _ in xrange(SIZES[self._counter]))

            self._time = time.time()
            self._pub.publish(self._str)

    def _resp(self, msg):
        stop = time.time()

        if not self._pub:
            return

        if len(self._str) == len(msg.data) and self._str[:10] == msg.data[:10]:
            self._times.append((stop - self._time) * 1000)
            self._time = None
            self._counter += 1
            self._req()

    def _runTopic(self):
        self._counter = 0
        self._times = []
        self._event = Event()
        self._pub = rospy.Publisher('stringEchoReq', String, latch=True)
        self._req()

        if (not self._event.wait(self._TIMEOUT) or
                len(self._times) != len(SIZES)):
            times = [-1.0] * len(SIZES)
        else:
            times = self._times

        self._pub = None
        return times

    def runTest(self, test):
        if test == 'service':
            times = self._runService()
        elif test == 'topic':
            times = self._runTopic()
        else:
            print('Unknown test: {0}'.format(test))
            times = [-1.0]

        return times
