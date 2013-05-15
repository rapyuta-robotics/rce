#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     rce-comm/rce/comm/buffer.py
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
#     \author/s: Dhananjay Sathe
#
#

from autobahn.websocket import WebSocketProtocol
from collections import defaultdict, deque


class BufferManager(object):
    implements(IPushProducer)

    def __init__(self, consumer, protocol):
        self.consumer = consumer
        self.protocol = protocol
        self._paused = False

    def resumeProducing(self):
        self._paused = False
        while not self._paused:
            try:
                data = self.protocol._binary_buff.popleft()
                msg = data[0] + data[1].getvalue()
                WebSocketProtocol.sendMessage(self.protocol, msg, binary=True)
            except IndexError:
                pass

    def pauseProducing(self):
        self._paused = True

    def stopProducing(self):
        pass

# TODO Implement Buffer Queues
class BufferQueue(object):
    "Priority Queues in ascending order"
    def __init__(self):
        self._queues = defaultdict(deque)
        self._prios = set()

    def append(self, item, priority=1):
        """ Adds an item to queues.
        Higher number indicates higher priority
        0 indicates real time 
        
        """
        self._curr_prio
        self._prios.add(prio)
        self._queues[priority].append(item)

    def get_item(self):
        """ Return the right element 
        """

        curr_prio = 0 if 0 in self._prios else max(self._prios)
        try:
            return self._queues[curr_prio].popleft()
        except IndexError:
            self._prios.remove(curr_prio)
            del self._queues[curr_prio]
