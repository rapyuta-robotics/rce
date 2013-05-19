#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     rce-comm/rce/comm/queue.py
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
from datetime import datetime, timedelta
from heapq import heappush, heappop, heapify

# twisted specific imports
from twisted.python import log
from twisted.internet.task import LoopingCall


class PriorityQueue(object):
    """ Priority Queue implementation with an automatic timeout for elements.
    """
    def __init__(self, timeout=30):
        """ Initialize the Priority Queue.

            @param timeout:     Timeout in seconds after which elements are
                                automatically removed from the queue.
            @type  timeout:     int
        """
        self._timeout = timedelta(seconds=timeout)
        self._queue = []
        self._cleaner = LoopingCall(self._cleanUp)

    def push(self, element, priority):
        """ Add an element to the queue.

            @param element:     Element which should be added to the queue.

            @param priority:    Priority which will be associated with the
                                element. The priority has to be larger than 0
                                with zero as the highest priority.
            @type  priority:    int
        """
        assert priority >= 0
        heappush(self._queue, (priority, datetime.now(), element))

    def pop(self):
        """ Retrieve the next element from the queue.

            @return:            Element which has the highest priority and which
                                is the oldest among equal priority elements.

            @raise:             IndexError if the queue is empty.
        """
        while 1:
            data = heappop(self._queue)
            if data[1] < datetime.now() + self._timeout:
                return data[2]

    def size(self):
        """ Get the current size of the queue.
        """
        return len(self._queue)

    def start(self):
        """ Start the cleaner of the queue.
        """
        self._cleaner.start(self._timeout.seconds / 4)

    def stop(self):
        """ Stop the cleaner of the queue and remove all remaining elements.
        """
        self._queue = []

        if self._cleaner.running:
            self._cleaner.stop()

    def _cleanUp(self):
        """ Clean up the queue by removing elements whose timeout has expired.
        """
        timeout = datetime.now() + self._timeout

        old_len = len(self._queue)
        self._queue = [ele for ele in self._queue if ele[1] < timeout]
        new_len = len(self._queue)

        if old_len != new_len:
            heapify(self._queue)
            log.msg('{0} elements have been dropped from '
                    'queue.'.format(old_len - new_len))
