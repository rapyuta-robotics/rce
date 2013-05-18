#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     rce-comm/rce/comm/protocol.py
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

# zope specific imports
from zope.interface import implements

# twisted specific imports
# from twisted.python import log
from twisted.internet.interfaces import IPullProducer

# rce specific imports
from rce.comm.assembler import recursiveBinarySearch, MessageAssembler
from rce.comm.queue import PriorityQueue


class RCERobotProtocolMixin(object):
    """ Protocol which is used for the connections from the robots to the
        robot manager.
    """
    implements(IPullProducer)

    # CONFIG
    MSG_QUEUE_TIMEOUT = 60

    def __init__(self, *args, **kw):
        """ Initialize the Protocol Mixin.
        """
        self._assembler = MessageAssembler(self, self.MSG_QUEUE_TIMEOUT)
        self._queue = PriorityQueue(self.MSG_QUEUE_TIMEOUT)
        self._waitingForMessage = False

    def start(self):
        """ Start the Protocol utilities.
        """
        self._assembler.start()
        self._queue.start()

        self.transport.registerProducer(self, False)

    def stop(self):
        """ Stop Start the Protocol utilities.
        """
        if self._assembler:
            self._assembler.stop()
            self._assembler = None

        if self._queue:
            self._queue.stop()
            self._queue = None

    def processCompleteMessage(self, msg):
        """ Process complete messages by calling the appropriate handler for
            the manager. (Called by rce.comm.assembler.MessageAssembler)
        """
        raise NotImplementedError

    def processMessage(self, msg, binary):
        """ Process a received message by sending the message to the message
            assembler.
        """
#        log.msg('WebSocket: Received new message from other side. '
#                '(binary={0})'.format(binary))

        self._assembler.processMessage(msg, binary)

    def addToQueue(self, msg, priority):
        """ Add a message to the output queue.
        """
        if self._waitingForMessage:
            self._waitingForMessage = False
            self._sendMessage(msg)
        else:
            self._queue.push(msg, priority)

    def _sendMessage(self, msg):
        """ Internally used method to send a message to the robot.

            @param msg:     Message which should be sent.
        """
        uriBinary, msgURI = recursiveBinarySearch(msg)

        self.sendMessage(json.dumps(msgURI))

        for data in uriBinary:
            self.sendMessage(data[0] + data[1].getvalue(), binary=True)

    def resumeProducing(self):
        """ Method is called by the twisted transport when the transport is
            ready to send a new message.
        """
        try:
            self._sendMessage(self._queue.pop())
        except IndexError:
            self._waitingForMessage = True

    def stopProducing(self):
        """ Method is called by the twisted transport when the transport is
            no longer able to send messages.
        """
        self.transport.unregisterProducer()
