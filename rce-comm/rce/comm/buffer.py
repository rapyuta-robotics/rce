#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     rce-comm/rce/comm/client.py
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


class BufferManager(object):
    implements(IPullProducer)

    def __init__(self, consumer, protocol):
        self.consumer = consumer
        self.protocol = protocol

    def resumeProducing(self):
        self._paused = False
        try:
            while True:
                data = self.protocol._binary_buff.popleft()
                msg = data[0] + data[1].getvalue()
                WebSocketProtocol.sendMessage(self.protocol, msg, binary=True)
                if self.paused:
                    break
        except IndexError:
            pass

    def stopProducing(self):
        self._paused = True
