#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     rce-comm/rce/comm/assembler.py
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
#     Copyright 2012 RoboEarth
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
from datetime import datetime, timedelta
from uuid import uuid4

try:
    from cStringIO import StringIO, InputType, OutputType
    from StringIO import StringIO as pyStringIO

    def _checkIsStringIO(obj):
        return isinstance(obj, (InputType, OutputType, pyStringIO))
except ImportError:
    from StringIO import StringIO

    def _checkIsStringIO(obj):
        return isinstance(obj, StringIO)

# twisted specific imports
from twisted.python import log
from twisted.internet.task import LoopingCall

# rce specific imports
from rce.comm.error import InvalidRequest


class AssemblerError(Exception):
    """ Exception is raised when an error in the message assembler occurs.
    """


def recursiveBinarySearch(multidict):
    """ Search a JSON message for StringIO instances which should be replaced
        with a reference to a binary message. Returns a list of all binary
        messages and the modified JSON string message.

        @param multidict:       JSON message which might contain StringIO
                                instances and which should be prepared for
                                sending.
        @type  multidict:       { str : ... }

        @return:                A list of tuples containing the URI and the
                                matching StringIO instance. Also the modified
                                JSON message where the StringIO instances have
                                been replaced with the URIs is returned.
        @rtype:                 ((str, StringIO), { str : ... })
    """
    uriBinary = []
    keys = []

    for k, v in multidict.iteritems():
        if isinstance(v, dict):
            uriBinaryPart, multidictPart = recursiveBinarySearch(v)
            uriBinary += uriBinaryPart
            multidict[k] = multidictPart
        elif isinstance(v, (list, tuple)):
            if v and _checkIsStringIO(v[0]):
                for e in v:
                    if not _checkIsStringIO(e):
                        raise ValueError('Can not mix binary and string '
                                         'message in an array.')

                keys.append(k)
        elif _checkIsStringIO(v):
            keys.append(k)

    for k in keys:
        ele = multidict.pop(k)

        if isinstance(ele, (list, tuple)):
            uris = []

            for e in ele:
                tmpURI = uuid4().hex
                uris.append(tmpURI)
                uriBinary.append((tmpURI, e))

            ele = uris
        else:
            tmpURI = uuid4().hex
            uriBinary.append((tmpURI, ele))
            ele = tmpURI

        multidict['{0}*'.format(k)] = ele

    return uriBinary, multidict


class _IncompleteMsg(object):
    """ Class which represents an incomplete class.
    """
    def __init__(self, assembler, msg, uris):
        """ Initialize the incomplete message.

            @param assembler:   Assembler to which the this instance
                                belongs.
            @type  assembler:   client.assembler.MessageAssembler

            @param msg:         Incomplete message as a dictionary.
            @type  msg:         dict

            @param uris:        Return value of _recursiveURISearch
            @type  nr:          [ (str, dict, str) ]
        """
        self._assembler = assembler
        self._msg = msg
        self._uris = {}

        for uri, msgDict, key in uris:
            self._uris[uri] = (msgDict, key)

        self._added = datetime.now()

    @property
    def msg(self):
        """ Get the represented message. """
        if self._uris:
            raise AssemblerError('Message still contains missing references.')

        return self._msg

    def older(self, timestamp):
        """ Returns True if this incomplete message is older the the given
            timestamp.
        """
        return self._added < timestamp

    def addBinary(self, uri, binaryData):
        """ Add the binary data with the given uri.

            @return:    True if the binary was used; False otherwise.
        """
        ref = self._uris.pop(uri, None)

        if ref:
            parent, key = ref
            parent[key] = binaryData

            if self._uris:
                self._added = datetime.now()
            else:
                self._assembler.forwardCompleteMessage(self)

            return True
        else:
            return False


class MessageAssembler(object):
    """ Class which is used to store incomplete messages for a certain time
        and which is used to assemble them when possible.
    """
    def __init__(self, protocol, timeout):
        """ Initialize the binary assembler.

            @param protocol:    Protocol instance for which this assembler is
                                used.

            @param timeout:     Timeout in seconds after which incomplete
                                messages are discarded.
            @type  timeout:     int
        """
        self._protocol = protocol
        self._timeout = timeout

        # Set of _IncompleteMessage instances
        self._incompleteMsgs = set()

        # Dictionary with binary UID as key and the binary as value
        self._binaries = {}

        # Setup repeated calling of the clean up method
        self._cleaner = LoopingCall(self._cleanUp)

    def forwardCompleteMessage(self, msgRepr):
        """ Callback for rce.comm.assembler._IncompleteMsg to send a completed
            message to the correct handler.
        """
        self._incompleteMsgs.remove(msgRepr)
        self._protocol.processCompleteMessage(msgRepr.msg)

    def _handleString(self, msg, uris):
        """ Try to process the received incomplete string message, i.e.
            assemble the message with the waiting binary data. Forward the
            message if it can be completed and store the incomplete message
            otherwise.

            @param msg:     Received string message.
            @type  msg:     str

            @param uris:    Return value of self._recursiveURISearch
            @type  uris:    [ (str, dict, str) or (str, list, int) ]
        """
        missing = []

        for ref in uris:
            uri, parent, key = ref
            binaryData = self._binaries.pop(uri, None)

            if binaryData:
                parent[key] = binaryData[0]
            else:
                missing.append(ref)

        if missing:
            self._incompleteMsgs.add(_IncompleteMsg(self, msg, missing))
        else:
            self._protocol.processCompleteMessage(msg)

    def _handleBinary(self, msg):
        """ Try to process a received binary message, i.e. assemble the waiting
            string message with the received binary data. Forward the message
            if the received binary data completes the waiting message and store
            the binary message otherwise.

            @param msg:     Received binary message.
            @type  msg:     str
        """
        uri = msg[:32]
        binaryData = StringIO()
        binaryData.write(msg[32:])

        for msg in self._incompleteMsgs:
            if msg.addBinary(uri, binaryData):
                break
        else:
            self._binaries[uri] = (binaryData, datetime.now())

    def _recursiveURISearch(self, multidict):
        """ Internally used method to find binary data in incoming messages.

            @return:    List of tuples of the forms (uri, dict, key) or
                        (uri, list, index)
        """
        valueList = []
        keys = []

        for k, v in multidict.iteritems():
            if isinstance(v, dict):
                valueList += self._recursiveURISearch(v)
            elif k[-1] == '*':
                keys.append(k)

        for k in keys:
            ele = multidict.pop(k)

            if isinstance(ele, list):
                lst = [None] * len(ele)
                multidict[k[:-1]] = lst

                for i, uri in enumerate(ele):
                    valueList.append((uri, lst, i))
            else:
                valueList.append((ele, multidict, k[:-1]))

        return valueList

    def processMessage(self, msg, binary):
        """ This method is used to process any messages which should pass
            through the assembler.
        """
        if binary:
            self._handleBinary(msg)
        else:
            try:
                msg = json.loads(msg)
            except ValueError:
                raise InvalidRequest('Message is not in valid JSON format.')

            uris = self._recursiveURISearch(msg)

            if uris:
                self._handleString(msg, uris)
            else:
                self._protocol.processCompleteMessage(msg)

    def start(self):
        """ Start the cleaner of the assembler.
        """
        self._cleaner.start(self._timeout / 4)

    def stop(self):
        """ Stop the cleaner of the assembler and remove any circular
            references.
        """
        self._incompleteMsgs = set()

        if self._cleaner.running:
            self._cleaner.stop()

    def _cleanUp(self):
        """ Internally used method to remove old incomplete messages.
        """
        limit = datetime.now() - timedelta(seconds=self._timeout)

        toClean = [msg for msg in self._incompleteMsgs if msg.older(limit)]

        if toClean:
            for msg in toClean:
                self._incompleteMsgs.remove(msg)

            log.msg('{0} incomplete messages have been dropped '
                    'from assembler.'.format(len(toClean)))

        toClean = [uri for uri, (_, timestamp) in self._binaries.iteritems()
                   if timestamp < limit]

        if toClean:
            for uri in toClean:
                del self._binaries[uri]

            log.msg('{0} unused binaries have been dropped '
                    'from assembler.'.format(len(toClean)))
