#!/usr/bin/env python
# -*- coding: utf-8 -*-
#     
#     assembler.py
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

try:
    from cStringIO import StringIO
except ImportError:
    from StringIO import StringIO

# twisted specific imports
from twisted.python import log
from twisted.internet.task import LoopingCall


class MessageAssembler(object):
    """ Class which is used to store incomplete messages for a certain time
        and which is used to assemble them when possible.
    """
    class _IncompleteMsg(object):
        """ Internally used class which represents an incomplete class.
        """
        def __init__(self, assembler, msg, uris):
            """ Initialize the incomplete message.
                
                @param assembler:   Assembler to which the this instance
                                    belongs.
                @type  assembler:   client.assembler.MessageAssembler
                
                @param msg:     Incomplete message as a dictionary.
                @type  msg:     dict
                
                @param uris:    Return value of _recursiveURISearch
                @type  nr:      [ (str, dict, str) ]
            """
            self._assembler = assembler
            self._msg = msg
            self._uris = {}
            
            for uri, msgDict, key in uris:
                self._uris[uri] = (msgDict, key)
            
            self._added = datetime.now()
        
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
                msgDict, key = ref
                
                del msgDict[key]
                msgDict[key[:-1]] = binaryData
                
                if self._uris:
                    self._added = datetime.now()
                else:
                    self._assembler._removeIncompleteMsg(self)
                    self._assembler._processCompleteMessage(self._msg)
                
                return True
            else:
                return False
    
    def __init__(self, protocol, timeout):
        """ Initialize the binary assembler.
            
            @param protocol:    Protocol instance for which this assembler is
                                used.
            
            @param timeout:     Timeout in seconds after which incomplete
                                message parts are removed.
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
    
    def _removeIncompleteMsg(self, msg):
        """ Callback for client.assembler.MessageAssembler._IncompleteMsg to
            remove itself from the set of incomplete messages.
        """
        self._incompleteMsgs.discard(msg)
    
    def _processCompleteMessage(self, msg):
        """ Callback for client.assembler.MessageAssembler._IncompleteMsg to
            send a completed message to the correct handler.
        """
        self._protocol.processCompleteMessage(msg)
    
    def _handleString(self, msg, uris):
        """ Try to process the received incomplete string message, i.e.
            assemble the message with the waiting binary data. Forward the
            message if it can be completed and store the incomplete message
            otherwise.
            
            @param msg:     Received string message.
            @type  msg:     str
            
            @param uris:    Return value of _recursiveURISearch
            @type  nr:      [ (str, dict, str) ]
        """
        missing = []
        
        for ref in uris:
            uri, msgDict, key = ref
            binaryData, _ = self._binaries.pop(uri, None)
            
            if binaryData:
                del msgDict[key]
                msgDict[key[:-1]] = binaryData
            else:
                missing.append(ref)
        
        if missing:
            self._incompleteMsgs.add(
                MessageAssembler._IncompleteMsg(self._protocol, msg, missing))
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
            
            @return:    List of tuples of the form (uri, dict, key)
        """
        valueList = []
        
        for k, v in multidict.iteritems():
            if isinstance(v, dict):
                valueList += self._recursiveURISearch(v)
            elif k[-1] == '*':
                valueList.append((v, multidict, k))
        
        return valueList
    
    def processMessage(self, msg, binary):
        """ This method is used to process any messages which should pass
            through the assembler.
        """
        if binary:
            self._handleBinary(msg)
        else:
            msg = json.loads(msg)
            uris = self._recursiveURISearch(msg)
            
            if uris:
                self._handleString(msg, uris)
            else:
                self._protocol.processCompleteMessage(msg)
    
    def start(self):
        """ Start the cleaner of the assembler.
        """
        self._cleaner.start(self._timeout/4)
    
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
        limit = datetime.now()-timedelta(seconds=self._timeout)
        
        toClean = [msg for msg in self._incompleteMsgs if msg.older(limit)]
        
        if toClean:
            for msg in toClean:
                self._incompleteMsgs.remove(msg)
            
            log.msg('{0} Incomplete messages have been dropped '
                    'from assembler.'.format(len(toClean)))
        
        toClean = [uri for uri, (_, timestamp) in self._binaries.iteritems()
                   if timestamp < limit]
        
        if toClean:
            for uri in toClean:
                del self._binaries[uri]
            
            log.msg('{0} unused binaries have been dropped '
                    'from assembler.'.format(len(toClean)))
