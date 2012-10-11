#!/usr/bin/env python
# -*- coding: utf-8 -*-
#     
#     fifo.py
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
from datetime import datetime, timedelta
from collections import deque

# twisted specific imports
from twisted.python import log

# Custom imports
from util.interfaces import verifyObject
from comm import definition
from comm.interfaces import IRCEProducer


class MessageFIFO(object):
    """ FIFO which is used to store a message internally for processing.
    """
    def __init__(self):
        """ Initialize the MessageFIFO.
        """
        self._buf = deque()
    
    def __len__(self):
        """ Built-in method to get the length of the buffer.
        """
        return reduce(lambda s, buf: s+len(buf), self._buf, 0)
    
    def __iadd__(self, fifo):
        """ Built-in method to add another FIFO to this one.
        """
        if not isinstance(fifo, MessageFIFO):
            raise TypeError('Can only add a MessageFIFO.')
        
        self._buf += fifo._buf
        return self
    
    def push(self, data):
        """ Add data to the FIFO.
        """
        self._buf.append(data)
    
    def pushFront(self, data):
        """ Add data to the front of the FIFO.
            (Should only be used for adding the header to the message!)
        """
        self._buf.appendleft(data)
    
    def pop(self, n):
        """ Try to get n characters from FIFO.
                        
            @return:    Tuple with the returned string as first element and the
                        number of effectively returned characters.
            @rtype:     ( str, int )
        """
        data = []
        
        while self._buf:
            ele = self._buf.popleft()
            length = len(ele)
            
            if length >= n:
                data.append(ele[:n])
                self._buf.appendleft(ele[n:])
                break
            else:
                data.append(ele)
                n -= length
        
        data = ''.join(data)
        return (data, len(data))
    
    write = push
    

class ProducerFIFO(object):
    """ FIFO which is used for buffering the outgoing messages to a specific
        destination.
    """
    def __init__(self, dest):
        """ Initialize the Producer FIFO.

            @param dest:    The destination for which this producer FIFO is
                            used.
            @type  dest:    str
        """
        self._fifo = deque()
        self._dest = dest
    
    @property
    def dest(self):
        """ Communication ID for which this producer FIFO is used. """
        return self._dest
    
    def __len__(self):
        """ Get the number of producers in the FIFO.
        """
        return len(self._fifo)
    
    def add(self, producer):
        """ Add a new producer to the FIFO.
            
            @raise:     util.interfaces.InterfaceError
        """
        verifyObject(IRCEProducer, producer)
        self._fifo.append((producer, datetime.now()))
    
    def getTimeStamp(self):
        """ Get the timestamp of the first producer in the FIFO.
            
            @rtype:     datetime
        """
        return self._fifo[0][1]
    
    def pop(self):
        """ Get the first producer from the FIFO.
            
            @rtype:     comm.handler._MessageSender or
                        comm.handler._MessageForwarder
        """
        return self._fifo.popleft()[0]
    
    def clean(self):
        """ Remove producers which have exceeded the timeout.
        """
        i = 0
        limit = (datetime.now()
                 - timedelta(seconds=definition.MSG_QUEUE_TIMEOUT/2))
        
        while len(self._fifo):
            if self._fifo[0][1] < limit:
                self._fifo.popleft()
            else:
                break
        
        if i:
            log.msg('{0} Producer(s) has been dropped from queue '
                    'for destination "{1}".'.format(i, self._dest))
