#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       FIFO.py
#       
#       This file is part of the RoboEarth Cloud Engine framework.
#       
#       This file was originally created for RoboEearth - http://www.roboearth.org/
#       The research leading to these results has received funding from the European Union 
#       Seventh Framework Programme FP7/2007-2013 under grant agreement no248942 RoboEarth.
#       
#       Copyright 2012 RoboEarth
#       
#       Licensed under the Apache License, Version 2.0 (the "License");
#       you may not use this file except in compliance with the License.
#       You may obtain a copy of the License at
#       
#       http://www.apache.org/licenses/LICENSE-2.0
#       
#       Unless required by applicable law or agreed to in writing, software
#       distributed under the License is distributed on an "AS IS" BASIS,
#       WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#       See the License for the specific language governing permissions and
#       limitations under the License.
#       
#       \author/s: 
#       
#       

# Python specific imports
from collections import deque

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
        return len(self._buf)
    
    def __iadd__(self, fifo):
        """ Built-in method to add another FIFO to this one.
        """
        self._buf += fifo._buf
        return self
    
    def push(self, data):
        """ Add data to the FIFO.
        """
        for c in data:
            self._buf.append(c)
    
    def pushFront(self, data):
        """ Add data to the front of the FIFO.
            (Should only be used for adding the header to the message!)
        """
        for c in reversed(data):
            self._buf.appendleft(c)
    
    def pop(self, n):
        """ Try to get n characters from FIFO.
                        
            @return:    Tuple with the returned string as first element and the
                        number of effectively returned characters.
            @rtype:     ( str, int )
        """
        lenBuf = len(self._buf)
        
        if lenBuf < n:
            n = lenBuf
        
        return (''.join(self._buf.popleft() for _ in xrange(n)), n)
    
    def write(self, data):
        """ Synonym for 'push'. Used for serialization of ROS messages.
        """
        self.push(data)
