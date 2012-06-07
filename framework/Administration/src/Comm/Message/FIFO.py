#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       FIFO.py
#       
#       Copyright 2012 dominique hunziker <dominique.hunziker@gmail.com>
#       
#       This program is free software; you can redistribute it and/or modify
#       it under the terms of the GNU General Public License as published by
#       the Free Software Foundation; either version 2 of the License, or
#       (at your option) any later version.
#       
#       This program is distributed in the hope that it will be useful,
#       but WITHOUT ANY WARRANTY; without even the implied warranty of
#       MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#       GNU General Public License for more details.
#       
#       You should have received a copy of the GNU General Public License
#       along with this program; if not, write to the Free Software
#       Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#       MA 02110-1301, USA.
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
