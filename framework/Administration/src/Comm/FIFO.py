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

# twisted specific imports
from zope.interface.verify import verifyObject
from twisted.python import log

# Python specific imports
from datetime import datetime, timedelta

# Custom imports
import settings
from Exceptions import InternalError
from Message.Interfaces import IReappengineProducer

class ProducerFIFO(object):
    """ FIFO which is used for buffering the outgoing messages to a specific destination.
    """
    def __init__(self, dest):
        """ Initialize the Producer FIFO.

            @param dest:    The destination for which this producer queue is used.
            @type  dest:    str
        """
        self._fifo = []
        self.dest = dest
    
    def __len__(self):
        """ Get the number of producers in the FIFO.
        """
        return len(self._fifo)
    
    def add(self, producer):
        """ Add a new producer to the FIFO.
        """
        if not verifyObject(IReappengineProducer, producer):
            raise InternalError('Tried to add a producer which does not implement the IReappengineProducer interface.')
        
        self._fifo.append((producer, datetime.now()))
    
    def getTimeStamp(self):
        """ Get the timestamp of the first producer in the FIFO.
            
            @rtype:     datetime
        """
        return self._fifo[0][1]
    
    def pop(self):
        """ Get the first producer from the FIFO.
            
            @rtype:     MessageSender or MessageForwarder
        """
        return self._fifo.pop(0)[0]
    
    def clean(self):
        """ Remove producers which have exceeded the timeout.
        """
        limit = datetime.now() - timedelta(seconds=settings.MSG_QUQUE_TIMEOUT)
        
        for i in xrange(len(self._fifo)):
            if self._fifo[i][1] > limit:
                break
        
        if i:
            self._fifo = self._fifo[i:]
            log.msg('{0} Producer(s) has been dropped from queue for destination "{1}".'.format(i, self.dest))
