#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       FIFO.py
#       
#       This file is part of the RoboEarth Cloud Engine framework.
#       
#       This file was originally created for RoboEearth
#       http://www.roboearth.org/
#       
#       The research leading to these results has received funding from
#       the European Union Seventh Framework Programme FP7/2007-2013 under
#       grant agreement no248942 RoboEarth.
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
#       \author/s: Dominique Hunziker 
#       
#       

# zope specific imports
from zope.interface.verify import verifyObject
from zope.interface.exceptions import Invalid

# twisted specific imports
from twisted.python import log

# Python specific imports
from datetime import datetime, timedelta
from collections import deque

# Custom imports
import settings
from Exceptions import InternalError
from Message.Interfaces import IRCEProducer

class ProducerFIFO(object):
    """ FIFO which is used for buffering the outgoing messages to a specific destination.
    """
    def __init__(self, dest):
        """ Initialize the Producer FIFO.

            @param dest:    The destination for which this producer FIFO is used.
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
        """
        try:
            verifyObject(IRCEProducer, producer)
        except Invalid as e:
            raise InternalError(
                'Verification of the producer class "{0}" for the Interface "IRCEProducer" failed: {1}'.format(
                    producer.__class__.__name__,
                    e
                )
            )
        
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
        return self._fifo.popleft()[0]
    
    def clean(self):
        """ Remove producers which have exceeded the timeout.
        """
        i = 0
        limit = datetime.now() - timedelta(seconds=settings.MSG_QUQUE_TIMEOUT)
        
        while len(self._fifo):
            if self._fifo[0][1] < limit:
                self._fifo.popleft()
            else:
                break
        
        if i:
            log.msg('{0} Producer(s) has been dropped from queue for destination "{1}".'.format(i, self._dest))
