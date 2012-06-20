#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       Base.py
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

# twisted specific imports
from twisted.internet.defer import Deferred

# Python specific imports
from threading import Thread, Event
from functools import wraps
from Queue import Queue, Empty

class CentralizedLoop(Thread):
    """ This class provides an endless loop which operates with a queue where tasks
        can be added which should be executed in the loop. Be aware that an iteration
        through the loop is not interrupted, when the CentralizedLoop should be
        terminated, but has to wait for the loop iteration to complete.

        To use this class create a subclass and decorate the methods which
        should be run in the endless loop with the decorator 'job'.
    """

    def __init__(self, reactor):
        """ Initialize the CentralizedLoop. Make sure this method is called
            when overwriting the constructor in the subclass.
        """
        super(CentralizedLoop, self).__init__()
        
        self._terminate = Event()
        
        self._reactor = reactor
        self._tiggerID = reactor.addSystemEventTrigger('before', 'shutdown', self._terminate.set)
        
        self._queue = Queue()
    
    def __del__(self):
        """ Destructor which is used to remove the SystemEventTrigger from the reactor if
            the instance is destroyed before the event is triggered.
        """
        self._reactor.removeSystemEventTrigger(self._tiggerID)
    
    def run(self):
        """ Mainloop of CentralizedLoop.
        """
        self.init()

        try:
            while not self._terminate.isSet():
                try:
                    try:
                        (deferred, func, args, kw) = self._queue.get(block=True, timeout=0.5)
                    except Empty:
                        continue
                    
                    deferred.callback(func(*args, **kw))
                except Exception as e:
                    deferred.errback(e)
        finally:
            self.close()

    def init(self):
        """ Overwrite this method if some initialization is required before
            entering the loop.
        """
        pass

    def close(self):
        """ Overwrite this method if some clean up is required in after leaving
            the loop.
        """
        pass

    @staticmethod
    def job(func):
        """ This decorator should be used on the methods which are called from the
            outside and which should be executed in the CentralizedLoop.
        """
        @wraps(func)
        def wrapper(*args, **kw):
            self = args[0]

            if not isinstance(self, CentralizedLoop):
                raise ValueError('First argument is not self.')

            deferred = Deferred()
            self._queue.put((deferred, func, args, kw))
            return deferred

        return wrapper
