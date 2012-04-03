#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       ThreadUtility.py
#       
#       Copyright 2011 dominique hunziker <dominique.hunziker@gmail.com>
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

import threading
import Queue
from functools import wraps

class ThreadManagerError(Exception):
    """ This error is raised if an error occurred in conjunction with the
        ThreadManager.
    """
    pass

class ThreadManager(object):
    """ Little helper class to simplify thread managment.
    """
    class Iter(object):
        """ Little helper class to iterate over all threads in ThreadManager.
        """
        def __init__(self, threadManager):
            """ Initalizes the iterator.
            """
            self.threadManager = threadManager
            self.prio = 2
            self.pos = 0
            
            with self.threadManager.lock:
                self.threadManager.updated = False
            
        def next(self):
            """ Get the next element.
            """
            with self.threadManager.lock:
                if self.threadManager.updated:
                    raise IndexError
                
                if self.pos >= len(self.threadManager.data[self.prio]):
                    self.prio -= 1
                    
                    if self.prio < 0:
                        raise StopIteration
                
                thread = self.threadManager.data[self.prio][self.pos]
                self.pos += 1
                return thread
    
    def __init__(self):
        """ Creates a new empty list.
        """
        self.lock = threading.Lock()
        self.data = [[],[],[]]
        self.updated = True
    
    def append(self, element, priority=1):
        """ Method to add an element to the list. The optional argument
            priority allows to control the termination process as a lower
            number means that the thread is terminated after the higher
            numbered threads have terminated. Valid priority levels are
            0,1,2.
        """
        if priority not in [0,1,2]:
            raise IndexError
        
        with self.lock:
            self.updated = True
            self.data[priority].append(element)
        
        element.registerThreadMngr(self)
    
    def terminateThread(self, thread):
        """ Method to terminate a single thread from the manager.
        """
        termThread = None
        
        if thread in self:
            termThread = thread
        
        if termThread:
            termThread.terminate.set()
            termThread.join()
            
            # Remark: The thread should remove himself from the manger;
            # make sure that this is the case
            if thread in self:
                raise ThreadManagerError('Could not terminate thread.')
        else:
            # Thread not managed by this manager (inform somebody?)
            pass
    
    def terminateAll(self, priorityLevel=[2,1,0]):
        """ Method to terminate all threads. If the argument
            priorityLevel is given only the corresponding threads are
            terminated. It should be a list.
        """
        for priority in [2,1,0]:
            if priority not in priorityLevel:
                continue
            
            with self.lock:
                threads = list(self.data[priority])
            
            for thread in threads:
                thread.terminate.set()
            
            for thread in threads:
                thread.join()
            
            # Remark: The threads should remove themselves from the manager;
            # make sure that this is the case
            if self.data[priority]:
                raise ThreadManagerError('Could not terminate threads.')
    
    def removeThread(self, thread):
        """ Callback function for thread to remove itself from the
            ThreadManager.
        """
        with self.lock:
            for priority in [2,1,0]:
                if thread in self.data[priority]:
                    self.data[priority].remove(thread)
                    break
            else:
                # Thread not managed by this manager (inform somebody?)
                pass
    
    def __iter__(self):
        return ThreadManager.Iter(self)

class LoopError(Exception):
    """ Use this error if an error occurred in the loop which should
        stop the current iteration, but not the loop.
    """
    pass

class ManagedThread(threading.Thread):
    """ This class is used to provide threads which are compatible with
        ThreadManager.
    """
    def __init__(self):
        """ Initalize the ManagedThread. Make sure this method is called
            when overwriting the constructor in the subclass.
        """
        threading.Thread.__init__(self)
        self.managedlock = threading.Lock()
        self.threadMngr = None
        self.terminate = threading.Event()
    
    def run(self):
        """ The method which has to be overwitten from the Thread base
            class.
        """
        try:
            self.threadMethod()
        except:
            import traceback
            traceback.print_exc()
        finally:
            with self.managedlock:
                if self.threadMngr:
                    self.threadMngr.removeThread(self)
    
    def threadMethod(self):
        """ This method has to be overwritten in the subclass. Else a
            NotImplementedError is raised.
        """
        raise NotImplementedError('The method threadMethod is not implemented for the class {0}'.format(self.__class__.__name__))
    
    def registerThreadMngr(self, threadMngr):
        """ Internal method to register the used threadMngr for this
            thread.
        """
        with self.managedlock:
            self.threadMngr = threadMngr

class Task(ManagedThread):
    """ This class is used as Task thread. Be aware that the given
        function is not interrupted, when the Task should be terminated,
        but the ThreadManager has to wait for the function to return.
    """
    def __init__(self, func, *args, **kw):
        """ Initialize the Task. As soon the Task is started the given
            function is executed with the positional arguments args and
            keyworded arguments kw.
        """
        super(Task, self).__init__()
        self.func = func
        self.args = args
        self.kw = kw
    
    def threadMethod(self):
        """ The method which has to be overwitten from the ManagedThread
            base class.
        """
        self.func(*self.args, **self.kw)

class QueueWorker(ManagedThread):
    """ This class is used to provide threads which operate with an
        endless loop. Be aware that an iteration through the loop is not
        interrupted, when the QueueWorker should be terminated, but the 
        ThreadManager has to wait for the loop iteration to complete.
        
        To use this class create a subclass and decorate the methods which
        should be run in the QueueWorker thread with the decorator 'job'.
    """
    class Capsule(object):
        """ This class is used to provide a thread-safe exchange of the
            queue workers results. If the returned data is None an error
            occurred.
        """
        def __init__(self):
            """ Initializes the Capsule.
            """
            self._event = threading.Event()
            self._data = None
        
        def getData(self):
            """ Use this method to get the data.
            """
            self._event.wait(3)     # Block the execution for maximal 3 secs
            return self._data
        
        def setData(self, data):
            """ Use this method to set the data.
            """
            self._data = data
            self._event.set()
        
        def delData(self):
            """ Use this method to delete the data.
            """
            del self._data
        
        data = property(getData, setData, delData, """ Data of the container. """)
    
    def __init__(self):
        """ Initalize the QueueWorker. Make sure this method is called
            when overwriting the constructor in the subclass.
        """
        super(QueueWorker, self).__init__()
        
        self.queue = Queue.Queue()
    
    def threadMethod(self):
        """ The method which has to be overwitten from the ManagedThread
            base class.
        """
        self.init()
        
        try:
            while not self.terminate.isSet():
                try:
                    query = self.queue.get(block=True, timeout=1)
                except Queue.Empty:
                    continue
                
                query[0](query[1], query[2], query[3])
        finally:
            self.stop()
    
    def init(self):
        """ Overwrite this method if some initalization is required in
            the thread.
        """
        pass
    
    def stop(self):
        """ Overwrite this method if some clean up is required in the
            thread.
        """
        pass
    
    @staticmethod
    def job(f):
        """ This decorator should be used on the methods which are called
            from the outside and which should be executed in the QueueWorker
            loop.
        """
        @wraps(f)
        def wrapper(*args, **kw):
            def loopFunc(capsule_, args_, kw_):
                try:
                    capsule_.data = f(*args_, **kw_)
                except Exception as e:
                    # What to do with the errors in the loop ?
                    capsule_.data = None
            
            capsule = QueueWorker.Capsule()
            args[0].queue.put((loopFunc, capsule, args, kw))
            
            return capsule.data
        
        return wrapper
