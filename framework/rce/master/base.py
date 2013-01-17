#!/usr/bin/env python
# -*- coding: utf-8 -*-
#     
#     base.py
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
from weakref import WeakSet

# twisted specific imports
from twisted.python.failure import Failure
from twisted.internet.defer import Deferred, succeed, fail
from twisted.spread.pb import RemoteReference, \
    DeadReferenceError, PBConnectionLost


class AlreadyDead(Exception):
    """ Exception is raised when a death notifier callback is registered with
        an already dead proxy.
    """


class Proxy(object):
    """ 
    """
    
    def __init__(self, *args, **kw):
        """ Initialize the Proxy.
        """
        super(Proxy, self).__init__(*args, **kw)
        
        self.__obj = None
        self.__failure = None
        self.__refs = WeakSet()
        
        self.__cbs = set()
        self.__pending = []
    
    def callRemote(self, _name, _returnType, *args, **kw):
        """
        """
        if self.__failure is not None:
            d = fail(self.__failure)
        elif self.__pending is not None:
            d = Deferred()
            self.__pending.append(d)
        else:
            d = succeed(self.__obj)
        
        d.addCallback(self.__call, _name, (args, kw))
        d.addErrback(self.__filter)
        
        if issubclass(_returnType, Deferred):
            cb = lambda result, deferred: deferred.callback(result)
            eb = lambda failure, deferred: deferred.callback(failure)
        elif issubclass(_returnType, Proxy):
            cb = lambda result, proxy: proxy.registerRemoteReference(result)
            eb = lambda failure, proxy: proxy.registerFailureObject(failure)
        else:
            raise TypeError('Unknown return type: {0}'.format(_returnType))
        
        proxy = _returnType(self)
        d.addCallbacks(cb, eb, callbackArgs=(proxy,), errbackArgs=(proxy,))
        return proxy
    
    def registerRemoteReference(self, obj):
        """ Register the remote reference which provides the necessary methods
            for this Proxy.
            
            Only one of the method 'registerRemoteReference' or
            'registerFailureObject' can be called once!
            
            @param obj:         Remote reference which should be registered.
            @type  obj:         twisted.spread.pb.RemoteReference
        """
        assert self.__obj is None, 'Only one object can be registered.'
        assert isinstance(obj, RemoteReference)
        
        self.__obj = obj
        obj.notifyOnDisconnect(self.__disconnected)
        
        for pending in self.__pending:
            pending.callback(obj)
        
        self.__pending = None
    
    def registerFailureObject(self, f):
        """ Register a failure object which was received during the creation
            of the object in the remote process.
            
            Only one of the method 'registerRemoteReference' or
            'registerFailureObject' can be called once!
            
            @param f:           Failure object which should be registered.
            @type  f:           twisted.python.failure.Failure
        """
        assert self.__obj is None, 'Only one object can be registered.'
        assert isinstance(f, Failure), "Failure has to be of type 'Failure'."
        self.__notify(f)
    
    def notifyOnDeath(self, cb):
        """ Register a callback which will be called once the remote object is
            dead, i.e. no longer connected to this process or a failure was
            received.
            
            @param cb:          Callback which should be registered. It should
                                take this instance as only argument.
            @type  cb:          callable
        """
        assert callable(cb)
        
        try:
            self.__cbs.add(cb)
        except AttributeError:
            raise AlreadyDead('{0} is already '
                              'dead.'.format(self.__class__.__name__))
    
    def dontNotifyOnDeath(self, cb):
        """ Unregister a callback which would have been called once the remote
            object is dead.
            
            @param cb:          Callback which should be unregistered.
            @type  cb:          callable
        """
        try:
            self.__cbs.remove(cb)
        except AttributeError:
            pass
    
    def __call__(self):
        """ Return a reference to the remote object as soon as the reference
            is available.
            
            @return:            Reference to the RemoteObject instance.
                                (type: rce.master.base.RemoteReference)
            @rtype:             twisted::Deferred
        """
        if self.__failure is not None:
            return fail(self.__failure)
        
        if self.__pending is not None:
            d = Deferred()
            self.__pending.append(d)
            return d
        
        return succeed(self.__obj)
    
    def destroy(self):
        """ Method should be called to destroy the Proxy as well as the remote
            object.
        """
        self.__destroy()
    
    def __call(self, ref, name, (args, kw)):
        """ Internally used method which is used as a callback to call the
            method of the referenced remote object.
        """
        return ref.callRemote(name, *args, **kw)
    
    def __filter(self, failure):
        """ Internally used method which is used as an errback to check the
            failure for errors indicating that the Proxy is dead.
        """
        if failure.check(DeadReferenceError, PBConnectionLost):
            self.__notify(failure)
        else:
            print('Received the following error message when calling {0} from '
                  'class {1}: {2}'.format(self.__name__,
                                          self.__class__.__name__,
                                          failure.getErrorMessage()))
        
        return failure
    
    def __notify(self, failure):
        """ Method is used as a callback to inform the proxy that a failure
            occurred.
        """
        if self.__failure:
            return
        
        if self.__obj:
            self.__obj.dontNotifyOnDisconnect(self.__disconnected)
        
        self.__failure = failure
        self.__obj = ()
        
        if self.__pending is not None:
            for pending in self.__pending:
                pending.errback(failure)
            
            self.__pending = None
        
        for cb in self.__cbs:
            cb(self)
        
        self.__cbs = None
    
    def __destroy(self):
        self.__notify(Failure(DeadReferenceError('Reference is deleted.')))
    
    def __disconnected(self, _):
        self.__notify(Failure(DeadReferenceError('Reference is dead.')))
    
    def __del__(self):
        self.__destroy()
