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
from types import MethodType
from weakref import WeakSet

# twisted specific imports
from twisted.python.failure import Failure
from twisted.internet.defer import Deferred, succeed, fail
from twisted.spread.pb import DeadReferenceError, PBConnectionLost


class AlreadyDead(Exception):
    """ Exception is raised when a death notifier callback is registered with
        an already dead proxy.
    """


class _DeferredMethod(object):
    __slots__ = ['method']
    
    def __init__(self, method):
        self.method = method


class _ProxyMethod(object):
    __slots__ = ['method', 'cls']
    
    def __init__(self, method, cls):
        self.method = method
        self.cls = cls


class _Destructor(object):
    __slots__ = ['method']
    
    def __init__(self, method):
        self.method = method


class _RedirectedMethodBase(object):
    """
    """
    def __init__(self, name, doc):
        self.__name__ = name
        self.__doc__ = doc
    
    def _makeProxy(self, instance):
        raise NotImplementedError()
    
    def _call(self, ref, (args, kw)):
        return getattr(ref, self.__name__)(*args, **kw)
    
    def _filter(self, failure, instance):
        if failure.check(DeadReferenceError, PBConnectionLost):
            instance.notify(failure)
        else:
            print('Received the following error message when calling {0} from '
                  'class {1}: {2}'.format(self.__name__,
                                          instance.__class__.__name__,
                                          failure.getErrorMessage()))
        
        return failure
    
    def _callback(self, result, proxy):
        raise NotImplementedError()
    
    def _errback(self, failure, proxy):
        raise NotImplementedError()
    
    def __call__(self, instance, *args, **kw):
        proxy = self._makeProxy(instance)
        d = instance()
        d.addCallback(self._call, (args, kw))
        d.addErrback(self._filter, instance)
        d.addCallbacks(self._callback, self._errback,
                       callbackArgs=(proxy,), errbackArgs=(proxy,))
        return proxy
    
    def __get__(self, instance, owner):
        return MethodType(self, instance, owner)


class _RedirectedMethodDeferred(_RedirectedMethodBase):
    """
    """
    def _makeProxy(self, instance):
        return Deferred()
    
    def _callback(self, result, proxy):
        proxy.callback(result)
    
    def _errback(self, failure, proxy):
        proxy.errback(failure)


class _RedirectedMethodProxy(_RedirectedMethodBase):
    """
    """
    def __init__(self, name, doc, proxyCls):
        super(_RedirectedMethodProxy, self).__init__(name, doc)
        self._proxyCls = proxyCls
    
    def _makeProxy(self, instance):
        return self._proxyCls(instance)
    
    def _callback(self, result, proxy):
        proxy.createRemoteObject(result)
    
    def _errback(self, failure, proxy):
        proxy.registerFailureObject(failure)


class RemoteObject(object):
    """
    """
    def __init__(self, obj, handler):
        """
        """
        # print('class {0} - object {1}'.format(self.__class__.__name__, obj))
        self.obj = obj
        handler.registerDeferredObject(self)
    
    def notifyOnDisconnect(self, cb):
        """
        """
        self.obj.notifyOnDisconnect(cb)
    
    def dontNotifyOnDisconnect(self, cb):
        """
        """
        self.obj.dontNotifyOnDisconnect(cb)


class ProxyMeta(type):
    """
    """
    def __new__(cls, name, bases, dict): #@ReservedAssignment
        rmtDict = {}
        
        for attrName, attr in dict.iteritems():
            if isinstance(attr, _DeferredMethod):
                rmtDict[attrName] = attr.method
                dict[attrName] = _RedirectedMethodDeferred(attrName,
                                                           attr.method.__doc__)
            elif isinstance(attr, _ProxyMethod):
                rmtDict[attrName] = attr.method
                dict[attrName] = _RedirectedMethodProxy(attrName,
                                                        attr.method.__doc__,
                                                        attr.cls)
            elif isinstance(attr, _Destructor):
                if '__del__' in rmtDict:
                    raise TypeError("Class '{0}' can have only one "
                                    'destructor.'.format(name))
                
                attr.method.__name__ = '__del__'
                rmtDict['__del__'] = attr.method
        
        rmtBases = tuple(getattr(base, '_REMOTE_CLS') for base in bases
                         if hasattr(base, '_REMOTE_CLS')) or (RemoteObject,)
        dict['_REMOTE_CLS'] = type('Remote{0}'.format(name), rmtBases, rmtDict)
        
        return type.__new__(cls, name, bases, dict)


class Proxy(object):
    """
    """
    __metaclass__ = ProxyMeta
    
    @staticmethod
    def returnDeferred(method):
        """
        """
        return _DeferredMethod(method)
    
    @staticmethod
    def returnProxy(cls):
        """
        """
        def decorator(method):
            return _ProxyMethod(method, cls)
        return decorator
    
    @staticmethod
    def destroyProxy(method):
        """
        """
        return _Destructor(method)
    
    def __init__(self, *args, **kw):
        """
        """
        super(Proxy, self).__init__(*args, **kw)
        
        self.__obj = None
        self.__failure = None
        self.__refs = WeakSet()
        
        self.__cbs = set()
        self.__pending = []
    
    def createRemoteObject(self, obj):
        """
        """
        return self._REMOTE_CLS(obj, self)
    
    def registerDeferredObject(self, obj):
        """
        """
        assert self.__obj is None, 'Only one object can be registered.'
        assert not isinstance(obj, Failure)
        
        self.__obj = obj
        
        if isinstance(obj, RemoteObject):
            obj.notifyOnDisconnect(self.__disconnected)
        
        for pending in self.__pending:
            pending.callback(obj)
        
        self.__pending = None
    
    def registerFailureObject(self, f):
        """
        """
        assert self.__obj is None, 'Only one object can be registered.'
        assert isinstance(f, Failure), "Failure has to be of type 'Failure'."
        self.notify(f)
    
    def notifyOnDeath(self, cb):
        """
        """
        assert callable(cb)
        
        try:
            self.__cbs.add(cb)
        except AttributeError:
            raise AlreadyDead('{0} is already '
                              'dead.'.format(self.__class__.__name__))
    
    def dontNotifyOnDeath(self, cb):
        """
        """
        try:
            self.__cbs.remove(cb)
        except AttributeError:
            pass
    
    def __call__(self):
        """
        """
        if self.__failure is not None:
            return fail(self.__failure)
        
        if self.__pending is not None:
            d = Deferred()
            self.__pending.append(d)
            return d
        
        return succeed(self.__obj)
    
    def notify(self, failure):
        if self.__failure:
            return
        
        if self.__obj:
            if isinstance(self.__obj, RemoteObject):
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
    
    def destroy(self):
        """
        """
        self.__destroy()
    
    def __destroy(self):
        self.notify(Failure(DeadReferenceError('Reference is deleted.')))
    
    def __disconnected(self, _):
        self.notify(Failure(DeadReferenceError('Reference is dead.')))
    
    def __del__(self):
        self.__destroy()
