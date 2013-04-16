#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     rce-core/rce/core/base.py
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

# twisted specific imports
from twisted.python import log
from twisted.python.failure import Failure
from twisted.internet.defer import Deferred, succeed, fail
from twisted.spread.pb import RemoteReference, Referenceable, \
    DeadReferenceError, PBConnectionLost

# rce specific imports
from rce.util.error import AlreadyDead


class Status(Referenceable):
    """ Status observer which the objects in the slave processes can used to
        report status changes back to the Master.
    """
    def __init__(self, proxy):
        """ Initialize the status observer.

            @param proxy:       Proxy which should be informed of status
                                changes.
            @type  proxy:       rce.core.base.Proxy
        """
        self._proxy = proxy
        proxy._registerStatus(self)

    def remote_died(self):
        """ Method which can be used by the slave objects to report that they
            have died and should be removed from the Master.
        """
        if self._proxy:
            self._proxy.destroy()

    def cancel(self):
        """ Method which is used by the Proxy to inform the status observer
            that he does no longer want any information about status changes.
        """
        self._proxy = None


class Proxy(object):
    """ The Proxy should be used to represent an object from a slave process in
        the Master.
        It provides the same methods as the twisted.spread.pb.RemoteReference.
        Additionally, the Proxy is callable to get a Deferred which fires as
        soon as the RemoteReference or a Failure is present.
    """
    def __init__(self, *args, **kw):
        """ Initialize the Proxy.
        """
        super(Proxy, self).__init__(*args, **kw)

        # Stores the remote reference
        self.__obj = None
        self.__failure = None

        # Status object that informs if remote reference died
        self.__status = None

        self.__cbs = set()
        self.__pending = []

    def _registerStatus(self, status):
        """ Method should only be used by a Status instance to register itself
            with the Proxy.
        """
        assert self.__status is None
        self.__status = status

    def callRemote(self, _name, *args, **kw):
        """ Make a call to the RemoteReference and return the result as a
            Deferred. It exists to allow queuing of calls to remote reference
            before the remote reference has arrived.

            For more information refer to twisted.spread.pb.RemoteReference.

            @param _name:       Name of the method which should be called.
                                The prefix 'remote_' will be added to the name
                                in the remote object to select the method which
                                should be called.
            @type  _name:       str

            @param *args:       Positional arguments which will be passed to
                                the remote method.

            @param **kw:        Keyworded arguments which will be passed to the
                                remote method.

            @return:            Deferred which will fire with the result of the
                                call or a Failure if there was a problem.
            @rtype:             twisted.internet.defer.Deferred
        """
        if self.__failure is not None:
            d = fail(self.__failure)
        elif self.__pending is not None:
            d = Deferred()
            self.__pending.append(d)
        else:
            d = succeed(self.__obj)

        d.addCallback(lambda ref: ref.callRemote(_name, *args, **kw))
        d.addErrback(self.__filter, _name)
        return d

    def callback(self, obj):
        """ Register the remote reference which provides the necessary methods
            for this Proxy. Fires all pending callbacks passing on the remote
            reference as a parameter.

            Exactly one call can be made to either 'callback' or 'errback'!

            @param obj:         Remote reference which should be registered.
            @type  obj:         twisted.spread.pb.RemoteReference
        """
        assert self.__obj is None, 'Only one object can be registered.'
        assert isinstance(obj, RemoteReference)

        # Store the remote reference
        self.__obj = obj

        # inform when the remote reference is disconnected using __disconnected
        obj.notifyOnDisconnect(self.__disconnected)

        # Call all remaining remote calls made before the remote reference
        # arrived
        for pending in self.__pending:
            pending.callback(obj)

        self.__pending = None

    def errback(self, f):
        """ Register a failure object which was received during the creation
            of the object in the remote process.

            Exactly one call can be made to either 'callback' or 'errback'!

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
            @rtype:             twisted.internet.defer.Deferred
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

    def __filter(self, failure, name):
        """ Internally used method which is used as an errback to check the
            failure for errors indicating that the Proxy is dead.
        """
        if failure.check(DeadReferenceError, PBConnectionLost):
            self.__notify(failure)
        else:
            print('Received the following error message when calling {0} from '
                  'class {1}: {2}'.format(name, self.__class__.__name__,
                                          failure.getErrorMessage()))

        return failure

    def __notify(self, failure):
        """ Method is used as a callback to inform the Proxy that a failure
            occurred.
        """
        # if the Proxy already stores a failure then do nothing
        if self.__failure:
            return

        # Disconnect callback for disconnect
        if self.__obj:
            self.__obj.dontNotifyOnDisconnect(self.__disconnected)

        # Disconnect status
        if self.__status:
            self.__status.cancel()

        # Mark that the Proxy is a failure and doesn't store remote reference.
        self.__failure = failure
        self.__status = None

        # fire errbacks on all remote calls
        if self.__pending is not None:
            for pending in self.__pending:
                pending.errback(failure)

            self.__pending = None

        # fire all callbacks to notify of the proxy's death
        for cb in self.__cbs:
            cb(self)

        self.__cbs = None

    def __destroy(self):
        """ Method to destroy Proxy. Takes care of whether it's been called from
            the remote side or the local side. Calls __notify to clear up
            pending callbacks of callRemote or notifyOnDeath. Calls
            remote reference's (self.__obj) destroy function to make sure object
            on the remote side is also dead
        """
        m = 'Referenced object {0} dead.'.format(self.__class__.__name__)
        self.__notify(Failure(DeadReferenceError(m)))

        # Destroy object on the remote side. Takes care if it's already
        # destroyed.
        if self.__obj:
            def eb(failure):
                from twisted.spread.pb import PBConnectionLost #@Reimport
                if not failure.check(PBConnectionLost):
                    log.err(failure)

            try:
                self.__obj.callRemote('destroy').addErrback(eb)
            except DeadReferenceError, PBConnectionLost:
                pass

        self.__obj = None

    def __disconnected(self, _):
        self.__notify(Failure(DeadReferenceError('Broker is disconnected.')))
        self.__obj = None
