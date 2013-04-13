#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     rce-core/rce/rosproxy.py
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
#     Copyright 2013 RoboEarth
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
#     \author/s: Mayank Singh
#
#

# Python specific imports
import httplib
import json
import fcntl

# ROS specific imports
import rospy
from rosservice import get_service_list
# TODO: Unused imports
#from rosservice import get_service_type as rosservice_get_service_type
#from rosservice import get_service_node as rosservice_get_service_node
#from rosservice import get_service_uri
#from rostopic import find_by_type
#from rosnode import get_node_names
#from rosgraph.masterapi import Master

get_published_topics = rospy.get_published_topics

# twisted specific imports
from twisted.cred.error import UnauthorizedLogin
from twisted.python import log
from twisted.web.resource import Resource
from twisted.web.server import NOT_DONE_YET, Site
from twisted.internet.defer import fail, succeed

# rce specific imports
from rce.util.error import InternalError


class InvalidRequest(Exception):
    """ Exception is raised in case the HTTP request could not be processed.
    """


class ROSProxy(object):
    def get_services(self):
        """ Returns a list of all the services advertised in the ROS system
        """
        return get_service_list()

    def get_topics(self):
        """ Returns a list of all the topics being published in the ROS system
        """
        return [x[0] for x in get_published_topics()]


class ConsoleROSProxyAuthentication(Resource):
    """ Authenticator and Request handler for the ROS Proxy Web Server.
    """
    isLeaf = True

    def __init__(self):
        self._ros = ROSProxy()
        self._dbFile = "/opt/rce/data/rosenvbridge.db"

    def _checkDB(self, userID, key):
        """ Method to check the rosproxy database to authenticate a web
            request.

            @param userID:    Username
            @type userID:     string

            @param key:       Secret key
            @type key:        string
        """
        # TODO: Why not return True directly instead all lines will be read
        # TODO: Should this be deferred to a separate thread due to flock,
        #       which is a blocking call?
        found = False
        with open(self._dbFile, 'r') as bridgefile:
            fcntl.flock(bridgefile.fileno(), fcntl.LOCK_EX)
            lines = bridgefile.readlines()
            for line in lines:
                g = line.split(':')
                if g[0] == userID and str(g[1].rstrip()) == str(key):
                    found = True
        return found

    def _processGETReq(self, args):
        """ Internally used method to process a GET request.
        """
        try:
            action = args['action']
            userID = args['userID']
            key = args['key']
        except KeyError as e:
            return fail(InvalidRequest('Request is missing parameter: '
                                       '{0}'.format(e)))

        if not self._checkDB(userID[0], key[0]):
            return fail(UnauthorizedLogin("Unknown user or key"))

        for name, param in [('action', action), ('userID', userID),
                            ('key', key)]:
            if len(param) != 1:
                return fail(InvalidRequest("Parameter '{0}' has to be unique "
                                           'in request.'.format(name)))

        return self.parseInputLine(action)

    def parseInputLine(self, action):
        """ Function to route various command requests.
            @param action:    The command to be executed.
            @type action:     list
        """
        output = None
        if action is not None and action is not '':
            func = getattr(self, 'cmd_' + str(action[0]).upper(), None)
            if func is not None:
                output = func()
            else:
                return fail(InvalidRequest("No such action"))
        return succeed(output)

    def cmd_SERVICES(self):
        """ Handler for services call.
        """
        return self._ros.get_services()

    def cmd_TOPICS(self):
        """ Handler for topics call.
        """
        return self._ros.get_topics()

    def _processGETResp(self, output, request):
        """ Internally used method to process a response to a GET request from
            the realm.
        """
        msg = {'key' : output}

        self._render_GET(request, httplib.OK,
                         'application/json; charset=utf-8', json.dumps(msg))

    def _processGETErr(self, e, request):
        """ Internally used method to process an error to a GET request from
            the realm.
        """
        if e.check(InvalidRequest):
            msg = e.getErrorMessage()
            code = httplib.BAD_REQUEST
        elif e.check(UnauthorizedLogin):
            msg = e.getErrorMessage()
            code = httplib.UNAUTHORIZED
        elif e.check(InternalError):
            e.printTraceback()
            msg = 'Internal Error'
            code = httplib.INTERNAL_SERVER_ERROR
        else:
            e.printTraceback()
            msg = 'Fatal Error'
            code = httplib.INTERNAL_SERVER_ERROR

        self._render_GET(request, code, 'text/plain; charset=utf-8', msg)

    def _render_GET(self, request, code, ctype, msg):
        """ Internally used method to render the response to a GET request.
        """
        request.setResponseCode(code)
        request.setHeader('content-type', ctype)
        request.write(msg)
        request.finish()

    def render_GET(self, request):
        """ This method is called by the twisted framework when a GET request
            was received.
        """
        d = self._processGETReq(request.args)
        d.addCallback(self._processGETResp, request)
        d.addErrback(self._processGETErr, request)

        return NOT_DONE_YET


def main(reactor, rosproxyPort):
    f = open('/opt/rce/data/rosproxy.log', 'w')
    log.startLogging(f)

    def terminate():
        reactor.callFromThread(reactor.stop)

    rospy.on_shutdown(terminate)

    #HTTP Server
    reactor.listenTCP(rosproxyPort, Site(ConsoleROSProxyAuthentication()))

    reactor.run(installSignalHandlers=False)

    f.close()
