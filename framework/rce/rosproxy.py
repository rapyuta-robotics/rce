#!/usr/bin/env python
# -*- coding: utf-8 -*-
#     
#     rosproxy.py
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

#python specific imports
from multiprocessing.managers import SyncManager
import sys, time
import httplib
import json

#twisted-specific imports
from twisted.cred.error import UnauthorizedLogin
from twisted.python import log
from twisted.web.resource import Resource
from twisted.web.server import NOT_DONE_YET, Site
from twisted.internet.defer import fail, succeed

#rosapi imports
from rospy import get_published_topics
from rosservice import get_service_list
from rosservice import get_service_type as rosservice_get_service_type
from rosservice import get_service_node as rosservice_get_service_node
from rosservice import get_service_uri
from rostopic import find_by_type
from ros import rosnode, rosgraph
from rosnode import get_node_names
from rosgraph.masterapi import Master
import rospy

#rce specific imports
from rce.error import InvalidRequest, InternalError, DeadConnection
from rce.master.console import ROSProxyClient

class ROSProxy():
    def get_services(self):
        """ Returns a list of all the services advertised in the ROS system """
        return get_service_list()

    def get_topics(self):
        """ Returns a list of all the topics being published in the ROS system """
        return [x[0] for x in get_published_topics()]

class ConsoleROSProxyAuthentication(Resource):
    
    isLeaf = True
    def __init__(self, Users):
        self._ros = ROSProxy()
        self._users = Users

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
        
        if not self._users.get(userID[0]) or self._users.get(userID[0]) != key[0]:
            return fail(UnauthorizedLogin("Unknown user or key"))
            
        for name, param in [('action', action), ('userID', userID),
                            ('key', key)]:
            if len(param) != 1:
                return fail(InvalidRequest("Parameter '{0}' has to be unique "
                                           'in request.'.format(name)))
        
        return self.parseInputLine(action)

    def parseInputLine(self, action):
        output = None
        if action is not None and action is not '':
            func = getattr(self, 'cmd_' + str(action[0]).upper(), None)
            if func is not None:
                output = func()
            else:
                return fail(InvalidRequest("No such action"))
        return succeed(output)
                
    def cmd_SERVICES(self):
        return self._ros.get_services()
        
    def cmd_TOPICS(self):
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

    shared_dict = ROSProxyClient('', 5000, 'dummypassword')

    reactor.listenTCP(rosproxyPort, Site(ConsoleROSProxyAuthentication(shared_dict.get_dict()))) #HTTP Server
    
    reactor.run(installSignalHandlers=False)
    
    f.close()
