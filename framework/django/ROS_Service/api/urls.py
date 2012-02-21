#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       urls.py
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

from django.conf.urls.defaults import patterns, include, url
from piston.resource import Resource
import ROS_Service.api.Service

class CsrfExemptResource(Resource):
    """ A Custom Resource that is csrf exempt.
    """
    def __init__(self, handler, authentication=None):
        super(CsrfExemptResource, self).__init__(handler, authentication)
        self.csrf_exempt = getattr(self.handler, 'csrf_exempt', True)

ServiceHandler = CsrfExemptResource(ROS_Service.api.Service.ServiceHandler)
EnvironmentHandler = CsrfExemptResource(ROS_Service.api.Service.EnvironmentHandler)
TaskHandler = CsrfExemptResource(ROS_Service.api.Service.TaskHandler)

urlpatterns = patterns('',
    url(r'^ros_service$', ServiceHandler),
    url(r'^ros_service/(?P<envID>[a-zA-Z]{10})$', EnvironmentHandler),
    url(r'^ros_service/(?P<envID>[a-zA-Z]{10})/(?P<taskID>[a-zA-Z]{10})$', TaskHandler),
    url(r'^ros_service/(?P<envID>[a-zA-Z]{10})/(?P<taskID>[a-zA-Z]{10})/(?P<ref>[^/]+)$', TaskHandler),
)
