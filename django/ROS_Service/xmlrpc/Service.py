#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       Service.py
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
#       Based on:
#           Patchless XMLRPC Service for Django
#           Kind of hacky, and stolen from Crast on irc.freenode.net:#django
#           Self documents as well, so if you call it from outside of an
#           XML-RPC Client it tells you about itself and its methods
#           
#           Brendan W. McAdams <brendan.mcadams@thewintergrp.com>
#
#           SimpleXMLRPCDispatcher lets us register xml-rpc calls w/o
#           running a full XMLRPC Server.  It's up to us to dispatch data
#
#           src:  https://code.djangoproject.com/wiki/XML-RPC


# Decorator imports
from functools import wraps

from SimpleXMLRPCServer import SimpleXMLRPCDispatcher
from django.http import HttpResponse
from piston.handler import AnonymousBaseHandler

# Import used to setup ROS environment
import setupROS
# Do not import packages which need a functioning ROS environment before here

# Communication interface to the ROS adminstration
import DjangoInterface

# Create a Dispatcher; this handles the calls and translates info to function maps
dispatcher = SimpleXMLRPCDispatcher(allow_none=True, encoding=None)

# register introspection functions
dispatcher.register_introspection_functions()

# you have to manually register all functions that are xml-rpc-able with the dispatcher
# the dispatcher then maps the args down.
# The first argument is the actual method, the second is what to call it from the XML-RPC side...
dispatcher.register_function(DjangoInterface.readService, 'getAvailableServices')
dispatcher.register_function(DjangoInterface.createEnvironment, 'changeEnv')
dispatcher.register_function(DjangoInterface.readEnvironment, 'getEnv')
dispatcher.register_function(DjangoInterface.deleteEnvironment, 'removeEnv')
dispatcher.register_function(DjangoInterface.createTask, 'addTask')
dispatcher.register_function(DjangoInterface.readTask, 'getTask')
dispatcher.register_function(DjangoInterface.deleteTask, 'removeTask')

class XMLRPCHandler(AnonymousBaseHandler):
    """ This class provides the methodes to handle requests to the xmlrpc
        service API.
    """
    allowed_methods = ('GET', 'POST')
    
    def create(self, request):
        """ Handle 'POST' request.
            This type of request should be used to use the xmlrpc interface.
        """
        response = HttpResponse(mimetype="application/xml")
        response.write(dispatcher._marshaled_dispatch(request.raw_post_data))
        response['Content-length'] = str(len(response.content))
        return response
    
    def read(self, request):
        """ Handle 'GET' request.
            This type of request should be used to gather information
            on the xmlrpc interface.
        """
        response = HttpResponse()
        response.write("<b>This is an XML-RPC Service.</b><br>")
        response.write("You need to invoke it using an XML-RPC Client!<br>")
        response.write("The following methods are available:<ul>")
        methods = dispatcher.system_listMethods()
        
        for method in methods:
            # this will tell the users what args are expected
            sig = dispatcher.system_methodSignature(method)

            # this just reads the docblock of the function
            help = dispatcher.system_methodHelp(method)

            response.write("<li><b>%s</b>: [%s] %s" % (method, sig, help))
        
        response.write("</ul>")
        response.write('<a href="http://www.djangoproject.com/"> <img src="http://media.djangoproject.com/img/badges/djangomade124x25_grey.gif" border="0" alt="Made with Django." title="Made with Django."></a>')
        
        response['Content-length'] = str(len(response.content))
        return response
