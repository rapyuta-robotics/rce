#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       DjangoDB.py
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

# django specific imports
import django.conf

# Python specific imports
import sys

# Custom imports
import settings
from Exceptions import InvalidRequest
from Base import CentralizedLoop

# Setup django DB connection
django.conf.settings.configure(
    DATABASE_ENGINE=settings.DJANGO_DB.get('DATABASE_ENGINE', ''),
    DATABASE_NAME=settings.DJANGO_DB.get('DATABASE_NAME', ''),
    DATABASE_USER=settings.DJANGO_DB.get('DATABASE_USER', ''),
    DATABASE_PASSWORD=settings.DJANGO_DB.get('DATABASE_PASSWORD', ''),
    DATABASE_HOST=settings.DJANGO_DB.get('DATABASE_HOST', ''),
    DATABASE_PORT=settings.DJANGO_DB.get('DATABASE_PORT', ''),
    TIME_ZONE=settings.DJANGO_DB.get('TIME_ZONE', ''),
)

if settings.DJANGO_ROOT_DIR not in sys.path:
    sys.path.append(settings.DJANGO_ROOT_DIR)

# Import Models used in django DB
from reappengine.models import Package, Node, Param, Interface

class DjangoDBError(Exception):
    """ This error is raised if an error occurred in conjunction with
        the django database.
    """
    pass

class Django(CentralizedLoop):
    """ This class provides the necessary methods to access the django database.
    """
    def _getPackage(self, pkg):
        """ Internal method to get the specified Package model instance.
    
            @param pkg:     Package name
            @type  pkg:     str
    
            @raise:     DjangoDBError if no or more than one matching model
                        is found.
        """
        package = Package.objects.filter(name=pkg) #@UndefinedVariable
    
        if len(package) != 1:
            if package:
                raise DjangoDBError('Package could not uniquely identified.')
            else:
                raise DjangoDBError('Package could not be found.')
    
        return package[0]
    
    def _getNode(self, package, node):
        """ Internal function to get the specified Node model instance located
            in the specified package.
    
            @param package:     Package in which the node is located
            @type  package:     Package model instance
    
            @param node:    Node name
            @type  node:    str
    
            @raise:     DjangoDBError if no or more than one matching model
                        is found.
        """
        node = Node.objects.filter(name=node, pkg=package) #@UndefinedVariable
    
        if len(node) != 1:
            if package:
                raise DjangoDBError('Node could not uniquely identified.')
            else:
                raise DjangoDBError('Node could not be found.')
    
        return node[0]
    
    def _getParam(self, node):
        """ Internal function to get a list of all the Param model instances
            for the given node.
            
            @param node:    Node for which the parameters should be searched
            @type  node:    Node
        """
        return Param.objects.filter(node=node) #@UndefinedVariable
    
    def _getInterface(self, node):
        """ Internal function to get a list of all the Interface model instances
            for the given node.
            
            @param node:    Node for which the interfaces should be searched
            @type  node:    Node
        """
        return Interface.objects.filter(node=node) #@UndefinedVariable
    
    def _split(self, name):
        """ Internal function to split and check a key.
        """
        parts = name.split('/')
    
        if len(parts) != 2:
            raise InvalidRequest('Key {0} is not valid.'.format(name))
    
        return tuple(parts)
    
    # TODO: Necessary ???
    @CentralizedLoop.job
    def existsNode(self, key):
        """ Check if the given key is valid, i.e. if there is a matching
            entry in the database.
    
            @param key:     Key which should be checked
            @type  key:     str
    
            @return:    True if the key is valid; False otherwise
        """
        try:
            pkg, node = self._split(key)
        except InvalidRequest:
            return False
    
        try:
            self._getNode(self._getPackage(pkg), node)
        except DjangoDBError:
            return False
    
        return True
    
    @CentralizedLoop.job
    def getNode(self, key):
        """ Get the node definition for the given key.
    
            @param key:     Key for which the node definition should be retrieved.
            @type  key:     str
    
            @return:    Node instance.
            @rtype:     Node
    
            @raise:     DjangoDBError
        """
        pkg, exe = self._split(key)
    
        pkg = self._getPackage(pkg)
        node = self._getNode(pkg, exe)
        params = self._getParam(node)
        #interfaces = self._getInterface(node)
        
        params = [(param.name, param.paramType, param.opt, param.default) for param in params]
        #interfaces = [(interface.msgType, '{0}/{1}'.format(pkg.name, interface.msgDef), interface.name) for interface in interfaces]
        return (pkg.name, node.name, params) #, interfaces)
