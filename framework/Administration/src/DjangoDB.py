#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       djangoDB.py
#       
#       Copyright 2012 dominique hunziker <dominique.hunziker@gmail.com>
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

import settings as rosSettings

import sys
from django.conf import settings

settings.configure(
    DATABASE_ENGINE = rosSettings.DJANGO_DB.get('DATABASE_ENGINE', ''),
    DATABASE_NAME = rosSettings.DJANGO_DB.get('DATABASE_NAME', ''),
    DATABASE_USER = rosSettings.DJANGO_DB.get('DATABASE_USER', ''),
    DATABASE_PASSWORD = rosSettings.DJANGO_DB.get('DATABASE_PASSWORD', ''),
    DATABASE_HOST = rosSettings.DJANGO_DB.get('DATABASE_HOST', ''),
    DATABASE_PORT = rosSettings.DJANGO_DB.get('DATABASE_PORT', ''),
    TIME_ZONE = rosSettings.DJANGO_DB.get('TIME_ZONE', ''),
)

if rosSettings.DJANGO_ROOT_DIR not in sys.path:
    sys.path.append(rosSettings.DJANGO_ROOT_DIR)

from reappengine.models import Package, Node, Param, Interface

from MessageUtility import InvalidRequest

class DjangoDBError(Exception):
    """ This error is raised if an error occurred in conjunction with
        the django database.
    """
    pass

def _getPackage(pkg):
    """
    """
    package = Package.objects.filter(name=pkg)
    
    if len(package) != 1:
        if package:
            raise DjangoDBError('Package could not uniquely identified.')
        else:
            raise DjangoDBError('Package could not be found.')
    
    return package[0]

def _getNode(package, node):
    """
    """
    node = Node.objects.filter(name=node, pkg=package)
    
    if len(node) != 1:
        if package:
            raise DjangoDBError('Node could not uniquely identified.')
        else:
            raise DjangoDBError('Node could not be found.')
    
    return node[0]

def _getParam(node):
    """
    """
    return Param.objects.filter(node=node)

def _getInterface(package, interface):
    """
    """
    interface = Interface.objects.filter(name=interface, pkg=package)
    
    if len(interface) != 1:
        if package:
            raise DjangoDBError('Interface could not uniquely identified.')
        else:
            raise DjangoDBError('Interface could not be found.')
    
    return interface[0]

def _split(name):
    """
    """
    parts = name.split('/')
    
    if len(parts) != 2:
        raise InvalidRequest('Key "{0}" is not valid.'.format(name))
    
    return tuple(parts)

def isValidNodeName(key):
    """
    """
    try:
        pkg, node = _split(key)
    except InvalidRequest:
        return False
    
    try:
        _getNode(_getPackage(pkg), node)
    except DjangoDBError:
        return False
    
    return True

def getNodeDef(key):
    """
    """
    pkg, exe = _split(key)
    
    package = _getPackage(pkg)
    node = _getNode(package, exe)
    params = _getParam(node)
    
    config = [(param.name, param.paramType, param.opt, param.default) for param in params]
    
    return (package.name, node.name, config)

def getServiceDef(key):
    """
    """
    pkg, srv = _split(key)
    
    package = _getPackage(pkg)
    service = _getInterface(package, srv)
    
    if service.msgType != 'srv':
        raise DjangoDBError('{0} is not a service.'.format(srv))
    
    srvName = service.name
    srvCls = '{0}/{1}'.format(package.name, service.msgDef)
    reqCls = '{0}Request'.format(srvCls)
    
    return (srvName, srvCls, reqCls)
