#!/usr/bin/env python
# -*- coding: utf-8 -*-
#     
#     loader.py
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
import os

# ROS specific imports
import rospkg

# twisted specific imports
from twisted.python import log

# Custom imports
from errors import InternalError


class ResourceNotFound(Exception):
    """ Error is raised by the Loader when a resource can not be found.
    """


class Loader(object):
    """ The Loader should be used to dynamically find and load message and
        service classes. Additionally, the Loader can be used to locate
        nodes/executables in packages.
        To increase the speed the Loader has a cache for the classes and the
        paths to the nodes.
    """
    def __init__(self, rosPath=None):
        """ Initialize the Loader.
            
            @param rosPath:     Ordered list of paths to search for resources.
                                If None (default), use environment ROS path.
            @type  rosPath:     [str] / None
        """
        self._rp = rospkg.RosPack(rosPath)
        
        # Key:    tuple (package name, clsType, cls)
        # Value:  msg/srv module
        self._moduleCache = {}
        
        # Key:    package name
        # Value:  node paths dictionary
        #                Keys:   node name
        #                Value:  absolute path to node
        self._nodeCache = {}
    
    def _loadModule(self, pkg, clsType, cls):
        """ Internally used method to load a module.
        """
        if isinstance(pkg, unicode):
            try:
                pkg = str(pkg)
            except UnicodeEncodeError:
                raise InternalError('The package "{0}" is not valid.'.format(
                                                                          pkg))
        
        if isinstance(cls, unicode):
            try:
                cls = str(cls)
            except UnicodeEncodeError:
                raise InternalError('The class "{0}" is not valid.'.format(
                                                                        cls))
        
        try:
            return __import__('.'.join([pkg, clsType]), fromlist=[cls])
        except ImportError:
            pass
        
        try:
            pkgDir = self._rp.get_path(pkg)
        except rospkg.ResourceNotFound:
            raise ResourceNotFound('Can not find ROS package "{0}".'.format(
                                                                         pkg))
        
        try:
            import imp
            return imp.load_source(cls, os.path.join(
                        pkgDir, 'src', pkg, clsType, '_{0}.py'.format(cls)))
        except ImportError as e:
            raise ResourceNotFound('Can not import ROS package '
                                   '"{0}": {1}'.format(pkg, e))
    
    def loadMsg(self, pkg, cls):
        """ Get the message class matching the string pair.
            This method uses a internal cache; therefore, changes on the
            filesystem will be ignored once the class is loaded into the cache.
            
            @param pkg:     Package name from where the class should be loaded.
            @type  pkg:     str
            
            @param cls:     Class name of the message class which should be
                            loaded.
            @type  cls:     str
            
            @return:        Class matching the string pair.
            @rtype:         subclass of genpy.message.Message
            
            @raise:         core.util.ResourceNotFound
        """
        key = (pkg, 'msg', cls)
        
        try:
            module = self._moduleCache[key]
        except KeyError:
            module = self._loadModule(*key)
            self._moduleCache[key] = module
        
        try:
            return getattr(module, cls)
        except AttributeError:
            raise ResourceNotFound('ROS package "{0}" does not have '
                                   'message class "{1}"'.format(pkg, cls))
    
    def loadSrv(self, pkg, cls):
        """ Get the service class matching the string pair.
            This method uses a internal cache; therefore, changes on the
            filesystem will be ignored once the class is loaded into the cache.
            
            @param pkg:     Package name from where the class should be loaded.
            @type  pkg:     str
            
            @param cls:     Class name of the message class which should be
                            loaded.
            @type  cls:     str
            
            @return:        Class matching the string pair.
            @rtype:         ROS service class
            
            @raise:         core.util.ResourceNotFound
        """
        key = (pkg, 'srv', cls)
        
        try:
            module = self._moduleCache[key]
        except KeyError:
            module = self._loadModule(*key)
            self._moduleCache[key] = module
        
        try:
            return getattr(module, cls)
        except AttributeError:
            raise ResourceNotFound('ROS package "{0}" does not have '
                                   'service class "{1}"'.format(pkg, cls))
    
    def findPkgPath(self, pkg):
        """ Find the path to the given package.
            
            @param pkg:     Package name in for which the path should be
                            returned.
            @type  pkg:     str
            
            @return:        Path to the package.
            @rtype:         str
            
            @raise:         core.util.ResourceNotFound
        """
        try:
            return self._rp.get_path(pkg)
        except rospkg.ResourceNotFound:
            raise ResourceNotFound('Can not find ROS package '
                                   '"{0}".'.format(pkg))
    
    def _findNodes(self, pkg):
        """ Internally used method to search for all nodes in the package.
        """
        nodes = {}
        
        for path, dirs, files in os.walk(self.findPkgPath(pkg)):
            for f in files:
                p = os.path.join(path, f)
                
                if os.access(p, os.X_OK):
                    if f in nodes:
                        log.msg('Found multiple executables with the name '
                                '"{0}" in ROS package "{1}". Only the first '
                                'match will be kept'.format(f, pkg))
                    else:
                        nodes[f] = p
            
            for d in dirs[:]:
                if d[0] == '.':
                    dirs.remove(d)
                elif d  in ['build', 'rospack_nosubdirs']:
                    dirs.remove(d)
        
        return nodes
    
    def findNode(self, pkg, exe):
        """ Find the node/executable in the given package.
            
            @param pkg:     Package name in which the node should be localized.
            @type  pkg:     str
            
            @param exe:     Name of the node/executable which should be
                            localized.
            @type  exe:     str
            
            @return:        Path to the executable in package.
            @rtype:         str
            
            @raise:         core.util.ResourceNotFound
        """
        try:
            nodeDict = self._nodeCache[pkg]
        except KeyError:
            nodeDict = self._findNodes(pkg)
            self._nodeCache[pkg] = nodeDict
        
        try:
            return nodeDict[exe]
        except KeyError:
            raise ResourceNotFound('Can not find executable "{0}" in '
                                   'ROS package "{0}".'.format(exe, pkg))
