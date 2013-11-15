#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     rce-util/rce/util/loader.py
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
#     This file is based on the module roslib.launcher of the ROS library:
#
#         Licensed under the Software License Agreement (BSD License)
#
#         Copyright (c) 2008, Willow Garage, Inc.
#         All rights reserved.
#
#         Redistribution and use in source and binary forms, with or without
#         modification, are permitted provided that the following conditions
#         are met:
#
#          * Redistributions of source code must retain the above copyright
#            notice, this list of conditions and the following disclaimer.
#          * Redistributions in binary form must reproduce the above
#            copyright notice, this list of conditions and the following
#            disclaimer in the documentation and/or other materials provided
#            with the distribution.
#          * Neither the name of Willow Garage, Inc. nor the names of its
#            contributors may be used to endorse or promote products derived
#            from this software without specific prior written permission.
#
#         THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#         "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#         LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#         FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#         COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#         INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#         BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#         LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#         CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#         LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#         ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#         POSSIBILITY OF SUCH DAMAGE.
#
#

# Python specific imports
import os
import sys

# ROS specific imports
try:
    import rospkg
    import roslib.packages
except ImportError:
    print('Can not import ROS Python libraries.')
    print('Make sure they are installed and the ROS Environment is setup.')
    exit(1)

class ResourceNotFound(Exception):
    """ Exception is raised by the Loader when a resource can not be found.
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

        # List of all packages which are already added to sys.path
        self._packages = set()

        # Key:    tuple (package name, clsType, cls)
        # Value:  msg/srv module
        self._moduleCache = {}

    def _getDepends(self, pkg):
        """ roslib.launcher

            Get all package dependencies which are not catkin-ized.

            @param pkg:     Name of the package for which the dependencies
                            should be returned.
            @type  pkg:     str

            @return:        List of all non catkin-ized dependencies.
            @rtype:         [str]
        """
        vals = self._rp.get_depends(pkg, implicit=True)
        return [v for v in vals if not self._rp.get_manifest(v).is_catkin]

    def _appendPackagePaths(self, manifest, paths, pkgDir):
        """ roslib.launcher

            Add paths for package to paths.

            @param manifest:    Package manifest
            @type  manifest:    Manifest

            @param paths:       List of paths to append to
            @type  paths:       [str]

            @param pkgDir:      Package's filesystem directory path
            @type  pkgDir:      str
        """
        exports = manifest.get_export('python', 'path')

        if exports:
            paths.extend(e.replace('${prefix}', pkgDir)
                             for export in exports
                                 for e in export.split(':'))
        else:
            dirs = [os.path.join(pkgDir, d) for d in ['src', 'lib']]
            paths.extend(d for d in dirs if os.path.isdir(d))

    def _generatePythonPath(self, pkg):
        """ roslib.launcher

            Recursive subroutine for building dependency list and Python path.

            @param pkg:     Name of package for which Python paths should be
                            generated.
            @type  pkg:     str
        """
        if pkg in self._packages:
            return []

        # short-circuit if this is a catkin-ized package
        m = self._rp.get_manifest(pkg)

        if m.is_catkin:
            self._packages.add(pkg)
            return []

        packages = self._getDepends(pkg)
        packages.append(pkg)

        paths = []

        try:
            for p in packages:
                m = self._rp.get_manifest(p)
                d = self._rp.get_path(p)
                self._appendPackagePaths(m, paths, d)
                self._packages.add(p)
        except:
            self._packages.discard(pkg)
            raise

        return paths

    def _loadManifest(self, pkg):
        """ roslib.launcher

            Update the Python sys.path with package's dependencies.

            @param pkg:     Name of the package for which the manifest should
                            be loaded.
            @type  pkg:     str
        """
        if pkg in self._packages:
            return

        sys.path = self._generatePythonPath(pkg) + sys.path

    def _checkPermission(self, module):
        """ Internally used method to check if there might me a candidate for
            the module for which we have insufficient permissions.

            @return:        True if there is a directory which we can not
                            access and which has the correct module name.
                            False if there is no candidate with insufficient
                            permissions.
            @rtype:         bool
        """
        permission = []

        for p in sys.path:
            path = os.path.join(p, module[0])

            if os.path.isdir(path):
                if not os.access(path, os.R_OK | os.X_OK):
                    permission.append(True)
                elif (len(module) > 1 and
                      any(os.access(os.path.join(path, init), os.F_OK)
                          for init in ['__init__.py', '__init__.pyc'])):
                    permission.append(self._checkPermission(module[1:]))

        return bool(permission and all(permission))

    def _loadModule(self, pkg, clsType, cls):
        """ Internally used method to load a module.
        """
        try:
            self._loadManifest(pkg)
        except rospkg.ResourceNotFound:
            raise ResourceNotFound('Can not load manifest for ROS package '
                                   '"{0}".'.format(pkg))

        try:
            return __import__('.'.join([pkg, clsType]), fromlist=[cls])
        except ImportError as e:
            if self._checkPermission([pkg, clsType]):
                raise ResourceNotFound('Can not import {0}.{1} of ROS package '
                                       '{2}: There is a module candidate for '
                                       'whose directory I have insufficient '
                                       'permissions.')

            raise ResourceNotFound('Can not import {0}.{1} of ROS package '
                                   '{2}: {1}'.format(clsType, cls, pkg, e))

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

            @raise:         ValueError, rce.util.loader.ResourceNotFound
        """
        if isinstance(pkg, unicode):
            try:
                pkg = str(pkg)
            except UnicodeEncodeError:
                raise ValueError('The package "{0}" is not valid.'.format(pkg))

        if isinstance(cls, unicode):
            try:
                cls = str(cls)
            except UnicodeEncodeError:
                raise ValueError('The class "{0}" is not valid.'.format(cls))

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

            @raise:         ValueError, rce.util.loader.ResourceNotFound
        """
        if isinstance(pkg, unicode):
            try:
                pkg = str(pkg)
            except UnicodeEncodeError:
                raise ValueError('The package "{0}" is not valid.'.format(pkg))

        if isinstance(cls, unicode):
            try:
                cls = str(cls)
            except UnicodeEncodeError:
                raise ValueError('The class "{0}" is not valid.'.format(cls))

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

            @raise:         rce.util.loader.ResourceNotFound
        """
        try:
            return self._rp.get_path(pkg)
        except rospkg.ResourceNotFound:
            raise ResourceNotFound('Can not find ROS package '
                                   '"{0}".'.format(pkg))

    def findNode(self, pkg, exe):
        """ Find the node/executable in the given package.

            @param pkg:     Package name in which the node should be localized.
            @type  pkg:     str

            @param exe:     Name of the node/executable which should be
                            localized.
            @type  exe:     str

            @return:        Path to the executable in package.
            @rtype:         str

            @raise:         rce.util.loader.ResourceNotFound
        """
        try:
            return roslib.packages.find_node(pkg, exe, rospack=self._rp)[0]
        except rospkg.ResourceNotFound:
            raise ResourceNotFound('Can not find ROS package '
                                   '"{0}".'.format(pkg))
        except IndexError:
            raise ResourceNotFound('Can not find executable "{0}" in '
                                   'ROS package "{1}".'.format(exe, pkg))
