#!/usr/bin/env python
# -*- coding: utf-8 -*-
#     
#     path.py
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
import re


# ASCII char followed by (alphanumeric, _)
_NAME_RE = re.compile('^[A-Za-z][\w_]*$') 


def isLegalName(name):
    """ Check if the name is legal, i.e. if it starts with an ASCII char
        followed by alphanumeric chars or '_'.
        
        @param name:    Name which should be checked.
        @type  name:    str
        
        @return:        True if the name is legal.
        @rtype:         bool
    """
    if not name:
        return False
    
    m = _NAME_RE.match(name)
    return m is not None and m.group(0) == name


def checkName(name):
    """ Check if the name is legal, i.e. if it starts with an ASCII char
        followed by alphanumeric chars or '_'.
        
        @param name:    Name which should be checked.
        @type  name:    str
        
        @raise:         ValueError if the name is not legal.
    """
    if not isLegalName(name):
        raise ValueError("'{0}' is not a legal name.".format(name))


def checkPath(path, description):
    """ Check if the path is valid.
        
        @param path:            Path which should be checked.
        @type  path:            str
        
        @param description:     Description which is used for the error
                                message if necessary.
        @type  description:     str
        
        @raise:                 ValueError, if path is not valid.
    """
    if not os.path.isabs(path):
        raise ValueError('{0} directory is not an absolute '
                         'path.'.format(description))
    
    if not os.path.isdir(path):
        raise ValueError('{0} directory does not exist: '
                         '{1}'.format(description, path))


def checkExe(folder, exe):
    """ Check if the executable is valid.
        
        @param folder:          Folder in which the executable is located.
        @type  folder:          str
        
        @param exe:             Executable which should be checked.
        @type  exe:             str
        
        @raise:                 ValueError, if executable is not valid.
    """
    path = os.path.join(folder, exe)
    
    if not os.path.isfile(path):
        raise ValueError("'{0}' is not a file.".format(exe))
    
    if not os.access(path, os.X_OK):
        raise ValueError("'{0}' is not a executable.".format(exe))


def processPackagePaths(paths):
    """ Utility function to process the attribute ROOT_PKG_DIR from settings.
        
        @param paths:       List of attribute ROOT_PKG_DIR from settings.py
        @type  paths:       [(str, str/None)]
        
        @return:            Processed list; each element of ROOT_PKG_DIR will
                            be transformed into a tuple of source path and
                            destination path which can than be used to
                            extend the fstab file for the container creation.
        @rtype:             [(str, str)]
    """
    pkgDir = []
    usedNames = set()
    
    for path, name in paths:
        checkPath(path, 'ROS Package')
        
        if not name:
            name = os.path.basename(path)
        
        checkName(name)
        
        if name in usedNames:
            raise ValueError("Package name '{0}' is not "
                             'unique.'.format(name))
        
        usedNames.add(name)
        pkgDir.append((path, os.path.join('opt/rce/packages', name)))
    
    return pkgDir
