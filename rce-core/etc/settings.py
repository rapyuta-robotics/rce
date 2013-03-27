#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     settings.py
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
import socket
import fcntl
import struct
import re
import urllib2

# Custom imports
from rce.util.name import validateName, IllegalName


##################################################
###                                            ###
###                 RCE settings               ###
###                                            ###
##################################################

# Custom JSON/ROS messages converters
CONVERTER_CLASSES = (
    'rce.util.converters.image.ImageConverter',
)

# Compression level used for communication
GZIP_LVL = 9

##################################################
###                                            ###
###           Communication settings           ###
###                                            ###
##################################################

# Port which should be used for the HTTP connections to robots from the Master
HTTP_PORT = 9000

# Port which should be used for the websocket connections to the robots
WS_PORT = 9010

# Port which should be used for the communication with the Master
MASTER_PORT = 8080

# Port which should be used for the internal communication
RCE_INTERNAL_PORT = 10030

#Port for connection between cloud engine and console client
RCE_CONSOLE_PORT = 8081

#Port for connection between rosproxy and console client
ROS_PROXY_PORT = 9020

##################################################
###                                            ###
###   Deployment specific interface settings   ###
###                                            ###
##################################################

# Platform used to deploy the cloud engine
# Currently supported :
#   DEFAULT_LOCAL
#   RACKSPACE
#   AWS
#   CUSTOM (ALter the Network Adapter Dictionary as required)
PLATFORM = 'DEFAULT_LOCAL'


# Network Adapter Dictionary
# The required interfaces are thus :
#   EXT_IF : Name of network interface used for the external communication or
#            Static/ElasticIP ipv4 address
#   INT_IF : Name of network interface used for the internal communication
#   BRIDGE_IF : Name of network interface used for the container network

# NOTE : In case of custom DNS configurations or using bound AWS ElasticIP
#        Just replace the required interface with the static IPv4
#        address of that interface.

ADAPTER_CONFIGURATION = {
# CONFIGURATION_NAME  : ('EXT_IF','INT_IF','BRIDGE_IF')
# Alternately for Static/ElasticIP configurations for external IP
# CONFIGURATION_NAME  : ('166.234.176.26','INT_IF','BRIDGE_IF')

'DEFAULT_LOCAL' : ('eth0','eth0','lxcbr0'),
'RACKSPACE'     : ('eth0','eth1','lxcbr0'),
'AWS'           : ('aws_dns','eth0','lxcbr0'),
#'CUSTOM'       : ('eth0','eth1','lxcbr0'),
}

#####################
# Do not modify this!
if PLATFORM not in ADAPTER_CONFIGURATION:
    raise ValueError('Invalid platform selected.')

EXT_IF, INT_IF, BRIDGE_IF = ADAPTER_CONFIGURATION[PLATFORM]

_IP_V4_REGEX = re.compile('^(?:(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.)'
                        '{3}(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$')

# Internal AWS url metadata retrieval address for type 'public-ipv4'
_AWS_IP_V4_ADDR = 'http://169.254.169.254/latest/meta-data/public-ipv4'

def _getIP(ifname):
    """ Get the IP address associated with a network interface.

        Based on:
            http://code.activestate.com/recipes/439094-get-the-ip-address-
            associated-with-a-network-inter/

            PSF License (Python Software Foundation)

        @param ifname:      Name of the network interface.
        @type  finame:      str

        @return:            IP address as a string, i.e. x.x.x.x
        @rtype:             str
    """
    # case where in a custom setup the global IP address is preconfigured
    # and does not necessarily bind to a network interface
    # eg: ElasticIP or custom DNS routings
    if _IP_V4_REGEX.match(ifname):
        return ifname
    
    # AWS Specific IP resolution method for the global ipv4 address
    if ifname == 'aws_dns' :
        return urllib2.urlopen(_AWS_IP_V4_ADDR).read()
    
    s=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return socket.inet_ntoa(fcntl.ioctl(
        s.fileno(),
        0x8915,  # SIOCGIFADDR
        struct.pack('256s', ifname[:15])
    )[20:24])

EXT_IP = _getIP(EXT_IF)
INT_IP = _getIP(INT_IF)
BRIDGE_IP = _getIP(BRIDGE_IF)
LOCALHOST_IP = _getIP('lo')

del _IP_V4_REGEX, _AWS_IP_V4_ADDR, _getIP
#####################

##################################################
###                                            ###
###              Machine settings              ###
###                                            ###
##################################################

# Maximum containers supported on this machine (Default : 10)
MAX_CONTAINER = 10

# Dictionary where the root of the container file system is located
ROOTFS = '/opt/rce/container/rootfs'

# Dictionary where temporary configuration files for containers are stored
CONF_DIR = '/opt/rce/container/config'

# Dictionary where temporary files of container are stored
DATA_DIR = '/opt/rce/container/data'

# Dictionary where the source code for the RCE is located
ROOT_SRC_DIR = '/opt/rce/framework'

# Dictionary where the custom ROS nodes are stored
#   - Has to be a list of (filesystem path, unique name) tuples.
#   - Elements with lower index are searched first.
#   - If 'unique name' is None or an empty string the basename of
#     'filesystem path' is used as name
ROOT_PKG_DIR = [('/opt/rce/test', 'rce_test')]

#####################
# Do not modify this!
def _checkPath(path, description):
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

def _processPkgPath(paths):
    """ Utility function to process the attribute ROOT_PKG_DIR from settings.
        
        @param paths:       List of attribute ROOT_PKG_DIR from settings.py
        @type  paths:       [(str, str/None)]
        
        @return:            Processed list; each element of ROOT_PKG_DIR will
                            be transformed into a tuple of source path and
                            destination path which can than be used to
                            extend the fstab file for the container creation.
        @rtype:             [(str, str)]
        
        @raise:             ValueError
    """
    pkgDir = []
    usedNames = set()
    
    for path, name in paths:
        _checkPath(path, 'ROS Package')
        
        if not name:
            name = os.path.basename(path)
        
        try:
            validateName(name)
        except IllegalName:
            raise ValueError("'{0}' is not a legal name.".format(name))
        
        if name in usedNames:
            raise ValueError("Package name '{0}' is not "
                             'unique.'.format(name))
        
        usedNames.add(name)
        
        # TODO: Contains container specific configuration: 'opt/rce/packages'
        pkgDir.append((path, os.path.join('opt/rce/packages', name)))
    
    return pkgDir

# Validate directory paths
_checkPath(CONF_DIR, 'Configuration')
_checkPath(DATA_DIR, 'Data')
_checkPath(ROOTFS, 'Container file system')
_checkPath(ROOT_SRC_DIR, 'RCE source')

# Process package directories
ROOT_PKG_DIR = _processPkgPath(ROOT_PKG_DIR)

del _checkPath, _processPkgPath
#####################

##################################################
###                                            ###
###             Admin Cred Settings            ###
###                                            ###
##################################################

# The file used by the RoboEarth Cloud Engine to save and record
# authentication for users
# Remember that the user running master must have write privileges to write
# to this file.

#####################
# Do not modify this!
def _prepare_conf():
    """ Helper function to build the basic conf dirs
    TODO: Expand this similar class to parse the entire settings file so as
          to store it in the user accessible dir
    """
    path = os.path.join(os.getenv('HOME'), '.rce')
    
    if not os.path.exists(path):
        os.makedirs(path)
    
    return os.path.join(path, 'creds')

PASSWORD_FILE = _prepare_conf()

del _prepare_conf
#####################

##################################################
###                                            ###
###            Developer Debug Mode            ###
###                                            ###
##################################################

DEV_MODE = True
