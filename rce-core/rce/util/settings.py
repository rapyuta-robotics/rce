#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     rce-core/rce/util/settings.py
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
#     \author/s: Dhananjay Sathe
#
#

# Python specific imports
from ConfigParser import SafeConfigParser
import os
import socket
import fcntl
import struct
import re
import urllib2

# rce specific imports
from rce.util.name import validateName, IllegalName


# Global storage of config parser
_settings = None


def getSettings():
    """ Get the cloud engine settings.
        The configuration file is parsed only once and cached for later.
    """
    global _settings
    if not _settings:
        _settings = _RCESettingsManager()
    return _settings


class _RCESettingsManager(SafeConfigParser, object):
    """ Configuration parser used for the cloud engine settings.
    """
    # case where in a custom setup the global IP address is preconfigured
    # and does not necessarily bind to a network interface
    # eg: ElasticIP or custom DNS routings
    _IP_V4_REGEX = re.compile('^(?:(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.)'
                              '{3}(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$')
    
    # Internal AWS url metadata retrieval address for type 'public-ipv4'
    _AWS_IP_V4_ADDR = 'http://169.254.169.254/latest/meta-data/public-ipv4'
    
    def __init__(self, *args, **kargs):
        """ Initialize the RCE settings manager. Additional arguments are passed
            to the __init__ of SafeConfigParser.
        """
        super(_RCESettingsManager, self).__init__(*args, **kargs)

        config_file = os.path.join(os.getenv('HOME'), '.rce', 'config.ini')

        # check if the config file exists
        if not os.path.exists(config_file):
            print('Config file missing please run the provision script first')
            exit()

        self.read(config_file)
        self._build_properties()
        
        # check paths and packages
        self._checkProcessPkgPaths()
        self._checkDirs()
        
        # set all the IP addresses
        self.EXT_IP, self.INT_IP, self.BRIDGE_IP = [self._getIP(iface)
                for iface in self.get('network', self.PLATFORM).split(',')]
        self.LOCALHOST_IP = self._getIP('lo')

        self.CONVERTER_CLASSES = [self.get('converters', option)
                                  for option in self.options('converters')]

    @staticmethod
    def _getIP(ifname):
        """ Get the IP address associated with a network interface.

            Based on:
                http://code.activestate.com/recipes/439094-get-the-ip-address-
                associated-with-a-network-inter/

                PSF License (Python Software Foundation)

            @param ifname:      Network interface descriptor; can be an IP
                                address, a name of a network interface, or the
                                string 'aws_dns' to dynamically resolve the IP
                                address of the network interface.
            @type  finame:      str

            @return:            IP address as a string, i.e. x.x.x.x
            @rtype:             str
        """
        if _RCESettingsManager._IP_V4_REGEX.match(ifname):
            return ifname

        # AWS Specific IP resolution method for the global ipv4 address
        if ifname == 'aws_dns':
            return urllib2.urlopen(_RCESettingsManager._AWS_IP_V4_ADDR).read()

        s=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        return socket.inet_ntoa(fcntl.ioctl(
            s.fileno(),
            0x8915,  # SIOCGIFADDR
            struct.pack('256s', ifname[:15])
        )[20:24])

    def _checkProcessPkgPaths(self):
        """ Utility function to process the section 'machine/packages' from
            settings.ini.

            @raise:             ValueError
        """
        pkgDir = []
        usedNames = set()

        for name, path in self.items('machine/packages'):
            self._checkPath(path, 'ROS Package')

            try:
                validateName(name)
            except IllegalName:
                raise ValueError("'{0}' is not a legal name.".format(name))

            if name in usedNames:
                raise ValueError("Package name '{0}' is not "
                                 'unique.'.format(name))

            usedNames.add(name)

            # TODO: Contains container specific configuration: opt/rce/packages
            pkgDir.append((path, os.path.join('opt/rce/packages', name)))

        self.ROOT_PKG_DIR = pkgDir

    def _checkDirs(self):
        """ Check all the basic directories to see if the paths are valid.

            @raise:             ValueError
        """
        self._checkPath(self.CONF_DIR, 'Configuration')
        self._checkPath(self.DATA_DIR, 'Data')
        self._checkPath(self.ROOTFS, 'Container file system')

    @staticmethod
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

    def _build_properties(self):
        """ Retrieve all settings, process them, and store them.
        """
        for section in self.sections():
            if section in ('global', 'comm', 'machine'):
                for opt in self.options(section):
                    val = self.get(section, opt)

                    if val.isdigit():
                        val = int(val)
                    elif val.lower() in ('true', 'false'):
                        val = val.lower() == 'true'

                    setattr(self, opt.upper(), val)
