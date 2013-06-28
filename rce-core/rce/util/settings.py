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
#     \author/s: Dhananjay Sathe, Dominique Hunziker
#
#

# Python specific imports
import os
import string
import socket
import fcntl
import struct
import re
import urllib2
from ConfigParser import SafeConfigParser, Error

# rce specific imports
from rce.util.name import validateName, IllegalName


# Global storage of config parser
_settings = None

# Path where the configuration file can be found
PATH = os.path.join(os.getenv('HOME'), '.rce', 'config.ini')


def get_host_ubuntu_release():
    """ Parse configuration file to get the codename of the Ubuntu release of
        the host filesystem.
    """
    with open('/etc/lsb-release') as config:
        for line in config:
            k, v = line.split('=')

            if k == 'DISTRIB_CODENAME':
                return v.strip(string.whitespace + '\'"')

    raise ValueError('Corrupt release information.')


class NoValidSettings(Exception):
    """ Exception is raised in case there is no valid configuration found
        for the cloud engine.
    """


def getSettings(throw=False, checks=True):
    """ Get the cloud engine settings.
        The configuration file is parsed only once and cached for later.

        @raise:                 rce.util.settings.NoValidSettings
    """
    global _settings

    if not _settings:
        try:
            _settings = _getSettings(checks)
        except NoValidSettings as e:
            _settings = e

    if isinstance(_settings, NoValidSettings):
        if throw:
            raise _settings
        else:
            print(str(e))
            print('Please check your configuration.')
            exit(1)

    return _settings


def _getSettings(checks):
    """ Get the settings for the cloud engine. Does the heavy lifting.
    """
    parser = _RCESettingsParser()

    if PATH not in parser.read(PATH):
        raise NoValidSettings('Config file is missing.')

    try:
        return _Settings.load(parser, checks)
    except (Error, ValueError) as e:
        raise NoValidSettings(str(e))


def _path_exists(path, description):
    """ Check if the path is valid and exists.

        @param path:            Path which should be checked.
        @type  path:            str

        @param description:     Description which is used for the error
                                message if necessary.
        @type  description:     str

        @raise:                 ValueError, if path is not valid.
    """
    if not os.path.isabs(path):
        raise ValueError('{0} is not an absolute path.'.format(description))

    if not os.path.exists(path):
        raise ValueError('{0} does not exist: {1}'.format(description, path))


def _valid_dir(path, description):
    """ Check if the path is a valid directory.

        @param path:            Path which should be checked.
        @type  path:            str

        @param description:     Description which is used for the error
                                message if necessary.
        @type  description:     str

        @raise:                 ValueError, if path is not valid.
    """
    _path_exists(path, description)

    if not os.path.isdir(path):
        raise ValueError('{0} is not directory.'.format(description))


def _getIP(ifname):
    """ Get the IP address associated with a network interface.

        Based on:
            http://code.activestate.com/recipes/439094-get-the-ip-address-
            associated-with-a-network-inter/

            PSF License (Python Software Foundation)

        @param ifname:          The name of the network interface for which the
                                IP address should be retrieved.
        @type  finame:          str

        @return:                IP address as a string, i.e. x.x.x.x
        @rtype:                 str
    """
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    try:
        return socket.inet_ntoa(fcntl.ioctl(
            s.fileno(),
            0x8915,  # SIOCGIFADDR
            struct.pack('256s', ifname[:15])
        )[20:24])
    except IOError:
        raise ValueError("Either the interface '{0}' isn't connected or there "
                         "seems to be a network error".format(ifname))


class _Settings(object):
    """ Class contains all settings variables.
    """
    def __init__(self):
        """ Initialize the variables with None. To load the Settings use the
            classmethod 'load'.
        """
        # Global
        self._gzip_lvl = None
        self._dev_mode = None
        self._pw_file = None
        self._host_ubuntu = None
        self._host_ros = None
        self._container_ubuntu = None
        self._container_ros = None

        # Network
        self._container_if = None
        self._external_ip = None
        self._internal_ip = None
        self._container_ip = None
        self._localhost_ip = None

        # Comm
        self._http_port = None
        self._ws_port = None
        self._internal_port = None
        self._external_port = None
        self._comm_port = None
        self._ros_proxy_port = None

        # Converters
        self._converters = None

        # Machine
        self._size = None
        self._cpu = None
        self._memory = None
        self._bandwidth = None
        self._special_features = None
        self._rootfs = None
        self._conf_dir = None
        self._data_dir = None
        self._packages = None

    @property
    def gzip_lvl(self):
        """ Compression level used in ROS message forwarder. """
        return self._gzip_lvl

    @property
    def dev_mode(self):
        """ Flag which is True if the cloud engine runs in developer mode. """
        return self._dev_mode

    @property
    def pw_file(self):
        """ Path to the credentials database. """
        return self._pw_file

    @property
    def host_ubuntu_release(self):
        """ Ubuntu release used in the host filesystem. """
        return self._host_ubuntu

    @property
    def host_ros_release(self):
        """ ROS release used in the host filesystem. """
        return self._host_ros

    @property
    def container_ubuntu_release(self):
        """ Ubuntu release used inside the container. """
        return self._container_ubuntu

    @property
    def container_ros_release(self):
        """ ROS release used inside the container. """
        return self._container_ros

    @property
    def container_interface(self):
        """ Name of the container network interface. """
        return self._container_if

    @property
    def external_IP(self):
        """ IP address of network interface used for external communication. """
        return self._external_ip

    @property
    def internal_IP(self):
        """ IP address of network interface used for internal communication. """
        return self._internal_ip

    @property
    def container_IP(self):
        """ IP address of network interface used for communication with
            containers.
        """
        return self._container_ip

    @property
    def localhost_IP(self):
        """ IP address of loopback network interface. """
        return self._localhost_ip

    @property
    def http_port(self):
        """ Port on which the Master process is listening for HTTP requests
            from the cloud engine clients.
        """
        return self._http_port

    @property
    def ws_port(self):
        """ Port on which the Robot processes are listening for WebSocket
            connections from the cloud engine clients.
        """
        return self._ws_port

    @property
    def external_port(self):
        """ Port on which the Master process is listening for external
            PerspectiveBroker connections.
        """
        return self._external_port

    @property
    def internal_port(self):
        """ Port on which the Master process is listening for cloud engine
            internal PerspectiveBroker connections.
        """
        return self._internal_port

    @property
    def comm_port(self):
        """ Port on which the Endpoint processes are listening for cloud engine
            internal data protocol connections.
        """
        return self._comm_port

    @property
    def ros_proxy_port(self):
        """ Port on which the ROS proxy processes are listening for HTTP
            requests from the cloud engine clients.
        """
        return self._ros_proxy_port

    @property
    def converters(self):
        """ List of custom message converters which are used in the Robot
            processes.
        """
        return self._converters

    @property
    def size(self):
        """ Maximum number of containers which can run in the machine. """
        return self._size

    @property
    def cpu(self):
        """ Parameter to define CPU attributes/capacity. """
        return self._cpu

    @property
    def memory(self):
        """ Parameter to define memory attributes/capacity. """
        return self._memory

    @property
    def bandwidth(self):
        """ Parameter to define bandwidth attributes/capacity. """
        return self._bandwidth

    @property
    def special_features(self):
        """ Parameter to define special attributes like avx,gpu,hadoop, etc. """
        return self._special_features

    @property
    def rootfs(self):
        """ Path to the root directory of the container filesystem. """
        return self._rootfs

    @property
    def conf_dir(self):
        """ Path to the directory in which temporary configuration files for
            the containers are stored.
        """
        return self._conf_dir

    @property
    def data_dir(self):
        """ Path to the directory in which temporary data files for the
            containers are stored.
        """
        return self._data_dir

    @property
    def packages(self):
        """ List of custom ROS packages which are mounted using bind into the
            container filesystem. Each element is a tuple containing the
            path to the directory of the ROS package in the host filesystem as
            well as the path to the directory of the ROS package in the
            container filesystem.
        """
        return self._packages

    @classmethod
    def load(cls, parser, checks):
        """ Factory method which creates a new Settings object using the
            provided cloud engine settings parser.

            @param parser:      Cloud engine settings parser which is used to
                                parse the configuration file.
            @type  parser:      rce.util.settings._RCESettingsParser

            @param checks:      Enable/Disable path checks.
            @type  checks:      bool

            @return:            New _Settings instance containing the parsed
                                settings.
            @rtype:             rce.util.settings._Settings
        """
        settings = cls()

        # Global
        settings._gzip_lvl = parser.getint('global', 'gzip_lvl')
        settings._dev_mode = parser.getboolean('global', 'dev_mode')
        settings._pw_file = parser.get('global', 'password_file')
        settings._host_ubuntu = get_host_ubuntu_release()
        settings._host_ros = parser.get('global', 'host_ros_release')
        settings._container_ros = parser.get('global', 'container_ros_release')
        settings._container_ubuntu = parser.get('global',
                                                'container_ubuntu_release')

        # Network
        settings._container_if = parser.get('network', 'container_if')
        settings._external_ip = parser.getIP('network', 'external_if')
        settings._internal_ip = parser.getIP('network', 'internal_if')
        settings._container_ip = parser.getIP('network', 'container_if')
        settings._localhost_ip = _getIP('lo')

        # Comm
        settings._http_port = parser.getint('comm', 'http_port')
        settings._ws_port = parser.getint('comm', 'ws_port')
        settings._internal_port = parser.getint('comm', 'internal_port')
        settings._external_port = parser.getint('comm', 'external_port')
        settings._comm_port = parser.getint('comm', 'comm_port')
        settings._ros_proxy_port = parser.getint('comm', 'ros_proxy_port')

        # Converters
        settings._converters = tuple(c for _, c in parser.items('converters'))

        # Machine
        settings._size = parser.getint('machine', 'size')
        settings._cpu = parser.getint('machine', 'cpu')
        settings._memory = parser.getint('machine', 'memory')
        settings._bandwidth = parser.getint('machine', 'bandwidth')
        settings._rootfs = parser.get('machine', 'rootfs')
        settings._conf_dir = parser.get('machine', 'conf_dir')
        settings._data_dir = parser.get('machine', 'data_dir')
        # Figure out the special features
        special_features = parser.get('machine', 'special_features')
        settings._special_features = [i.strip() for i in
                                      special_features.strip('[]').split(',')]

        # ROS packages
        settings._packages = []
        usedNames = set()

        if checks:
            for name, path in parser.items('machine/packages'):
                _valid_dir(path, "ROS package '{0}'".format(name))

                try:
                    validateName(name)
                except IllegalName:
                    raise ValueError("Package name '{0}' is not a legal "
                                     'name.'.format(name))

                if name in usedNames:
                    raise ValueError("Package name '{0}' is not "
                                     'unique.'.format(name))

                usedNames.add(name)

                settings._packages.append((path,
                                           os.path.join('opt/rce/packages', name)))

            settings._packages = tuple(settings._packages)

            # Validate paths
            _valid_dir(settings._rootfs, 'Container file system')
            _valid_dir(settings._conf_dir, 'Configuration directory')
            _valid_dir(settings._data_dir, 'Data directory')

        return settings


class _RCESettingsParser(SafeConfigParser, object):
    """ Configuration parser used for the cloud engine settings.
    """
    # case where in a custom setup the global IP address is preconfigured
    # and does not necessarily bind to a network interface
    # e.g.: ElasticIP or custom DNS routings
    _IP_V4_REGEX = re.compile('^(?:(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.)'
                              '{3}(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$')

    # Internal AWS url metadata retrieval address for type 'public-ipv4'
    _AWS_IP_V4_ADDR = 'http://169.254.169.254/latest/meta-data/public-ipv4'

    def __init__(self, *args, **kwargs):
        super(_RCESettingsParser, self).__init__(*args, **kwargs)
        self._get_iface_list()

    def _get_iface_list(self):
        self._ifaces = set()
        with open('/proc/net/dev') as net_devices:
            for line in net_devices.readlines()[2:]:
                self._ifaces.add(line.split(':')[0].strip())

    def getIP(self, section, option):
        """ Get IP address.

            @param section:     Section from which the option should be
                                retrieved.
            @type  section:     str

            @param option:      Option which should be parsed for an IP address.
            @type  option:      str

            @return:            IP address as a string, i.e. x.x.x.x
            @rtype:             str
        """
        ifname = self.get(section, option)

        if _RCESettingsParser._IP_V4_REGEX.match(ifname):
            return ifname

        # AWS Specific IP resolution method for the global ipv4 address
        try:
            if ifname == 'aws_dns':
                return urllib2.urlopen(_RCESettingsParser._AWS_IP_V4_ADDR,
                                       timeout=3).read()
        except urllib2.URLError:
            raise NoValidSettings('There seems to be something wrong with '
                                  'AWS or configuration settings.')

        if ifname not in self._ifaces:
            raise NoValidSettings("The network device '{0}' does not exist on "
                                  'your system check your '
                                  'configuration.'.format(ifname))

        return _getIP(ifname)
