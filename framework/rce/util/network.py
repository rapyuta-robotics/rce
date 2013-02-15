#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     network.py
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

import socket
import fcntl
import struct
import re
import urllib2

IPv4_regex = '^(?:(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.){3}(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$'
# Internal AWS url metatdata retrival address for type 'public-ipv4'
aws_ipv4_addr = 'http://169.254.169.254/latest/meta-data/public-ipv4'

def getIP(ifname):
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
    # case where in a custom setup the global ipaddress is preconfigured
    # and does not necessarily bin to a network interface
    # eg: ElasticIP or custom DNS routings
    if re.match(IPv4_regex,ifname):
        return ifname
    # AWS Specific ip resolution method for the global ipv4 address
    if ifname == 'aws_dns' :
        return urllib2.urlopen(aws_ipv4_addr).read()

    s=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return socket.inet_ntoa(fcntl.ioctl(
        s.fileno(),
        0x8915,  # SIOCGIFADDR
        struct.pack('256s', ifname[:15])
    )[20:24])


def isLocalhost(ip):
    """ Check if the IP address matches the loopback address.

        @param ip:          IP address which should be checked
        @type  ip:          str

        @return:            True if the address is the loopback address;
                            False otherwise.
        @rtype:             bool
    """
    return ip in ('127.0.0.1', 'localhost')
