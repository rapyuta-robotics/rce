#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     rce-core/rce/util/container.py
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

pjoin = os.path.join

# twisted specific imports
from twisted.python import log

# rce specific imports
from rce.util.process import execute


_CONFIG_CGROUP = """
lxc.cgroup.devices.deny = a
# /dev/null and zero
lxc.cgroup.devices.allow = c 1:3 rwm
lxc.cgroup.devices.allow = c 1:5 rwm
# consoles
lxc.cgroup.devices.allow = c 5:1 rwm
lxc.cgroup.devices.allow = c 5:0 rwm
lxc.cgroup.devices.allow = c 4:0 rwm
lxc.cgroup.devices.allow = c 4:1 rwm
# /dev/{,u}random
lxc.cgroup.devices.allow = c 1:9 rwm
lxc.cgroup.devices.allow = c 1:8 rwm
lxc.cgroup.devices.allow = c 136:* rwm
lxc.cgroup.devices.allow = c 5:2 rwm
# rtc
lxc.cgroup.devices.allow = c 254:0 rwm
"""

_CONFIG_CAP = """
# restrict capabilities
#   can't use: lxc.cap.drop = sys_admin
#   see: man capabilities for more information
#lxc.cap.drop = audit_control
#lxc.cap.drop = audit_write
#lxc.cap.drop = mac_admin
#lxc.cap.drop = mac_override
#lxc.cap.drop = mknod
#lxc.cap.drop = setfcap
#lxc.cap.drop = setpcap
#lxc.cap.drop = sys_boot
#lxc.cap.drop = sys_chroot
#lxc.cap.drop = sys_module
#lxc.cap.drop = sys_rawio
#lxc.cap.drop = sys_time
"""

_FSTAB_BASE = """
proc    {proc}    proc    nodev,noexec,nosuid    0 0
devpts    {devpts}    devpts    defaults    0 0
sysfs    {sysfs}    sysfs    defaults    0 0
"""

_FSTAB_BIND = """
{srcDir}    {dstDir}    none    bind{ro}    0 0
"""


class Container(object):
    """ Class representing a single container.
    """
    def __init__(self, reactor, rootfs, conf, hostname):
        """ Initialize the Container.

            @param reactor:     Reference to the twisted::reactor
            @type  reactor:     twisted::reactor

            @param rootfs:      Filesystem path of the root directory of the
                                container filesystem.
            @type  rootfs:      str

            @param conf:        Filesystem path of folder where configuration
                                files for the container should be stored.
            @type  conf:        str

            @param hostname:    Host name of the container.
            @type  hostname:    str
        """
        self._reactor = reactor
        self._rootfs = rootfs
        self._conf = pjoin(conf, 'config')
        self._fstab = pjoin(conf, 'fstab')
        self._hostname = hostname

        if not os.path.isabs(conf):
            raise ValueError('Container configuration directory is not an '
                             'absolute path.')

        if not os.path.isdir(conf):
            raise ValueError('Container Configuration directory does not '
                             'exist: {0}'.format(conf))

        if os.path.exists(self._conf):
            raise ValueError('There is already a config file in the container '
                             "configuration directory '{0}'.".format(conf))

        if os.path.exists(self._fstab):
            raise ValueError('There is already a fstab file in the container '
                             "configuration directory '{0}'.".format(conf))

        self._ifs = []
        self._fstabExt = []

    def addNetworkInterface(self, name, link=None, ip=None, up=None, down=None):
        """ Add a network interface to the configuration file.

            @param name:    Name of the network interface inside the container.
            @type  name:    str

            @param link:    Name of the network interface in the host system
                            which will be connected to the container network
                            interface.
            @type  link:    str

            @param ip:      IP address which will be assigned to the container
                            network interface. Use '0.0.0.0' for DHCP.
            @type  ip:      str

            @param up:      Path to a script which should be executed in the
                            host system once the interface has to be set up.
            @type  up:      str

            @param down:    Path to a script which should be executed in the
                            host system once the interface has to teared down.
            @type  down:    str
        """
        if up:
            if not os.path.isabs(up):
                raise ValueError('Path to up script has to be absolute.')

            if not os.path.isfile(up):
                raise ValueError('Path to up script is not a file.')

            if not os.access(up, os.X_OK):
                raise ValueError('Up script is not executable.')

        if down:
            if not os.path.isabs(down):
                raise ValueError('Path to down script has to be absolute.')

            if not os.path.isfile(down):
                raise ValueError('Path to down script is not a file.')

            if not os.access(down, os.X_OK):
                raise ValueError('Down script is not executable.')

        self._ifs.append((name, link, ip, up, down))

    def extendFstab(self, src, fs, ro):
        """ Add a line to the fstab file using bind.

            @param src:     Source path in host filesystem.
            @type  src:     str

            @param fs:      Path in container filesystem to which the source
                            should be bound.
            @type  fs:      str

            @param ro:      Flag to indicate whether bind should be read-only
                            or not.
            @type  ro:      bool
        """
        dst = pjoin(self._rootfs, fs)

        if not os.path.isabs(src):
            raise ValueError('Source path has to be absolute.')

        if not os.path.exists(src):
            raise ValueError('Source path does not exist.')

        if not os.path.exists(dst):
            raise ValueError('Destination path does not exist.')

        self._fstabExt.append((src, dst, ro))

    def _setupFiles(self):
        """ Setup the configuration and fstab file.
        """
        with open(self._conf, 'w') as f:
            # Write base config
            f.write('lxc.utsname = {0}\n'.format(self._hostname))
            f.write('\n')
            f.write('lxc.rootfs = {0}\n'.format(self._rootfs))
            f.write('lxc.mount = {0}\n'.format(self._fstab))

            # Write interface config
            for name, link, ip, up, down in self._ifs:
                f.write('\n')
                f.write('lxc.network.type = veth\n')
                f.write('lxc.network.flags = up\n')
                f.write('lxc.network.name = {0}\n'.format(name))

                if link:
                    f.write('lxc.network.link = {0}\n'.format(link))

                if ip:
                    f.write('lxc.network.ipv4 = {0}/24\n'.format(ip))

                if up:
                    f.write('lxc.network.script.up = {0}\n'.format(up))

                if down:
                    f.write('lxc.network.script.down = {0}\n'.format(down))


            # Write cgroup config
            f.write(_CONFIG_CGROUP)

            # Write capabilities config
            # TODO: Add at some point?
            # f.write(_CONFIG_CAP)

        with open(self._fstab, 'w') as f:
            f.write(_FSTAB_BASE.format(proc=pjoin(self._rootfs, 'proc'),
                                       devpts=pjoin(self._rootfs, 'dev/pts'),
                                       sysfs=pjoin(self._rootfs, 'sys')))

            for src, dst, ro in self._fstabExt:
                f.write(_FSTAB_BIND.format(srcDir=src, dstDir=dst,
                                           ro=',ro' if ro else ''))

    def start(self, name):
        """ Start the container.

            @param name:    Name of the container which should be started.
            @type  name:    str

            @return:        Deferred whose callback is triggered on success or
                            whose errback is triggered on failure with an
                            error message.
            @rtype:         twisted.internet.defer.Deferred
        """
        self._setupFiles()

        log.msg("Start container '{0}'".format(name))
        return execute(('/usr/bin/lxc-start', '-n', name, '-f', self._conf,
                        '-d'), reactor=self._reactor)

    def stop(self, name):
        """ Stop the container.

            @param name:        Name of the container which should be stopped.
            @type  name:        str

            @param command:     Deferred whose callback is triggered on success
                                or whose errback is triggered on failure with
                                an error message.
            @type  command:     twisted.internet.defer.Deferred
        """
        log.msg("Stop container '{0}'".format(name))
        return execute(('/usr/bin/lxc-stop', '-n', name), reactor=self._reactor)
