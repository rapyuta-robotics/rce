#!/usr/bin/env python
# -*- coding: utf-8 -*-
#     
#     template.py
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

CONFIG = """lxc.utsname = ros

lxc.tty = 4
lxc.pts = 1024
lxc.rootfs = {rootfs}
lxc.mount = {fstab}

lxc.network.type = veth
lxc.network.flags = up
lxc.network.name = eth0
lxc.network.link = lxcbr0
lxc.network.ipv4 = 0.0.0.0

lxc.cgroup.devices.deny = a
# /dev/null and zero
lxc.cgroup.devices.allow = c 1:3 rwm
lxc.cgroup.devices.allow = c 1:5 rwm
# consoles
lxc.cgroup.devices.allow = c 5:1 rwm
lxc.cgroup.devices.allow = c 5:0 rwm
lxc.cgroup.devices.allow = c 4:0 rwm
lxc.cgroup.devices.allow = c 4:1 rwm
# /dev/{{,u}}random
lxc.cgroup.devices.allow = c 1:9 rwm
lxc.cgroup.devices.allow = c 1:8 rwm
lxc.cgroup.devices.allow = c 136:* rwm
lxc.cgroup.devices.allow = c 5:2 rwm
# rtc
lxc.cgroup.devices.allow = c 254:0 rwm
"""

FSTAB = """proc    {proc}    proc    nodev,noexec,nosuid    0 0
devpts    {devpts}    devpts    defaults    0 0
sysfs    {sysfs}    sysfs    defaults    0 0
{homeDir}    {fsHome}    none    bind    0 0
{srcDir}    {fsSrc}    none    bind,ro    0 0
{dataDir}    {fsData}    none    bind    0 0
{upstartComm}    {fsComm}    none    bind,ro    0 0
{upstartLauncher}    {fsLauncher}    none    bind,ro    0 0
"""

FSTAB_PKG = """{pkgDir}    {fsPkg}    none    bind,ro    0 0
"""

UPSTART_COMM = """# description
author "Dominique Hunziker"
description "ReCloudEngine Comm - Framework for managing and using ROS Apps"

# start/stop conditions
start on started rce
stop on stopping rce

kill timeout 5

script
    # setup environment
    . /opt/rce/setup.sh
    
    # start environment node
    start-stop-daemon --start -c rce:rce -d /opt/rce/data --retry 5 --exec /opt/rce/src/environment.py -- {commID} 10.0.3.1 {serverID}
end script
"""

UPSTART_LAUNCHER = """# description
author "Dominique Hunziker"
description "ReCloudEngine Launcher - Framework for managing and using ROS Apps"

# start/stop conditions
start on started rce
stop on stopping rce

# timeout before the process is killed; generous as a lot of processes have
# to be terminated by the launcher.
kill timeout 30

script
    # setup environment
    . /opt/rce/setup.sh
    
    # start launcher
    start-stop-daemon --start -c ros:ros -d /home/ros --retry 5 --exec /opt/rce/src/launcher.py
end script
"""
