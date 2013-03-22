#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     container.py
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
#     \author/s: Dominique Hunziker
#
#

# twisted specific imports
from twisted.internet import reactor
from twisted.cred.credentials import UsernamePassword
from hashlib import sha256

# Custom imports
from rce.container import main
import settings


def _get_argparse():
    from argparse import ArgumentParser

    parser = ArgumentParser(prog='container',
                            description='RCE Container Client Slave.')

    parser.add_argument('MasterIP', help='IP address of Master Process.',
                        type=str)
    parser.add_argument('MasterPassword', help='Admin Password',
                        type=str)
    parser.add_argument('InfraPassword', help='Admin-Infrastructure Password',
                        type=str)
    parser.add_argument('--maxContainers', help='Maximum Number of Containers '
                        'to support on this machine.', type=int,
                        default=settings.MAX_CONTAINER)

    return parser


if __name__ == '__main__':
    # Credentials which should be used to login to Master process
    
    
    # Before we start, check if:
    #  - the script can be executed, i.e. if we have the necessary super
    #    user privileges
    #  - we have the right amount of arguments
    import os, sys
    
    if os.getuid() != 0:
        print('{0} has to be run as super '
              'user.'.format(os.path.basename(sys.argv[0])))
        exit(1)
    
    args = _get_argparse().parse_args()
    cred = UsernamePassword('container', sha256(args.InfraPassword).digest())
    MasterPasswd = sha256(args.MasterPassword).digest()
    
    main(reactor, cred, args.MasterIP, MasterPasswd, cred.password, 
         settings.MASTER_PORT, settings.INT_IF, settings.BRIDGE_IF,
         settings.RCE_INTERNAL_PORT, settings.ROS_PROXY_PORT, settings.ROOTFS, 
         settings.CONF_DIR, settings.DATA_DIR, settings.ROOT_SRC_DIR, 
         settings.ROOT_PKG_DIR, args.maxContainers)

