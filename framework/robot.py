#!/usr/bin/env python
# -*- coding: utf-8 -*-
#     
#     robot.py
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

# Before we start to import everything check if we have the right amount of
# arguments
from hashlib import md5

def _get_argparse():
    from argparse import ArgumentParser

    parser = ArgumentParser(prog='robot',
                            description='RCE Robot Client Process.')

    parser.add_argument('ipMaster', help='IP address of master process.',
                        type=str)
    parser.add_argument('client-password', help='Admin-Client password',
                        type=str)

    return parser


# twisted specific imports
from twisted.internet import reactor
from twisted.cred.credentials import UsernamePassword

# Custom imports
from rce.robot import main
import settings

if __name__ == '__main__':

    args = _get_argparse().parse_args()
    cred = UsernamePassword('robot', md5(args.client-password).digest())

    # Run main function
    main(reactor, cred, args.ipMaster, settings.MASTER_PORT, settings.EXT_IF,
         settings.WS_PORT, settings.RCE_INTERNAL_PORT, settings.ROOT_PKG_DIR,
         settings.CONVERTER_CLASSES)
