#!/usr/bin/env python
# -*- coding: utf-8 -*-
#     
#     master.py
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
from twisted.cred.checkers import InMemoryUsernamePasswordDatabaseDontUse

# Custom imports
from rce.master.core import main
from rce.util.cred import RCECredChecker
import settings


# Credentials checkers used in the cloud engine
# TODO: At the moment only one user available and Password database in memory
extCred = RCECredChecker(settings.PASSWORD_FILE)
intCred = InMemoryUsernamePasswordDatabaseDontUse(robot='robot',
                                                  container='container',
                                                  environment='environment')


# Run main function
main(reactor, intCred, extCred, settings.MASTER_PORT, settings.HTTP_PORT,
     settings.INT_IF, settings.RCE_INTERNAL_PORT)
