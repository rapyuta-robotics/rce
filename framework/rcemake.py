#!/usr/bin/env python
# -*- coding: utf-8 -*-
#     
#     rcemake.py
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
import sys

# Custom imports
import settings
from core.container import MakeContainer
from util import path as pathUtil


def main(reactor, pkgs):
    pkgDir = pathUtil.processPackagePaths(settings.ROOT_PKG_DIR)
    
    for _, path in pkgDir:
        os.mkdir(os.path.join(settings.ROOTFS, path))
    
    try:
        make = MakeContainer(reactor, settings.ROOTFS,
                             pkgDir)
        make.execute(pkgs)
        reactor.run()
    finally:
        for _, path in pkgDir:
            os.rmdir(os.path.join(settings.ROOTFS, path))


if __name__ == '__main__':
    from twisted.internet import reactor
    main(reactor, sys.argv[1:])
