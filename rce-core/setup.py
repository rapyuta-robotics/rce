#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     rce-core/setup.py
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

from setuptools import setup

LONG_DESCRIPTION = """ TODO """

setup(
    name='rce-core',
    description='Core package of RoboEarth Cloud Engine.',
    long_description=LONG_DESCRIPTION,
    version='0.3',
    author='Dominique Hunziker',
    author_email='dominique.hunziker@gmail.com',
    url='http://github.com/IDSCETHZurich/rce.git',
    license='Apache 2.0',
    install_requires=['rce_comm', 'rce_util', 'python_iptables'],
    keywords='',
    platforms='',
    namespace_packages=['rce', 'rce.util'],
    packages=['rce', 'rce.core', 'rce.slave', 'rce.monitor',
              'rce.monitor.interface', 'rce.util', 'rce.util.converters'],
    scripts=['scripts/rce-make', 'scripts/rce-setup-rcemake',
             'scripts/rce-master', 'scripts/rce-container',
             'scripts/rce-robot', 'scripts/rce-environment',
             'scripts/rce-rosproxy', 'scripts/rce-maintain'],
    package_data={'rce.core': ['data/*.upstart', 'data/*.script']},
)
