#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       setupService.py
#       
#       Copyright 2011 dominique hunziker <dominique.hunziker@gmail.com>
#       
#       This program is free software; you can redistribute it and/or modify
#       it under the terms of the GNU General Public License as published by
#       the Free Software Foundation; either version 2 of the License, or
#       (at your option) any later version.
#       
#       This program is distributed in the hope that it will be useful,
#       but WITHOUT ANY WARRANTY; without even the implied warranty of
#       MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#       GNU General Public License for more details.
#       
#       You should have received a copy of the GNU General Public License
#       along with this program; if not, write to the Free Software
#       Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#       MA 02110-1301, USA.
#       
#       

from __future__ import print_function

import os
import shutil
import subprocess
import sys
from string import ascii_letters, digits

_USAGE="setupService.py 'path/packageName' [ 'ServiceName_1' 'ServiceName_2' ]"
_VALID_CHARS=ascii_letters+digits+'/_-'

# First check that enough arguments are supplied.
if len(sys.argv) <= 0:
	print(_USAGE)
	exit(0)

# Now check ich ROS environment is set up.
if 'ROS_ROOT' not in os.environ:
	print('Error: The ROS environment has to be set up before the setupService.py script can be run.')
	exit(1)

# Some very crude checking of the given packageName
for char in sys.argv[1]:
	if char not in _VALID_CHARS:
		print('Error: package name/path is not valid.')
		exit(1)

# Get paths
(packagePath, packageName)=os.path.split(os.path.abspath(sys.argv[1]))
templatePath=os.path.join(os.path.dirname(os.path.abspath(sys.argv[0])), 'templateService')

# Check if the directory 'template' is here
if not os.path.isdir(templatePath):
	print('Error: The setupService.py script has to be in the same directory as the template directory.')
	exit(1)

if 	not os.path.isfile(os.path.join(templatePath, 'initNode.py')) or \
	not os.path.isfile(os.path.join(templatePath, 'convert.py')) or \
	not os.path.isfile(os.path.join(templatePath, 'Worker.py')) or \
	not os.path.isfile(os.path.join(templatePath, '__init__.py')) or \
	not os.path.isfile(os.path.join(templatePath, 'Base.py')) or \
	not os.path.isfile(os.path.join(templatePath, 'Service.py')) or \
	not os.path.isfile(os.path.join(templatePath, 'debug.py')) or \
	not os.path.isfile(os.path.join(templatePath, 'Service.conf')) or \
	not os.path.isfile(os.path.join(templatePath, 'Query.srv')):
	print('Error: Could not find all necessary files in the directory template.')

# If the ROS package does not already exist create it
if not os.path.exists(packageName):
	subprocess.call(['roscreate-pkg', packageName, 'rospy', 'std_msgs', 'sensor_msgs'])

# Create subdirectories if they not already exist	
for directory in ['nodes', 'srv']:
	pathName=os.path.join(packagePath, packageName, directory)
	
	if not os.path.exists(pathName):
		os.mkdir(pathName)
	else:
		if not os.path.isdir(pathName):
			print('Error: Can not create the directory {0} in package {1}.'.format(directory, packageName))
			exit(1)

pathName=os.path.join(packagePath, packageName)

# Now create all necessary services
for serviceName in sys.argv[2:]:
	print(' ')
	
	# First a very curde name check
	for char in serviceName:
		if char not in _VALID_CHARS:
			print('Error: Service name is not valid.')
			exit(1)
	
	lowerName=serviceName[:1].lower()+serviceName[1:]
	upperName=serviceName[:1].upper()+serviceName[1:]
	
	# Check if files can be created
	if 	os.path.exists(os.path.join(pathName, 'nodes', upperName)) or \
		os.path.exists(os.path.join(pathName, 'src', '{0}s'.format(upperName))) or \
		os.path.exists(os.path.join(pathName, '{0}.conf'.format(lowerName))) or \
		os.path.exists(os.path.join(pathName, 'Query{0}.srv'.format(upperName))):
		print('Error: Can not create the service {0}.'.format(serviceName))
		exit(1)
	
	# Create files
	print('Creating service {0}.'.format(serviceName))
	
	# Create srv file
	shutil.copyfile(os.path.join(templatePath, 'Query.srv'), os.path.join(pathName, 'srv', 'Query{0}.srv'.format(upperName)))
	
	# Create python package
	pyPackagePath=os.path.join(pathName, 'src', '{0}s'.format(upperName))
	os.mkdir(pyPackagePath)
	shutil.copyfile(os.path.join(templatePath, '__init__.py'), os.path.join(pyPackagePath,  '__init__.py'))
	
	with open(os.path.join(templatePath, 'Base.py'), 'r') as f:
		content=f.read()
	
	content=content.replace('%Service%', upperName)
	
	with open(os.path.join(pyPackagePath, '{0}Base.py'.format(upperName)), 'w') as f:
		f.write(content)
	
	# Create node files
	with open(os.path.join(templatePath, 'Service.py'), 'r') as f:
		content=f.read()
	
	content=content.replace('%Service%', upperName)
	content=content.replace('%Package%', packageName)
	
	with open(os.path.join(pathName, 'nodes', '{0}.py'.format(upperName)), 'w') as f:
		f.write(content)
	
	with open(os.path.join(templatePath, 'debug.py'), 'r') as f:
		content=f.read()
	
	content=content.replace('%Service%', upperName)
	content=content.replace('%Package%', packageName)
	
	with open(os.path.join(pathName, 'nodes', 'debug{0}.py'.format(upperName)), 'w') as f:
		f.write(content)
	
	# Create config file
	shutil.copyfile(os.path.join(templatePath, 'Service.conf'), os.path.join(pathName, '{0}.conf'.format(lowerName)))
	
	print('Edit the files {0}, {1}, {2}, {3} and {4}.'.format(	os.path.join(packageName, 'srv', 'Query{0}.srv'.format(upperName)),
																os.path.join(packageName, 'src', '{0}s'.format(upperName), '{0}Base.py'.format(upperName)),
																os.path.join(packageName, 'nodes', '{0}.py'.format(upperName)),
																os.path.join(packageName, 'nodes', 'debug{0}.py'.format(upperName)),
																os.path.join(packageName, '{0}.conf'.format(lowerName))))

# Create general files
shutil.copyfile(os.path.join(templatePath, 'Worker.py'), os.path.join(pathName, 'src',  'Worker.py'))
	
with open(os.path.join(templatePath, 'convert.py'), 'r') as f:
	content=f.read()

content=content.replace('%Package%', packageName)

with open(os.path.join(pathName, 'src', 'convert.py'), 'w') as f:
	f.write(content)

with open(os.path.join(templatePath, 'initNode.py'), 'r') as f:
	content=f.read()

content=content.replace('%Package%', packageName)

with open(os.path.join(pathName, 'src', 'initNode.py'), 'w') as f:
	f.write(content)

print('\ndone')
