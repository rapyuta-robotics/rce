#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       settings.py
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

#######################################
###                                 ###
###         Master settings         ###
###                                 ###
#######################################

# Custom converters for django/piston input to ROS messages.
CONVERTER_CLASSES = (
    'ServerUtil.Converter._ImageConverter.ImageConverter',
)

# Definition of django database
DJANGO_DB = {   'DATABASE_ENGINE': 'django.db.backends.sqlite3',
                'DATABASE_NAME': '/var/django/reappengine/data/users.db',
                'TIME_ZONE' : 'Europe/Zurich' }

DJANGO_ROOT_DIR = '/home/dominique/reappengine/framework/django'

# Time in seconds after which inactive entries in the databases are removed.
TIMEOUT = 600

# Time in seconds after which unresponsive services are treated as dead.
WAIT_FOR_SERVICE_TIMEOUT = 5

# Path to directory where temporary files to store the results should be kept
TMP_RESULT_DIR = '/var/www/tmp'

# Time in seconds after which unused UIDs are freed up again.
UID_TIMEOUT = 60

#######################################
###                                 ###
###        Machine settings         ###
###                                 ###
#######################################

# Absolute path to root file system for container
ROOTFS = '/opt/rce/container/rootfs'

# Absolute path to directory where config/stab files for containers should be stored
CONF_DIR = '/opt/rce/container/config'

# Absolute path to directory where temporary folders for container data should be stored
DATA_DIR = '/opt/rce/container/data'

# Absolute path to source root directory
ROOT_SRC_DIR = '/opt/rce/framework'

# Absolute path to source package directory
ROOT_PKG_DIR = '/opt/rce/test'

# Time interval in which the newest load information are sent to the master node/load balancer
LOAD_INFO_UPDATE = 5

#######################################
###                                 ###
###      Environment settings       ###
###                                 ###
#######################################

# Time in seconds after which results are removed
RESULT_TIMEOUT = 600

#######################################
###                                 ###
###        Network settings         ###
###                                 ###
#######################################

# Character pool which is used to build base addresses
import string
ADDR_BASE = string.ascii_uppercase

# Length of addresses
ADDRESS_LENGTH = 6

# IP address of Master
IP_MASTER = 'localhost'

# IP address of server for containers
IP_SATELLITE = '10.0.3.1'

# Port of Master
PORT_MASTER = 10011

# Port of Master for UID Server
PORT_UID = 10013

# Port of Database
PORT_DB = 10015

# Port of Container Manager
PORT_CONTAINER_MNGR = 10020

# Port of Server for other Servers
PORT_SATELLITE_SATELLITE = 10025

# Port of Server for Environment Nodes
PORT_SATELLITE_ENVIRONMENT = 11010

# Port of Launcher
PORT_LAUNCHER = 11011

#######################################
###                                 ###
###        Message settings         ###
###                                 ###
#######################################

# Maximal length for a message in bytes
MAX_LENGTH_MESSAGE = 99999

# Time in seconds after which queue messages are removed from the queue
MSG_QUQUE_TIMEOUT = 30

# Chunk size into which the message is split
CHUNK_SIZE = 8192
