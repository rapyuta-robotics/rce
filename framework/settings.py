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
###          RCE settings           ###
###                                 ###
#######################################

# Custom converters for django/piston input to ROS messages.
CONVERTER_CLASSES = (
    'ServerUtil.Converter._ImageConverter.ImageConverter',
)

# Time in seconds after which unresponsive services are treated as dead.
WAIT_FOR_SERVICE_TIMEOUT = 5

# Time in seconds after which unused UIDs are freed up again.
UID_TIMEOUT = 60

# Time interval in which the newest load information are sent to the master node/load balancer
LOAD_INFO_UPDATE = 5

#######################################
###                                 ###
###        Machine settings         ###
###                                 ###
#######################################

# User for Server node
SERVER_USER = 'rce-user'

# Absolute path to directory where SSL files are stored
SSL_DIR = '/home/rce-user/rce'

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

# Bool whether SSL should be used for internal communication or not
USE_SSL = True

# Character pool which is used to build base addresses
import string
ADDR_BASE = string.ascii_uppercase

# Length of addresses
ADDRESS_LENGTH = 6

# IP address of Master
IP_MASTER = 'localhost'

# IP address of server for containers
IP_SERVER = '10.0.3.1'

# Port of Master
PORT_MASTER = 10011

# Port of Master for UID Server
PORT_UID = 10013

# Port of Container Manager
PORT_CONTAINER_MNGR = 10020

# Port of Server for other Servers
PORT_SERVER_SERVER = 10025

# Port of Server for Environment Nodes
PORT_SERVER_ENVIRONMENT = 11010

# Port of Launcher
PORT_LAUNCHER = 11011

#######################################
###                                 ###
###        Message settings         ###
###                                 ###
#######################################

# Maximal length for a message in bytes
MAX_LENGTH_MESSAGE = 10000000 # ~10mb

# Time in seconds after which queue messages are removed from the queue
MSG_QUQUE_TIMEOUT = 30

# Chunk size into which the message is split
CHUNK_SIZE = 8192
