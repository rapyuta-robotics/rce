#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       models.py
#       
#       Copyright 2012 dominique hunziker <dominique.hunziker@gmail.com>
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

import os

from django.db import models
from django.db.models import fields
from django.contrib.auth.models import User
from django.db.models.signals import post_save
from django.core.exceptions import ValidationError

# ROS import
import setupROS
import roslib.names
import roslib.packages

VALID_TYPES = ( ('str', 'str'),
                ('bool', 'bool'),
                ('int', 'int'),
                ('float', 'float'),
                ('file', 'file') )

class Service(models.Model):
    """ Model for a service """
    name = fields.CharField(max_length=30, validators=[roslib.names.is_legal_base_name])
    pkg = fields.CharField(max_length=30)
    definition = fields.CharField(max_length=30)
    user = models.ForeignKey(User)
    
    def clean(self):
        """ Custom clean method to enable correct validation definition """
        if not roslib.packages.get_pkg_dir(self.pkg, False, '', user.get_profile().path):
            raise ValidationError('Package is not valid.')
        if not roslib.packages.find_resource(self.pkg, '{0}.srv'.format(self.definition)):
            raise ValidationError('Service definition is not valid.')

class Param(models.Model):
    """ Model for a parameter """
    name = models.CharField(max_length=30, validators=[roslib.names.is_legal_base_name])
    opt = models.BooleanField()
    paramType = models.CharField(max_length=5, choices=VALID_TYPES)
    default = models.CharField(max_length=100, blank=True)
    service = models.ForeignKey(Service)
    
    def clean(self):
        """ Custom clean method to enable correct validation of default field """
        if not self.opt:
            if self.paramType == 'bool':
                self.default = self.default.lower()
                
                if self.default not in ['true', 'false']:
                    if self.default == '1':
                        self.default = 'true'
                    elif self.default == '':
                        self.default = 'false'
                    else:
                        raise ValidationError('Default value is not a bool.')
            elif self.paramType == 'int':
                try:
                    self.default = str(int(self.default))
                except ValueError:
                    raise ValidationError('Default value is not an int.')
            elif self.paramType == 'float':
                try:
                    self.default = str(float(self.default))
                except ValueError:
                    raise ValidationError('Default value is not a float.')
            elif self.paramType == 'file':
                if self.default.find('../') != -1:
                    raise ValidationError('Default value is not a path. Only relative paths origin from the package are valid.')
                
                self.default = os.path.join(roslib.packages.get_dir_pkg(self.service.pkg), self.default)
                
                if not os.path.isfile(self.default):
                    raise ValidationError('Default value is not a path. Only relative paths origin from the package are valid.')

def create_new_dir():
    """ Create a new home directory for a user """
    # atm a dummy function
    return '/home/dominique/ROS/nodes'

class UserProfile(models.Model):
    """ UserProfile contains necessary additional data for a user """
    # This field is required.
    user = models.OneToOneField(User)
    
    # Other fields here
    path = models.CharField(max_length=200, default=create_new_dir)

def create_user_profile(sender, instance, created, **kwargs):
    """ Callback to ensure a new user profile is created if a new user is added """
    if created:
        UserProfile.objects.create(user=instance)

# register callback
post_save.connect(create_user_profile, sender=User)
