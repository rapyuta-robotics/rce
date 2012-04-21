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
from django.db.models import Q
from django.contrib.auth.models import User
from django.db.models.signals import post_save
from django.core.exceptions import ValidationError

# ROS import
import reappengine.ROS      # sets up the necessary environment variables
import roslib.names
import roslib.packages

MAX_LENGTH = 30

VALID_PARAM_TYPES = (
    ('str', 'str'),
    ('bool', 'bool'),
    ('int', 'int'),
    ('float', 'float'),
    ('file', 'file'),
)

VALID_INTERFACE_TYPES = (
    ('srv', 'Service'),
    ('topic', 'Topic'),
)

def rosNameValidator(value):
    """ Validator to check whether value is a valid ROS base name """
    if not roslib.names.is_legal_base_name(value):
        raise ValidationError('"{0}" is not a valid ROS base name.'.format(value))

class ROSNameField(models.CharField):
    """ CharField which checks for a valid base name """
    default_validators = [rosNameValidator]

class Package(models.Model):
    """ Model for a package """
    name = ROSNameField(max_length=MAX_LENGTH, verbose_name='Package Name')
    user = models.ForeignKey(User, verbose_name='Package Owner')

    def clean(self):
        """ Custom clean method to validate package """
        try:
            roslib.packages.get_pkg_dir(self.name, True, '', self.user.get_profile().path)
        except roslib.packages.InvalidROSPkgException:
            raise ValidationError('Package is not valid.')

        if Package.objects.filter(~Q(user=self.user) & Q(name=self.name)):
            raise ValidationError('Package name is already used by another user.')

class Node(models.Model):
    """ Model for a ROS node """
    name = models.CharField(max_length=MAX_LENGTH, verbose_name='Node Name')
    pkg = models.ForeignKey(Package, verbose_name='Package')

    def clean(self):
        """ Custom clean method to validate node """
        try:
            if not roslib.packages.find_node(self.pkg.name, self.name):
                raise ValidationError('Node executable is not valid.')
        except roslib.packages.InvalidROSPkgException:
            raise ValidationError('Package is not valid.')

        if Node.objects.filter(name=self.name, pkg=self.pkg):
            raise ValidationError('Node already exists for this package.')

class Param(models.Model):
    """ Model for a parameter """
    name = ROSNameField(max_length=MAX_LENGTH, verbose_name='Parameter Name')
    opt = models.BooleanField(verbose_name='Optional Parameter')
    paramType = models.CharField(max_length=5, choices=VALID_PARAM_TYPES, verbose_name='Parameter Type')
    default = models.CharField(max_length=100, blank=True, verbose_name='Default Value/Path')
    node = models.ForeignKey(Node)

    def clean(self):
        """ Custom clean method to validate default field of parameter """
        if self.opt:
            if self.paramType == 'bool':
                self.default = self.default.lower()

                if self.default not in ['true', 'false']:
                    if self.default == '1':
                        self.default = 'true'
                    elif self.default == '0':
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
                    raise ValidationError('Default value is not a path. Only relative paths originating from the package are valid.')

                self.default = os.path.join(self.node.pkg.user.get_profile().path, self.default)

                if not os.path.isfile(self.default):
                    raise ValidationError('Default value is not a path. Only relative paths originating from the home directory are valid.')

class Interface(models.Model):
    """ Model for a interface, i.e. service or topic """
    msgType = models.CharField(max_length=5, choices=VALID_INTERFACE_TYPES, verbose_name='Interface Type')
    name = ROSNameField(max_length=MAX_LENGTH, verbose_name='Name')
    msgDef = ROSNameField(max_length=MAX_LENGTH, verbose_name='Definition')
    pkg = models.ForeignKey(Package, verbose_name='Package')

    def clean(self):
        """ Custom clean method to validate interface """
        try:
            pkgDir = roslib.packages.get_pkg_dir(self.pkg.name)
        except roslib.packages.InvalidROSPkgException:
            raise ValidationError('Package is not valid.')

        if self.msgType == 'srv':
            if not os.path.isfile(os.path.join(pkgDir, 'srv', '{0}.srv'.format(self.msgDef))):
                raise ValidationError('Definition is not valid.')
        elif self.msgType == 'topic':
            if not os.path.isfile(os.path.join(pkgDir, 'msg', '{0}.msg'.format(self.msgDef))):
                raise ValidationError('Definition is not valid.')
        else:
            raise ValidationError('Type is not valid.')

        if Interface.objects.filter(name=self.name, pkg=self.pkg):
            raise ValidationError('{0} already exists for this package.'.format(self.msgType))

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
