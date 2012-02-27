#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       ros.py
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
import glob

# ROS import
import setupROS
import roslib.names
import roslib.packages

from django import forms
from django.forms import fields, ModelForm
from django.forms.formsets import formset_factory
from django.http import HttpResponse, HttpResponseBadRequest, HttpResponseRedirect
from django.shortcuts import render_to_response

from reappengine.models import Service, Param
from reappengine.site.site import webpage_values, error, success

class ServiceForm(ModelForm):
    """ Matching form for a service """
    name = fields.CharField(label='Service Name', max_length=30, required=True)
    pkg = fields.ChoiceField(label='Package', choices=(), required=True)
    definition = fields.ChoiceField(label='Service Definition', required=True, widget=forms.Select(attrs={ 'disabled' : 'true' }))
    
    def __init__(self, pkgs, *args, **kwargs):
        super(ServiceForm, self).__init__(*args, **kwargs)
        self.fields['pkg'] = fields.ChoiceField(label='Package', choices=pkgs, required=True)
    
    class Meta:
        model = Service
        exclude = ('user',)

class ParamForm(ModelForm):
    """ Matching form for a parameter """
    class Meta:
        model = Param
        exclude = ('service',)

ParamFormSet = formset_factory(ParamForm)

def select(request):
    """ Webpage with form to register a new service """
    if not request.user.is_authenticated():
        return HttpResponseRedirect("/login/")
    
    root_path = request.user.get_profile().path
    pkgs = (('', 'Select...'),) + tuple((pkg, pkg) for pkg in roslib.packages.list_pkgs_by_path(root_path))
    
    return render_to_response('service.html', webpage_values(request, { 'service' : ServiceForm(pkgs), 'paramForm' : ParamFormSet() }))

def feedService(request, pkg):
    """ Used by javascript function to get available services in specified package """
    if not request.user.is_authenticated():
        return HttpResponseRedirect("/login/")
    
    pkg = roslib.packages.get_pkg_dir(pkg)
    
    if not pkg:
        return HttpResponseBadRequest()
    
    return HttpResponse('\n'.join(os.path.basename(srvPath)[:-4] for srvPath in glob.glob(os.path.join(pkg, 'srv/*.srv'))), content_type='text/plain')

def sumit(request):
    """ Process form to register a new service """
    if not request.user.is_authenticated():
        return HttpResponseRedirect("/login/")
    
    if request.method == 'POST':
        # Fetch data
        srvName = request.POST.get('srvName', '')
        pkgName = request.POST.get('pkg', '')
        srvDef = request.POST.get('srvDef', '')
        
        # Validate ROS name
        if not roslib.names.is_legal_base_name(srvName):
            return error(request, nextPage='/service/select/', errorMessage='Selected service name is not valid.')
        
        # Validate package name
        pkgDir = roslib.packages.get_pkg_dir(pkgName)
        
        if not pkgDir:
            return error(request, nextPage='/service/select/', errorMessage='Selected package does not exist.')
        
        # Validate service definition
        if not os.path.isfile(os.path.join(pkgDir, 'srv', '{0}.srv'.format(srvDef))):
            return error(request, nextPage='/service/select/', errorMessage='Selected service definition does not exist.')
        
        #form = PartialAuthorForm(request.POST)
        #author = form.save(commit=False)
        #author.title = 'Mr'
        #author.save()
        
        
        return success(request)
    else:
        return error(request, nextPage='/')
