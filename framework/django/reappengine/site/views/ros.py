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
import stat

# ROS import
import reappengine.ROS      # sets up the necessary environment variables
import roslib.names
import roslib.packages

from django.http import HttpResponse, HttpResponseBadRequest, HttpResponseRedirect
from django.shortcuts import render_to_response

from reappengine.models import Package, Node
from reappengine.forms import PackageForm, NodeForm, ParamFormSet, InterfaceFormSet
from reappengine.site.site import webpage_values, error, success

def service(request):
    """ Webpage with form to register a new service """
    if not request.user.is_authenticated():
        return HttpResponseRedirect("/login/")
    
    if request.method == 'POST':
        # Validate Package form
        package = Package(user=request.user)
        packageForm = PackageForm(request.POST, request.FILES, instance=package, prefix='package')
        
        if packageForm.is_valid():
            packageDict = packageForm.cleaned_data.copy()
            packageDict['user'] = request.user
            package, created = Package.objects.get_or_create(**packageDict)
        else:
            return error(request, nextPage='/add/', errorTitle="Package", errorMessage=str(packageForm.errors))
        
        # Validate Node form
        node = Node(pkg=package)
        nodeForm = NodeForm(request.POST, request.FILES, instance=node, prefix='node')
        
        if nodeForm.is_valid():
            node = nodeForm.save(commit=False)
        else:
            return error(request, nextPage='/add/', errorTitle="Node", errorMessage=str(nodeForm.errors))
        
        # Validate Param form
        paramFormSet = ParamFormSet(request.POST, request.FILES, instance=node, prefix='param')
        
        for paramForm in paramFormSet:
            paramForm.empty_permitted = False
        
        if not paramFormSet.is_valid():
            return error(request, nextPage='/add/', errorTitle="Parameter", errorMessage=str(paramFormSet.errors))
        
        # Validate Interface form
        interfaceFormSet = InterfaceFormSet(request.POST, request.FILES, instance=package, prefix='interface')
        
        for interfaceForm in interfaceFormSet:
            interfaceForm.empty_permitted = False
        
        if not interfaceFormSet.is_valid():
            return error(request, nextPage='/add/', errorTitle="Interface", errorMessage=str(interfaceFormSet.errors))
        
        # Everything ok
        package.save()
        node.save()
        paramFormSet.save()
        interfaceFormSet.save()
        
        return success(request)
    
    # Package Form
    packageForm = PackageForm(prefix='package')
    root_path = request.user.get_profile().path
    pkgs = (('', 'Select...'),) + tuple((pkg, pkg) for pkg in roslib.packages.list_pkgs_by_path(root_path))
    packageForm.register(pkgs)
    
    # Node Form
    nodeForm = NodeForm(prefix='node')
    
    # Parameter Form
    paramFormSet = ParamFormSet(prefix='param', queryset=Node.objects.none())
    
    # Interface Form
    interfaceFormSet = InterfaceFormSet(prefix='interface', queryset=Package.objects.none())
    
    return render_to_response('add.html', webpage_values(request, { 'packageForm' : packageForm, 
                                                                    'nodeForm' : nodeForm,
                                                                    'paramForm' : paramFormSet,
                                                                    'interfaceForm' : interfaceFormSet }))

def feedService(request):
    """ Used by javascript function to get available services in specified package """
    if not request.user.is_authenticated():
        return HttpResponseRedirect("/login/")
    
    try:
        pkg = request.GET['package']
        interfaceType = request.GET['msgType']
    except KeyError:
        return HttpResponseBadRequest()
    
    try:
        pkg = roslib.packages.get_pkg_dir(pkg, False)
    except roslib.packages.InvalidROSPkgException:
        return HttpResponseBadRequest()
    
    if interfaceType != 'srv':
        interfaceType = 'msg'
    
    interfaces = (os.path.basename(path)[:-4] for path in glob.glob(os.path.join(pkg, '{0}/*.{0}'.format(interfaceType))))
    content = '\n'.join(['Select...\t']+['{0}\t{0}'.format(interface) for interface in interfaces])
    
    return HttpResponse(content, content_type='text/plain')

def feedNode(request):
    """ Used by javascript function to get available executables/nodes in specified package """
    if not request.user.is_authenticated():
        return HttpResponseRedirect("/login/")
    
    try:
        pkg = request.GET['package']
    except KeyError:
        return HttpResponseBadRequest()
    
    root_path = request.user.get_profile().path
    
    if root_path[-1:] != '/':
        root_path += '/'
    
    len_root_path = len(root_path)
    
    try:
        pkg = roslib.packages.get_pkg_dir(pkg)
    except roslib.packages.InvalidROSPkgException:
        return HttpResponseBadRequest()
    
    exes = []
    
    for p, dirs, files in os.walk(pkg):
        for file in files:
            test_path = os.path.join(p, file)
            s = os.stat(test_path)
            if (s.st_mode & (stat.S_IRUSR | stat.S_IXUSR) == (stat.S_IRUSR | stat.S_IXUSR)) and test_path.startswith(root_path):
                exes.append(test_path[len_root_path:])
        if '.svn' in dirs:
            dirs.remove('.svn')
        elif '.git' in dirs:
            dirs.remove('.git')
        elif 'build' in dirs:
            dirs.remove('build')
    
    content = '\n'.join(['Select...\t']+['{0}\t{1}'.format(exe, os.path.basename(exe)) for exe in exes])
    
    return HttpResponse(content, content_type='text/plain')
