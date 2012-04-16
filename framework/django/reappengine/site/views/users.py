#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       users.py
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

from django.http import HttpResponse, HttpResponseRedirect
from django.contrib import auth
from django.shortcuts import render_to_response

from reappengine.site.site import webpage_values

########################################################################
# login / logout

def loginForm(request, error=False):
    """ show login form """
    if request.user.is_authenticated():
        return HttpResponseRedirect('/')
    
    values = {'error' : error}
    
    return render_to_response('login.html', webpage_values(request,values))

def login(request):
    """ authenticate a user. The user has to provide his user name and password """
    try:
        username = request.POST['username']
        password = request.POST['password']
    except KeyError:
        return HttpResponse(loginForm(request, error=True))
    
    user = auth.authenticate(username=username, password=password)
    
    if user is not None and user.is_active:
        # Correct password, and the user is marked "active"
        auth.login(request, user)
        # Redirect to a success page.
        return HttpResponseRedirect('/')
    else:
        # Show an error page
        return HttpResponse(loginForm(request, error=True))

def logout(request):
    """ user logout """
    auth.logout(request)
    return HttpResponseRedirect('/')

########################################################################
# start page

def index(request):
    """ show start page """
    return render_to_response('index.html', webpage_values(request,{}))
