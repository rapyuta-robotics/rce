#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       Service.py
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

from piston.handler import AnonymousBaseHandler
from piston.utils import rc

from django.http import HttpResponse

KEY = 'HC2s8q9aUXop1WnKYn4zJxCWUVimGLiOC0r9hUxRWSO8wtStUp'
VALID_USERNAMES = ['admin', 'user']

class LoginHandler(AnonymousBaseHandler):
    allowed_methods = ('POST',)

    def create(self, request):
        if request.POST.get('key', '') != KEY:
            return rc.BAD_REQUEST

        username = request.POST.get('username', '')
        password = request.POST.get('password', '')

        if username in VALID_USERNAMES and username == password:
            return rc.ALL_OK

        return rc.FORBIDDEN
