#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       %Service%.py
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

import roslib; roslib.load_manifest('%Package%')
import %Package%.srv

import convert
from initNode import initNode, Request

class %Service%Request(Request):
    
    def fuseData(self):
        ###
        ### Add your code here.
        ###
        ### The collected data from the Workers can be found in self._rawData
        ### which is a of type list.
        ###
        
        self._result=%Package%.srv.Query%Service%Response( ### Edit ### )

if __name__ == '__main__':
    initNode('%Service%', %Service%Request)
