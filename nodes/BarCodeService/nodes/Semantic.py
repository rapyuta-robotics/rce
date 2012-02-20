#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       Semantic.py
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

import roslib; roslib.load_manifest('BarCodeService')
import BarCodeService.srv

from initNode import initNode, Request

class SemanticRequest(Request):
    """ This class is used to represent a request to the Semantic service.
    """
    
    def fuseData(self):
        """ Overwrites the necessary method from the base class.
            This method is used fuse the collected data from the workers
            into the return object QuerySemanticResponse.
        """
        self._result = BarCodeService.srv.QuerySemanticResponse(self._rawData[0])

if __name__ == '__main__':
    initNode('Semantic', SemanticRequest)
