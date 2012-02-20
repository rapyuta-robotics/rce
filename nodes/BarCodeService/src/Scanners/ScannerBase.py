#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       ScannerBase.py
#       
#       Copyright 2011 dominiquehunziker <dominique.hunziker@gmail.com>
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

from __future__ import print_function

from Worker import ObjectWorker

class ScannerError(Exception):
    """ General Exception in connection with a Scanner.
    """
    pass

class ScannerBase(ObjectWorker):
    """ Generic base class for all barcode scanners. You should not
        instantiate this class directly.
        
        The keyworded argument data of the constructor should contain
        a dictionary with an entry 'image' which is again a dictionary
        with the keys 'width', 'height' and 'raw'.
    """
    
    DATA_FIELDS={ 'image' : None }
    
    def run(self):
        """ Overwrites the necessary method from the base class.
            This method scans for a barcode by calling the method scan
            which has to be overwitten.
        """
        try:
            self.scan()
        except ScannerError as e:
            error(e)
            return
    
    def scan(self):
        """ Overwrite this method, else a NotImplementedError is raised.
            This method is used to run a scan an image.
        """
        raise NotImplementedError('The method scan for the used Scanner is not implemented.')
