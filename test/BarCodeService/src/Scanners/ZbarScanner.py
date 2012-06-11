#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       ZbarScanner.py
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
#
#       Note:   This barcode scanner uses the 'zbar bar code reader' library
#               and its python wrapper, which are available under the GNU
#               Lesser General Public License, version 2.1.
#               The sourcecode is available under:
#                   -   http://zbar.sourceforge.net/

from __future__ import print_function

from ScannerBase import ScannerBase, ScannerError

# Import zbar (barcode reader library)
try:
    import zbar
except ImportError:
    print('Requires zbar.\n')
    exit(0)

class ZbarScanner(ScannerBase):
    """ A barcode scanner which uses the 'zbar bar code reader' library.
    """

    def run(self):
        """ Overwrites the necessary function from the base class.
            This method scans for a barcode.
        """
        # wrap image data
        image = self.image.convert('L')

        width, height = image.size
        raw = image.tostring()

        image = zbar.Image(width, height, 'Y800', raw)

        # create a reader
        scanner = zbar.ImageScanner()

        # configure the reader
        scanner.parse_config('enable')

        # scan the image for barcodes
        scanner.scan(image)

        # print and add found barcodes
        for symbol in image:
            self.info('Found: {0} - {1}'.format(symbol.type, symbol.data))
            self.appendData((symbol.type, symbol.data))
