#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       WebDBBase.py
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

from Worker import ThreadWorker

class WebDBError(Exception):
    """ General Exception in connection with a WebDB.
    """
    pass

class WebDBBase(ThreadWorker):
    """ Generic base class for all WebDB interfaces. You should not
        instantiate this class directly.

        WebDB is a subclass of Thread. Therefore after instantiating
        a WebDB it has to be started by calling task.start().

        The keyworded argument data of the constructor should contain
        a dictionary with a key 'gtin' and as value a 13-digit string.

        Valid keys for a response from a WebDB are:
        product             Name the product
        type                Type of the product
        description         Description of the product
        additional_info     Additional infos of the product
        size                Size / Quantity of the product
        images              list of http-links to images of the prduct

        origin              Country from where the barcode is
        originCode          Code for country from where the barcode is,
        e.g. ('us', 'de')
        company             Company to which the product behind the
        barcode belongs
        company_alternate   Company to which the barcode belongs

        upc8                Barcode in the UPC-E format (8 digits)
        upc12               Barcode in the UPC-A format (12 digits)
        ean13               Barcode in the EAN-13 format

        language            Language of the entry
        added               datetime object containing the time the entry
        was added to the DB
        modified            datetime object containing the time the entry
        was last modified in the DB
        dbclass             __name__ of the used WebDB to obtain the
        data
    """
    DATA_FIELDS = { 'gtin' : None }
    HEADERS = {   #'User-Agent' : 'Mozilla/5.0 (X11; Linux x86_64; rv:7.0.1) Gecko/20100101 Firefox/7.0.1',
                'Content-type' : 'application/x-www-form-urlencoded',
                'Accept' : 'text/plain' }

    def run(self):
        """ Do not overwrite this method. This method is automatically
            called, when the Thread is started. It will handle the request
            by calling the method queryDB which has to be overwritten.
        """
        for field in self.__class__.CONF_FIELDS:
            if field is None:
                self.error('{0} needs to be configured before it can be accessed.'.format(self.__class__.__name__))
                return

    #    if len(self.gtin) != 13:
    #        self.error('The length of the given GTIN is not equal to 13.')
    #        return

        try:
            self.queryDB()
        except WebDBError as e:
            self.error(e)
            return
        except Exception as e:
            self.fatal(e)
            return

    def queryDB(self):
        """ Overwrite this method, else a NotImplementedError is raised.
            This method is used to run a request on a WebDB.
        """
        raise NotImplementedError('The method queryDB for the used WebDB is not implemented.')

    def appendEntry(self, entry):
        """ This method allows to append a WebDBEntry to the collected
            data for the request to the WebDBs. The argument should be
            a dictionary containing only keys which are in the definition
            above.
        """
        if entry is None or not isinstance(entry, dict):
            return

        self.info('Entry found in {0}.'.format(self.__class__.__name__))

        entry['dbclass'] = self.__class__.__name__
        self.appendData(entry)

    def formatMsg(self, msg, func):
        """ If the method is overwitten make sure to check whether the
            given func is None or not.
        """
        if func is not None:
            func('[{0} - {1}] {2}'.format(self.__class__.__name__, self.gtin, msg))
