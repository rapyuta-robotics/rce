#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       UPCdata.py
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
#       Note:   This WebDB interface uses 'upcdata.info'.
#                   -   http://www.upcdata.info

from __future__ import print_function

from WebDBBase import WebDBBase, WebDBError
from urllib import urlencode
from urllib2 import Request, URLError, urlopen
import json
from datetime import datetime

class UPCdata(WebDBBase):
    """ A WebDB interfaces which uses upcdata.info.
    """
    CONF_FIELDS=['keycode']
    _KEYCODE=None
    _MAPPER={   'description' : 'description',
                'ean13' : 'ean13',
                'image' : 'images',
                'modified' : 'modified',
                'product' : 'product',
                'upca' : 'upc12',
                'upce' : 'upc8'}
    
    def queryDB(self):
        """ Overwrites the necessary method from the base class.
            This method is used to run a request on the WebDB
            upcdata.info.
        """
        # run request to WebDB upcdata.info
        # format and append received data
        self.appendEntry(self._format(self._query()))
    
    def _query(self):
        """ This method handles the request to the Web and returns a
            HTMLResponse object on success and None if the request failed.
        """
        params=urlencode({ 'keycode' : UPCdata._KEYCODE, 'mode' : 'json', 'find' : self.gtin })
        req=Request('http://www.upcdata.info/feed.php?'+params, '', WebDBBase.HEADERS)
        
        try:
            response=urlopen(req)
        except URLError as e:
            raise WebDBError(e)
        
        return response
    
    def _format(self, response):
        """ This method maps the received data in the HTMLResponse
            object to the convention of this service.
        """
        response=json.load(response)
        
        if response['status']['code'] != '200':
            return None
        
        # convert time stamp to datetime object
        response=response['product']
        response['modified']=datetime.strptime(response['modified'], '%Y-%m-%d %H:%M:%S')
        
        # extract links for the images
        if len(response['image']) != 0:
            if isinstance(response['image'], unicode):
                response['image']=[response['image'].encode('utf-8')]
            else:
                response['image']=[response['image']]
        
        # format data
        formatedResponse={}
        
        if 'company' in response:
            if isinstance(response['company']['name'], unicode):
                formatedResponse['company']=response['company']['name'].encode('utf-8')
            else:
                formatedResponse['company']=response['company']['name']
        
        for key in response:
            if key in UPCdata._MAPPER:
                if isinstance(response[key], unicode):
                    formatedResponse[UPCdata._MAPPER[key]]=response[key].encode('utf-8')
                else:
                    formatedResponse[UPCdata._MAPPER[key]]=response[key]
            else:
                self.debug("Key '{0}' of an UPCdata entry ignored.".format(key))
        
        for key in ['upca', 'upce']:
            if response[key] == 'n/a':
                del(formatedResponse[UPCdata._MAPPER[key]])
        
        formatedResponse['language']='en'
        
        return formatedResponse

#   Available keys in a database entry:
#   
#       status
#           find        the find value you sent in your request
#           code        200 = success, 404 = not found, 400 and 500 = error
#           message         details about the code
#       product
#           modified        YYYY-MM-DD HH:MM:SS (24 hour time)
#           ean13       13 digit EAN-13 code
#           upca        12 digit UPC-A code (may be n/a)
#           upce        8 digit UPC-E code (may be n/a)
#           product         the name of the product
#           description         size, weight, count, color, etc.
#           category_no         numeric category code
#           category_text       text version of the category
#           url         a link to the product page (not on our site)
#           image       a link to an image of the product (on our site)
#           barcode         a link to a barcode image (on our site)
#           locked      is this data locked from updates (1/0)
#       company
#           name        company name
#           logo        a link to a company logo (on our site)
#           url         a link to the company's official web site
#           address         the company's mailing address
#           phone       the company's phone number (digits only)
#           locked      is this data locked from updates (1/0)
