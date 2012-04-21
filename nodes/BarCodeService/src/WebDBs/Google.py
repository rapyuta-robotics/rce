#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       Google.py
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
#       Note:   This WebDB interface uses google's shopping API.
#                   -   code.google.com/apis/shopping/search

from __future__ import print_function

from WebDBBase import WebDBBase, WebDBError
from datetime import datetime

# Import google API
try:
    from apiclient.discovery import build
except ImportError:
    print('Requires google API.\n')
    exit(0)

class Google(WebDBBase):
    """ A WebDB interfaces which uses google's shopping API.
    """
    CONF_FIELDS = ['key', 'countries']
    _KEY = None
    _COUNTRIES = None
    _SHOPPING_API_VERSION = 'v1'
    _MAPPER = {   'brand' : 'company',
                'creationTime' : 'added',
                'description' : 'description',
                'gtin' : 'ean13',
                'images' : 'images',
                'language' : 'language',
                'modificationTime' : 'modified',
                'title' : 'product'}

    def queryDB(self):
        """ Overwrites the necessary method from the base class.
            This method is used to run a request on google's
            shopping API.
        """
        for country in Google._COUNTRIES:
            # run request to WebDB google's shopping API
            response = self._query(country)

            self.info('{0} items found in Google {1}.'.format(response['totalItems'], country))

            if response['totalItems'] == 0:
                continue

            # process each entry
            for entry in response['items']:
                # format and append received data
                self.appendEntry(self._format(entry))

    def _query(self, country):
        """ This method handles the request to the Web and returns a
            Dictionary on success and None if the request failed.
        """
        client = build('shopping', Google._SHOPPING_API_VERSION, developerKey=Google._KEY)
        resource = client.products()
        request = resource.list(source='public', country=country, q=self.gtin)
        response = request.execute()

        return response

    def _format(self, response):
        """ This method maps the received data in the Dictionary to
            the convention of this service.
        """
        response = response['product']

        # convert time stamp to datetime object
        for key in ['creationTime', 'modificationTime']:
            response[key] = datetime.strptime(response[key][:-5], '%Y-%m-%dT%H:%M:%S')

        # modify GTIN to EAN-13
        if 'gtin' not in response:
            self.warning('No GTIN in Google entry. Used GTIN for DB query instead.')

            response['gtin'] = self.gtin
        else:
            if response['gtin'][0] == '0' and len(response['gtin']) == 14:
                if isinstance(response['gtin'], unicode):
                    response['gtin'] = response['gtin'][1:].encode('utf-8')
                else:
                    response['gtin'] = response['gtin'][1:]
            else:
                raise WebDBError('GTIN in Google entry does not match the expected pattern.')

        # extract links for the images
        images = []

        if 'images' in response:
            for img in response['images']:
                if isinstance(img['link'], unicode):
                    images.append(img['link'].encode('utf-8'))
                else:
                    images.append(img['link'])

            response['images'] = images

        # format data
        formatedResponse = {}

        for key in response:
            if key in Google._MAPPER:
                if isinstance(response[key], unicode):
                    formatedResponse[Google._MAPPER[key]] = response[key].encode('utf-8')
                else:
                    formatedResponse[Google._MAPPER[key]] = response[key]
            else:
                self.debug("Key '{0}' of a Google entry ignored.".format(key))

        return formatedResponse
