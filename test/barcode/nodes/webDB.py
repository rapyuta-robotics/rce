#!/usr/bin/env python
# -*- coding: utf-8 -*-
#     
#     webDB.py
#     
#     This file is part of the RoboEarth Cloud Engine test.
#     
#     This file was originally created for RoboEearth
#     http://www.roboearth.org/
#     
#     The research leading to these results has received funding from
#     the European Union Seventh Framework Programme FP7/2007-2013 under
#     grant agreement no248942 RoboEarth.
#     
#     Copyright 2012 RoboEarth
#     
#     Licensed under the Apache License, Version 2.0 (the "License");
#     you may not use this file except in compliance with the License.
#     You may obtain a copy of the License at
#     
#     http://www.apache.org/licenses/LICENSE-2.0
#     
#     Unless required by applicable law or agreed to in writing, software
#     distributed under the License is distributed on an "AS IS" BASIS,
#     WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#     See the License for the specific language governing permissions and
#     limitations under the License.
#     
#     \author/s: Dominique Hunziker 
#
#     Note:   This WebDB interface uses google's API python client, which is
#             available under the Apache License, Version 2.0.
#                 http://code.google.com/p/google-api-python-client/
#

import os.path
import time
from ConfigParser import RawConfigParser

import roslib; roslib.load_manifest('barcode')
import rospy

# Import google API
try:
    from apiclient.discovery import build
except ImportError:
    print('Requires google API.\n')
    exit(0)

from barcode.msg import Barcode, DBEntry, DBInfo


class WebDBError(Exception):
    """ Raised if there is a problem with a WebDB query.
    """


class Google(object):
    _COUNTRIES = ['US', 'GB']
    _SHOPPING_API_VERSION = 'v1'
    _MAP = {'brand' : 'company', 'creationTime' : 'added',
            'description' : 'description', 'gtin' : 'ean13',
            'images' : 'images', 'language' : 'language',
            'modificationTime' : 'modified', 'title' : 'product'}
    
    def __init__(self, key, pub):
        """ Initialize the Google shopping database handler.
            
            @param key:     Google API key which is used for queries.
            @type  key:     str
            
            @param pub:     Publisher where the query results should be
                            published.
            @type  pub:     rospy.Publisher
        """
        self._key = key
        self._pub = pub
    
    def query(self, req):
        """ Query the Google shopping database for the given GTIN.
        """
        rospy.loginfo('Received request {}'.format(req))
        resp = []
        
        for country in Google._COUNTRIES:
            # run request to WebDB google's shopping API
            response = self._query(country, req.gtin)
            
            rospy.loginfo('{0} items found in Google '
                          '{1}.'.format(response['totalItems'], country))
            
            if response['totalItems'] == 0:
                continue
            
            # process each entry
            for entry in response['items']:
                resp.append(self._format(entry['product'], req.gtin))
        
        self._pub.publish(req, resp)
    
    def _query(self, country, gtin):
        """ Method is used internally to query a barcode for a given country.
        """
        client = build('shopping', Google._SHOPPING_API_VERSION,
                       developerKey=self._key)
        resource = client.products()
        request = resource.list(source='public', country=country, q=gtin)
        return request.execute()
    
    def _format(self, entry, gtin):
        """ Method is used internally to map received data to response message.
        """
        # modify GTIN
        if 'gtin' not in entry:
            rospy.logwarn('No GTIN in Google entry. '
                          'Used GTIN from DB query instead.')
            
            entry['gtin'] = gtin
        else:
            googleGTIN = entry['gtin']
            
            if googleGTIN[0] == '0' and len(googleGTIN) == 14:
                googleGTIN = googleGTIN[1:]
                
                if isinstance(googleGTIN, unicode):
                        googleGTIN = googleGTIN.encode('utf-8')
                
                entry['gtin'] = googleGTIN
            else:
                raise WebDBError('GTIN in Google entry does not match '
                                 'the expected pattern.')
        
        # extract links for the images
        images = []
        
        if 'images' in entry:
            for img in entry['images']:
                link = img['link']
                
                if isinstance(link, unicode):
                    link = link.encode('utf-8')
                
                images.append(link)
            
            entry['images'] = images
        
        # modify time stamps to ROS time
        for key in ['creationTime', 'modificationTime']:
            # Need to correct seconds since epoch('t'), because time.mktime only
            # is available for local timezone, but timestamp is UTC
            dst = time.localtime().tm_isdst*3600
            t = time.mktime(time.strptime(entry[key][:-5], '%Y-%m-%dT%H:%M:%S'))
            entry[key] = rospy.Time.from_sec(t-time.timezone+dst)
        
        # fill ROS message
        resp = DBEntry()
        
        for key, value in entry.iteritems():
            if key in Google._MAP:
                if isinstance(value, unicode):
                    value = value.encode('utf-8')
                
                setattr(resp, Google._MAP[key], value)
            else:
                rospy.logdebug("Key '{0}' of Google entry ignored.".format(key))
        
        return resp


def webDB_server():
    rospy.init_node('webDBNode')
    
    path = os.path.join(roslib.packages.get_pkg_dir('barcode'), 'barcode.cfg')
    config = RawConfigParser()
    config.read(path)
    key = config.get('webDB', 'key')
    
    g = Google(key, rospy.Publisher('barcodeDBInfo', DBInfo))
    rospy.Subscriber('barcodeWebDBQuery', Barcode, g.query)
    rospy.spin()


if __name__ == '__main__':
    webDB_server()
