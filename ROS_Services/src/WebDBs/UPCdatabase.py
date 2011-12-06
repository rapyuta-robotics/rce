#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       UPCdatabase.py
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
#		Note:	This WebDB interface uses 'upcdatabase.com'.
#					-	http://www.upcdatabase.com

from __future__ import print_function

from WebDBBase import WebDBBase, WebDBError
from xmlrpclib import ServerProxy, Error

class UPCdatabase(WebDBBase):
	""" A WebDB interfaces which uses upcdatabase.com.
	"""
	
	CONF_FIELDS=['rpc_key']
	_RPC_KEY=None
	_MAPPER={'description' : 'description',
			 'ean' : 'ean13',
			 'issuerCountry' : 'origin',
			 'issuerCountryCode' : 'originCode',
			 'lastModifiedUTC' : 'modified',
			 'size' : 'size'}
	
	def run(self):
		""" Overwrites the necessary function from the base class.
			This method is used to run a request on the WebDB
			upcdatabase.com.
		"""
		# run request to WebDB upcdatabase.com
		response=self._query()
		
		if response is None:
			return
		
		# format and append received data
		self.appendEntry(self._format(response))
	
	def _query(self):
		""" This method handles the request to the Web and returns a
			HTMLResponse object on success and None if the request failed.
		"""
		params={ 'rpc_key': UPCdatabase._RPC_KEY, 'ean' : self.gtin }
		
		try:
			server=ServerProxy('http://www.upcdatabase.com/xmlrpc')
			response=server.lookup(params)
		except Error as e:
			raise WebDBError(e)
		
		# preprocess received data
		status=response.pop('status')
		
		if status == 'fail':
			self.debug('Query of UPCdatabase failed. Error message: {0}'.format(status))
			return None
		elif status != 'success':
			self.debug("Unhandled status '{0}' encountered in response of UPCdatabase".format(status))
			return None
		
		if response.pop('found'):
			try:
				del(response['message'])
				del(response['noCacheAfterUTC'])
				del(response['pendingUpdates'])
			except KeyError:
				self.debug('Tried to remove a key in an UPCdatabase entry which does not exsist.')
		else:
			return None
		
		return response
	
	def _format(self, response):
		""" This method maps the received data in the HTMLResponse
			object to the convention of this service.
		"""
		formatedResponse={}
		
		for key in response:
			if key in UPCdatabase._MAPPER:
				if isinstance(response[key], unicode):
					formatedResponse[UPCdatabase._MAPPER[key]]=response[key].encode('utf-8')
				else:
					formatedResponse[UPCdatabase._MAPPER[key]]=response[key]
			else:
				self.debug("Key '{0}' of an UPCdatabase entry ignored.".format(key))
		
		formatedResponse['language']='en'
		
		return formatedResponse
