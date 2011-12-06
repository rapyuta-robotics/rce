#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       Codecheck.py
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
#		Note:	This WebDB interface uses 'codecheck.info'.
#					-	http://www.codecheck.info

from __future__ import print_function

# Import general
from WebDBBase import WebDBBase, WebDBError

# Import Codecheck
from urllib import urlencode
from urllib2 import Request, URLError, urlopen

# Import CodecheckParser
from datetime import datetime
from re import compile as reCompile
from string import joinfields
from HTMLParser import HTMLParser, HTMLParseError

class Codecheck(WebDBBase):
	""" A WebDB interfaces which uses codecheck.info.
	"""
	
	def queryDB(self):
		""" Overwrites the necessary method from the base class.
			This method is used to run a request on the WebDB
			codecheck.info.
		"""
		# run request to WebDB codecheck.info
		# preprocess, format and append received data
		self.appendEntry(CodecheckParser.parse(self._query(), self))
	
	def _query(self):
		""" This method handles the request to the Web and returns a
			HTMLResponse object on success and None if the request failed.
		"""
		params=urlencode({	'formids' : 'If_2_0,searchText2,If_3_0,If_7_0',
							'seedids' : '',
							'submitmode' : '',
							'submitname' : '',
							'If_2_0' : 'F',
							'If_3_0' : 'F',
							'If_7_0' : 'F',
							'searchText2' : self.gtin,
							'OK2' : 'Suchen' })
		req=Request('http://www.codecheck.info/Home/search/sdirect.pag?'+params, '', WebDBBase.HEADERS)
		
		try:
			response=urlopen(req)
		except URLError as e:
			self.error(e)
		
		return response

class CodecheckParser(HTMLParser):
	""" This class is used to parse the received HTMLResponse object (or
		any file like object which provides a read() function, which
		returns the html code as a string) from codecheck.info.
		Usage:
			>>> data = CodecheckParser.parse('response object')
	"""
	
	_MAPPER={'EAN-Nummer' : 'ean13',
			 'Erfasst' : 'added',
			 'Herkunft' : 'origin',
			 'Hersteller / Vertrieb' : 'company',
			 'Hersteller / Vertrieb (Strichcodeanmelder GS1 / EAN)' : 'company_alternate',
			 'Inhaltsstoffe / tech. Angaben' : 'description',
			 'Letzte Änderung' : 'modified',
			 'Menge / Grösse' : 'size',
			 'Zusatzinformationen' : 'additional_info'}
	_RE=reCompile('[0-3][0-9]. [0-1][0-9]. [0-9][0-9]')
	
	def __init__(self, codecheckInstance):
		""" Constructor for CodecheckParser. Do not use the constructor
			directly, but use the classmethod parse.
		"""
		HTMLParser.__init__(self)
		
		self._codecheckInstance=codecheckInstance
		
		self._active=False
		self._key=''
		self._value=''
		self.data={}
	
	def handle_starttag(self, tag, attrs):
		""" Inherited handler method of HTMLParser which needs to be
			overwritten.
		"""
		if tag == 'span':
			if self._active:
				raise HTMLParseError('Encountered a span starttag before the end of privious one was found.')
			else:
				self._active=True
			
			self._addData()
	
	def handle_endtag(self, tag):
		""" Inherited handler method of HTMLParser which needs to be
			overwritten.
		"""
		if tag == 'span':
			if not self._active:
				raise HTMLParseError('Encountered a span endtag before the start of the tag was found.')
			else:
				self._active=False
	
	def handle_data(self, data):
		""" Inherited handler method of HTMLParser which needs to be
			overwritten.
		"""
		data=data.strip()
		
		if self._active:
			self._key=joinfields([self._key, data], ' ')
		else:
			self._value=joinfields([self._value, data], ' ')
	
	def _addData(self):
		if self._key != '':
			self._key=self._key.strip()
			self._key=self._key.replace('\n', ' ')
			self._key=self._key.replace('\t', '')
			self._value=self._value.strip()
			
			if self._key in CodecheckParser._MAPPER:
				if isinstance(self._value, unicode):
					self.data[CodecheckParser._MAPPER[self._key]]=self._value.encode('utf-8')
				else:
					self.data[CodecheckParser._MAPPER[self._key]]=self._value
			else:
				self._codecheckInstance.debug("Key '{0}' of a Codecheck entry ignored.".format(self._key))
		
		self._key=''
		self._value=''
	
	def _convertDate(self):
		for key in ['added', 'modified']:
			if key in self.data:
				match=CodecheckParser._RE.search(self.data[key])
				self.data[key]=datetime.strptime(self.data[key][match.start(0):match.end(0)], '%d. %m. %y')
	
	@classmethod
	def parse(cls, response, codecheckInstance):
		""" Parses a received HTML file from codecheck.info and returns
			the gained data in form of a dictionary. The argument
			codecheckInstance is necessary for correct message handling.
		"""
		try:
			rawFeed=response.read()
		except AttributeError:
			raise WebDBError('Received object to parse is not response object.')
		try:
			rawFeed.index('<!-- Details zum Produkt -->')
		except ValueError:
			return None
		
		parser=cls(codecheckInstance)
		
		i=rawFeed.index('"Insert_2"')+len('"Insert_2">')
		
		if isinstance(rawFeed, unicode):
			parser.data['product']=rawFeed[i:rawFeed.index('<', i)].encode('utf-8')
		else:
			parser.data['product']=rawFeed[i:rawFeed.index('<', i)]
		
		i=rawFeed.index('"Insert_3"')+len('"Insert_3">')
		
		if isinstance(rawFeed, unicode):
			parser.data['type']=rawFeed[i:rawFeed.index('<', i)].encode('utf-8')
		else:
			parser.data['type']=rawFeed[i:rawFeed.index('<', i)]
		
		i=rawFeed.index('<!-- Details zum Produkt -->', i)+len(('<!-- Details zum Produkt -->'))
		imgFeed=rawFeed[:i]
		rawFeed=rawFeed[i:]
		
		i=imgFeed.rindex('<img')
		imgFeed=imgFeed[i:imgFeed.index('>', i)+1]
		i=imgFeed.index('src')+len('src="')
		imgFeed=imgFeed[i:imgFeed.index('"', i)]
		
		if len(imgFeed) != 0:
			if isinstance(imgFeed, unicode):
				parser.data['images']=['http://www.codecheck.info{0}'.format(imgFeed.encode('utf-8'))]
			else:
				parser.data['images']=['http://www.codecheck.info{0}'.format(imgFeed)]
		
		rawFeed=rawFeed[:rawFeed.index('Vollständigkeit:')]
		rawFeed=rawFeed[:rawFeed.rindex('<div')]
		rawFeed=rawFeed[:rawFeed.rindex('<div')]
		
		try:
			parser.feed(rawFeed)
			parser.close()
		except HTMLParseError as e:
			raise WebDBError('Could not parse the response object from codecheck.info: {0}'.format(e))
		
		parser._addData()
		parser._convertDate()
		
		parser.data['language']='de'
		
		return parser.data
