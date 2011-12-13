#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       initNode.py
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

from __future__ import print_function

# ROS specific imports
import roslib; roslib.load_manifest('ROS_Services')
import ROS_Services.srv
import rospy

# Request specific imports
import convert
from Worker import WorkerError
from threading import Lock, Thread

# init specific imports
from string import ascii_letters
import json

class RequestError(Exception):
	""" General Exception in connection with a Request.
	"""
	pass

class Request(object):
	""" Generic base class which handles a request to a ROS service. This
		class should be used in conjunction with the Worker classes
		Worker.ThreadWorker and Worker.ObjectWorker.
	"""
	_TYPE=None
	_WORKER=[]
	
	def __init__(self):
		""" Use the run method and not this constructor directly, if you
			wish to use this class. Make sure that this constructor is
			called, if you create a new constructor for the subclass. """
		self._rawData=[]
		self._result=None
		self._lock=Lock()
	
	@classmethod
	def run(cls, requestObject):
		""" Do not overwrite this function. This function is automatically
			called, when a new request comes in from the service. It will
			handle the request according to the configuration and returns
			a Response object according to the method fuseData which has
			to be overwritten, else an NotImplementedError is raised.
		"""
		self=cls()
		
		# convert requestObject to python dictionary
		input=convert.ROSToPyDict(requestObject)
		
		# setup the workers
		workerList=[]
		for Worker in Request._WORKER:
			try:
				workerList.append(Worker(	data=input,
											target=self.callback,
											debug=rospy.logdebug,
											info=rospy.loginfo,
											warning=rospy.logwarn,
											error=rospy.logerr,
											fatal=rospy.logfatal))
			except WorkerError as e:
				rospy.logfatal(e)
				return None
		
		# run the workers depending on type parallel or serial
		if isinstance(Request._TYPE, Thread):
			for thread in workerList:
				thread.start()
		
			for thread in workerList:
				thread.join()
		else:
			for worker in workerList:
				worker.run()
		
		# now prepare the collected data to be returned to ROS service
		self.fuseData()
		
		if self._result is None:
			raise RequestError('Could not fuse the data.')
		
		return self._result
	
	def fuseData(self):
		""" Overwrite this method, else a NotImplementedError is raised.
			This function is used to fuse the collected data from the
			workers into a correct ROS response object.
			The collected data is available as a list in self._rawData.
			The result has to be saved in self._result.
		"""
		raise NotImplementedError('The method run for the used WebDB is not implemented.')
	
	def callback(self, data):
		""" This method is used to provide a threadsafe method to collect
			the returned data from the workers.
		"""
		with self._lock:
			self._rawData.append(data)
	
	@staticmethod
	def setWorkers(workers):
		""" This method allows to configure which workers should be used
			to process this request. The argument should be a list of
			subclasses of either Worker.ThreadWorker or Worker.ObjectWorker.
		"""
		Request._TYPE=type(workers[0])
		
		for worker in workers:
			if not isinstance(worker, Request._TYPE):
				raise RequestError('All workers have to be of the same base class.')
		
		Request._WORKER=workers

def initNode(name, requestClass):
	""" Initializes the node '{name}Server'. The argument 'name' is the
		name used for the Service. The name must be a string and can only
		contain characters in [A-Z] and [a-z]. The argument requestClass
		is the class which is used to handle the request to the service
		and has to be a subclass of the class 'Request'.
		
		For illustration  two examples:
			name of ...			name = foo			name = BAR
			---------------------------------------------------------
			node				FooServer			BARServer
			configFile			foo.conf			bAR.conf
			package				Foos				BARs
			service				Foo					BAR
			service definiton	QueryFoo.srv		QueryBAR.srv
	"""
	if not isinstance(name, str):
		rospy.logfatal('Given name for initialization of a node is not a string.')
		return
	
	for char in name:
		if char not in ascii_letters:
			rospy.logfatal("Given name contains invalid character '{0}'.".format(char))
			return
	
	lowerName=name[0:1].lower()+name[1:]
	upperName=name[0:1].upper()+name[1:]
	
	rospy.init_node('{0}Server'.format(upperName))
	
	try:
		filePath=roslib.packages.find_resource('ROS_Services', '{0}.conf'.format(lowerName))[0]
	except IndexError:
		rospy.logfatal('Could not find configuration file.')
		return
	
	# get configuration
	with open(filePath) as f:
		config=json.load(f)
	
	workerList=[]
	
	# load all workers which are in the 'lowerName.conf' file
	for db in config:
		try:
			mod=__import__('{0}s.{1}'.format(upperName, db), fromlist=[db])
			worker=getattr(mod, db)
		except ImportError, AttributeError:
			rospy.logfatal('Could not find specified {0} {1}.'.format(upperName, db))
		else:
			workerList.append(worker)
	
	# configure all workers
	for worker in workerList[:]:
		try:
			worker.configure(config[worker.__name__])
		except WorkerError as e:
			rospy.logerror(e)
			workerList.remove(woker)
	
	if len(workerList) == 0:
		rospy.logfatal('There are no {0}s to use.'.format(upperName))
		return
	
	# add the workers to the Request subclass which will handle the requests
	try:
		requestClass.setWorkers(workerList)
	except RequestError as e:
		rospy.logfatal(e)
		return
	
	# now launch ROS Service 'upperName'
	QueryWork=getattr(ROS_Services.srv, 'Query{0}'.format(upperName))
	s=rospy.Service(upperName, QueryWork, requestClass.run)
	rospy.spin()
