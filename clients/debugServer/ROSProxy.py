"""This module provides a class that allows for (slightly) easier dynamic access to ROS"""

"""Original code from http://www.ros.org/wiki/rosbridge package
    modified to fit RoboEarth Cloud Engine needs"""

import roslib; roslib.load_manifest('rospy')
import rospy
roslib.load_manifest('rosservice')
import roslib.rostime
import rosservice
import re 

from base64 import standard_b64encode
from types import TypeType

class ROSProxy(object):
	def __init__(self):
		self.mans = {}
		self.mods = {}

	def __classFromTypeString(self, typeString, subname):
		basemodule, itype = typeString.split('/')

		if not (basemodule in self.mans):
			try:
				roslib.load_manifest(basemodule)	
				self.mans[basemodule] = True;
			except:
				print "Can't find class for %s" % (typeString,)
				return None

		modname = basemodule + '.'+ subname + '._' + itype
		if not (modname in self.mods):
			try:
				mod = __import__(modname)
				self.mods['modname'] = mod
			except:
				return None

		mod = self.mods['modname']

		return getattr(getattr(getattr(mod,subname),'_' + itype), itype)

	def msgClassFromTypeString(self, typeString):
		return self.__classFromTypeString(typeString, 'msg')

	def srvClassFromTypeString(self, typeString):
		return self.__classFromTypeString(typeString, 'srv')

	def callService(self, service, arguments, callback=False, wait=True):
		def defCallback(x):
			pass

		if callback == False:
			callback = defCallback

		if wait:
			try:
				rospy.wait_for_service(service)
			except:
				callback(None)
				raise
		try:
			function = rospy.ServiceProxy(service, self.classFromService(service))
			if isinstance(arguments, list):
				response = function(*arguments)
			else:
				response = function(arguments)
			callback(response)
		except:
			callback(None)
			raise

	def generalize(self, inst):
		if hasattr(inst,'__slots__'):
			obj = {}
			for i in xrange(len(inst.__slots__)):
				field = inst.__slots__[i]
				if (hasattr(inst,'_slot_types') and inst._slot_types[i] == 'uint8[]'):
					obj[field] = self.generalize(standard_b64encode(getattr(inst,field)))
				else:
					obj[field] = self.generalize(getattr(inst,field))
			return obj
		elif isinstance(inst,tuple) or isinstance(inst,list):
			return [self.generalize(x) for x in inst]
		else:
			return inst

	braces = re.compile(r'\[[^\]]*\]') 
	atomics = ['bool', 'byte','int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64', 'float32', 'float64', 'string']

	def specify(self, typeStr, obj):
		if isinstance(typeStr,TypeType):
			cls = typeStr
		elif isinstance(typeStr,list):
			lst = []
			for i in xrange(len(typeStr)):
				lst.append(self.specify(typeStr[i],obj[i]))
			return lst
		elif typeStr != self.braces.sub('',typeStr):
			return [self.specify(self.braces.sub('',typeStr), x) for x in obj]
		elif typeStr in self.atomics:
			if typeStr == 'string':
				return obj.encode('ascii','ignore')
			return obj
		elif typeStr == 'time' or typeStr == 'duration':
			inst = None
			if typeStr == 'time':
				inst = roslib.rostime.Time()
			else:
				inst = roslib.rostime.Duration()
			inst.nsecs = obj['nsecs']
			inst.secs = obj['secs']
			return inst
		else:
			if typeStr == 'Header':
				typeStr = 'std_msgs/Header'
			cls = self.msgClassFromTypeString(typeStr)

		if not hasattr(cls,'__call__'):
			return None
		inst = cls()
		for i in xrange(len(cls._slot_types)):
			field = cls.__slots__[i]
			typ = cls._slot_types[i]
			if field in obj:
				value = self.specify(typ,obj[field])
				if value != None:
					setattr(inst,field,value)
				else:
					print "Warning: passed object was only partially specified."
		return inst