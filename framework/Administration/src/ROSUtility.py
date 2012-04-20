#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       ROSUtility.py
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

# ROS specific imports
import roslib; roslib.load_manifest('Administration')
import rospy

# Python specific imports
import os

# Custom imports
import settings
from MessageUtility import InvalidRequest, InternalError
from ThreadUtility import Task
from MiscUtility import mktempfile
from ConverterBase import Converter
from ConverterUtility import resolveReference

class ParameterFactory(object):
    """ Abstract base class which represents and provides a factory method
        for the node configuration parameters.
    """
    def __init__(self, name, opt=False, default=None):
        """ Constructor which initializes the basic fields of a parameter.
            
            Should not used directly. Instead use the method createParameter(...)
            
            @param name:    Name of the parameter
            @type  name:    str
            
            @param opt:     Flag which indicates whether the configuration
                            parameter is optional or not
            @type  opt:     bool
            
            @param default: Default value for the case where the parameter
                            is optional
            
            @raise:     InternalError in case the opt flag is set but no
                        default value is given.
        """
        if opt and default is None:
            raise InternalError('Argument {0} is defined optional without a given default value.'.format(self.name))
        
        self.name = name
        self.opt = opt
        self.default = default
        self.value = None
        self.rosName = None
    
    def __del__(self):
        """ Destructor
        """
        if self.rosName:
            try:
                rospy.delete_param(self.rosName)
            except rospy.ROSException:
                pass
    
    def setParam(self, rosName):
        """ Set the parameter value to the ROS Parameter Server.
            
            @param rosName:     Name under which the parameter should be
                                saved.
            @type  rosName:     str
        """
        if hasattr(self, 'opt') or hasattr(self, 'default') or self.value is None:
            raise InternalError('Can not set the parameter before the configuration has been parsed.')
        
        try:
            if rospy.has_param(rosName):
                raise InternalError('Parameter already exists.')
            
            rospy.set_param(rosName, self.value)
        except rospy.ROSException as e:
            raise InternalError('ROS Parameter Server reported an error: {0}'.format(str(e)))
        
        self.rosName = rosName
    
    def parse(self, config, files):
        """ Parses the given configuration parameter.
            
            @param config:  Dictionary which contains all given parameters
                            as keys and their corresponding values as values.
            @type  config:  { str : ... }
            
            @param files:   Dictionary which contains the cStringIO.StringO
                            objects which contain the binary files sent with
                            the request.
            @type  files:   { str : cStringIO.StringO }
            
            @raise:     InvalidRequest in case of a parsing error or
                        InternalError.
        """
        # Check whether the parameter has already been parsed
        if not hasattr(self, 'opt') or not hasattr(self, 'default'):
            return
        
        # Try to load the value or the default value
        if self.name in config:
            self.value = config[self.name]
        elif self.opt:
            self.value = self.default
        
        # Make sure that there is a value
        if self.value is None:
            raise InvalidRequest('Parameter {0} is missing.'.format(self.name))
        
        # Validate value
        try:
            self._validate(files)
        except (InvalidRequest, InternalError):
            raise
        except Exception as e:
            raise InternalError('{0}: {1}'.format(e.__class__.__name__, str(e)))
        
        # Delete the opt flag and default value to signalize successful parsing
        del self.opt
        del self.default
    
    def _validate(self, files):
        """ This method is used to validate the parsed parameter and has
            to be implemented for the different parameter types.
            
            @param files:   Dictionary which contains the cStringIO.StringO
                            objects which contain the binary files sent with
                            the request.
            @type  files:   { str : cStringIO.StringO }
            
            @raise:     InvalidRequest or InternalError for compliance
                        with declaration of parse()
        """
        raise NotImplementedError('Can not instantiate a Parameter without a type.')
    
    @staticmethod
    def createParameter(name, paramType, opt=False, default=None):
        """ Creates a new parameter instance depending of paramType.
            
            @param name:    Name of the parameter
            @type  name:    str
            
            @param paramType:   String which represents the type of parameter
                                to create. Possible values are:
                                    int, str, float, bool, file
            @type  paramType:   str
            
            @param opt:     Flag which indicates whether the configuration
                            parameter is optional or not
            @type  opt:     bool
            
            @param default: Default value for the case where the parameter
                            is optional
            
            @raise:     InternalError in case the opt flag is set but no
                        default value is given.
        """
        if paramType == 'int':
            param = IntParam(name, opt, default)
        elif paramType == 'str':
            param = StrParam(name, opt, default)
        elif paramType == 'float':
            param = FloatParam(name, opt, default)
        elif paramType == 'bool':
            param = BoolParam(name, opt, default)
        elif paramType == 'file':
            param = FileParam(name, opt, default)
        else:
            raise InternalError('Could not identify the type of the configuration parameter.')
        
        return param

class IntParam(ParameterFactory):
    """ Represents an int parameter for a node.
    """
    def _validate(self, files):
        try:
            self.value = int(self.value)
        except ValueError:
            raise InvalidRequest('Configuration parameter {0} is not a valid int.'.format(self.name))

class StrParam(ParameterFactory):
    """ Represents an str parameter for a node.
    """
    def _validate(self, files):
        if isinstance(self.value, unicode):
            self.value = self.value.encode('utf-8')
        
        self.value = str(self.value)

class FloatParam(ParameterFactory):
    """ Represents an float parameter for a node.
    """
    def _validate(self, files):
        try:
            self.value = float(self.value)
        except ValueError:
            raise InvalidRequest('Configuration parameter {0} is not a valid float.'.format(self.name))

class BoolParam(ParameterFactory):
    """ Represents an bool parameter for a node.
    """
    def _validate(self, files):
        if isinstance(self.value, str) or isinstance(self.value, unicode):
            value = self.value.lower()
            
            if value == 'true':
                self.value = True
            elif value == 'false':
                self.value = False
            else:
                raise InvalidRequest('Configuration parameter {0} is not a valid bool.'.format(self.name))
        else:
            self.value = bool(self.value)

class FileParam(ParameterFactory):
    """ Represents an file parameter for a node.
    """
    def __init__(self, name, opt=False, default=None):
        super(FileParam, self).__init__(name, opt, default)
        
        self.tmpFile = False
    
    def _validate(self, files):
        # If we have to add a file to the ParameterServer which we received from the user create a temporary file
        if self.value != self.default:
            try:
                self.value = resolveReference(self.value)
            except ValueError:
                content = self.value
            else:
                content = files[self.value]
                content.seek(0)
                content = content.read()
            
            self.tmpFile = True
            (f, self.value) = mktempfile()
            tempFiles.append(self.value)
            f.write(content)
            f.close()
        
        if not os.path.isfile(self.value):
            raise InternalError('Could not find file.')
    
    def __del__(self):
        if self.tmpFile:
            try:
                os.remove(self.value)
            except OSError:
                pass
        
        super(FileParam, self).__del__()

class InterfaceFactory(object):
    """ Abstract base class which represents and provides a factory method
        for the interfaces to the nodes.
    """
    def __init__(self, msgCls, interfaceName):
        """ Constructor which initializes the basic fields of an interface.
            
            Should not used directly. Instead use the method createInterface(...)
            
            @param msgCls:  String representation of the message class
                            which is used to communicate with the interface.
            @type  msgCls:  str
            
            @param interfaceName:   String representation of the name of
                                    the interface.
            @type  interfaceName:   str
        """
        self.msgCls = msgCls
        self.interfaceName = interfaceName
    
    def parseMessage(self, data, files):
        """ Convert the message constisting of the dictionary data and the
            binaries files into a valid ROS message.
            
            @param data:    Dictionary which contains all the necessary data
                            to build a ROS message form it, i.e. all keys
                            correspond to a field and the values match the
                            field type.
            @type  data:    { str : ... }
            
            @param files:   Dictionary which contains the cStringIO.StringO
                            objects which contain the binary files sent with
                            the request.
            @type  files:   { str : cStringIO.StringO }
            
            @raise:     InvalidRequest if the message can not be parsed.
        """
        try:
            converter = Converter()
            self.msg = converter.decode(self.msgCls, data, files)
        except (TypeError, ValueError) as e:
            raise InvalidRequest(str(e))
    
    def run(self, manager, task):
        """ This method is used to send the request to the node.
            
            @param manager:     The EnvironmentManager which is used in
                                this environment node.
            @type  manager:     EnvironmentManager
            
            @param task:    The task ID which represents the current
                            request.
            @type  task:    str
            
            @raise:     InternalError if the message can not be sent.
        """
        raise NotImplementedError('Can not instantiate a Parameter without a type.')
    
    @staticmethod
    def createInterface(interfaceType, msgCls, interfaceName, interfaceDef):
        """ Creates a new interface instance depending on interfaceType.
            
            @param interfaceType:   String which represents the type of
                                    interface to create. Possible values are:
                                        topic, srv
            @type  interfaceType:   str
            
            @param msgCls:  String representation of the message class
                            which is used to communicate with the interface.
            @type  msgCls:  str
            
            @param interfaceName:   String representation of the name of
                                    the interface.
            @type  interfaceName:   str
            
            @param interfaceDef:    Additional arguments which are needed
                                    for the interface defintion.
            @type  interfaceDef:    tuple
            
            @raise:     InternalError in case the interface type can not
                        be resolved.
        """
        if interfaceType == 'srv':
            interface = ServiceInterface(msgCls, interfaceName, interfaceDef[0])
        elif interfaceType == 'topic':
            interface = TopicInterface(msgCls, interfaceName)
        else:
            InternalError('Could not identify the type of the interface.')
        
        return interface

class ServiceInterface(InterfaceFactory):
    """ Represents a service interface for a node.
    """
    def __init__(self, msgCls, interfaceName, srvCls):
        """ Constructor which initializes the basic fields of an interface.
            
            Should not used directly. Instead use the method createInterface(...)
            
            @param msgCls:  String representation of the message class
                            which is used to communicate with the interface.
            @type  msgCls:  str
            
            @param interfaceName:   String representation of the name of
                                    the interface.
            @type  interfaceName:   str
            
            @param srvCls:  String representaion of the service class which
                            is used.
            @type  srvCls:  str
        """
        super(ServiceInterface, self).__init__(msgCls, interfaceName)
        
        self.srvCls = srvCls
    
    def run(self, manager, task):
        self.manager = manager
        self.task = task
        
        taskThread = Task(self._taskFunc)
        manager.addTaskToThreadMngr(taskThread)
        manager.setTaskRunning(task)
        
        taskThread.start()
    
    def _taskFunc(self):
        try:
            srvCls = roslib.message.get_service_class(self.srvCls)
            
            rospy.wait_for_service(self.interfaceName, timeout=settings.WAIT_FOR_SERVICE_TIMEOUT)
            serviceFunc = rospy.ServiceProxy(self.interfaceName, srvCls)
            
            response = serviceFunc(self.msg)
        except rospy.ROSInterruptException:
            return
        except:
            self.manager.abortTask(self.task)
            import traceback
            traceback.print_exc()
            return
        
        self.manager.addResult(self.task, response)

class TopicInterface(InterfaceFactory):
    """ Represents a topic interface for a node.
    """
    def run(self, manager, task):
        try:
            msgCls = roslib.message.get_message_class(self.msgCls)
        except ValueError:
            raise InternalError('Message class is invalid.')
        
        try:
            rospy.Publisher(self.interfaceName, msgCls, latch=True).publish(self.msg)
        except rospy.ROSInterruptException:
            return
        except rospy.ROSSerializationException:
            raise InternalError('Message could not be serialized by ROS.')
        
        manager.addResult(task)
