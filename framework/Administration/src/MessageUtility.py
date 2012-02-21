#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       MessageUtility.py
#       
#       Copyright 2012 dominique hunziker <dominique.hunziker@gmail.com>
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
import json
import cStringIO
import struct
from functools import wraps

# Custom imports
import settings

class _Status(object):
    OK = 0
    INVALID = 1
    ERROR = 2
    FATAL = 3

class InternalError(Exception):
    """ This class is used to signal an internal error.
    """
    pass

class InvalidRequest(Exception):
    """ This class is used to signal an invalid request.
    """
    pass

class FatalError(Exception):
    """ This class is used to signal a fatal error.
    """
    pass

def serviceCallback(responseMsgType):
    """ Decorator which allows to use callback functions which use the
        two exceptions InvalidRequest and InternalError.
        The decorator needs a response message which has at least the two
        fields msg (string) and status (uint8).
        
        @param responseMsgType: The class which should be used to generate
                                the response message
        @type  responseMsgType: ROS Service Response class
    """
    def decorator(f):
        @wraps(f)
        def wrapper(*args, **kw):
            try:
                msg = f(*args, **kw)
                status = _Status.OK
            except InvalidRequest as e:
                msg = str(e)
                status = _Status.INVALID
            except InternalError as e:
                msg = str(e)
                status = _Status.ERROR
            except Exception:
                import traceback
                msg = traceback.format_exc()
                status = _Status.FATAL
            
            return responseMsgType(status=status, msg=msg)
        
        return wrapper
    
    return decorator

def setupService(name, serviceDef, callback, *args, **kw):
    """ Use this function to setup a ROS Service where the callback function
        needs more than the request object as argument. The function returns
        the new rospy.Service instance.
        
        Usage:  The first three arguments are the same as for the constructor
                of the rospy.Service class. The following arguments are just
                passed to the callback function in the same order as received.
    """
    return rospy.Service(name, serviceDef, lambda request: callback(request, *args, **kw))

def preprocessFiles(files):
    """ The preprocessing does serialize the files object into a string
        to enable the transportation with a ROS message.
    """
    try:
        return serializeFiles(files)
    except ConversionError:
        raise InternalError('Could not serialize the files in POST request.')

def genericService(name, service, **kw):
    """ Standard behaviour for the service functions.
        The decorated function should use only keyworded arguments.
        
        @param name:    Name should be the service name which should be
                        used.
                        (First argument of rospy.ServiceProxy)
        @type  name:    str
        
        @param service: Class of the ROS Service which should be used.
                        (Second argument of rospy.ServiceProxy)
        @type  service: ROS Service class
        
        @param **kw:        Optional argument.
                            All keyworded arguments are passed to the
                            Service.
    """
    try:
        for key in kw:
            if isinstance(kw[key], unicode):
                kw[key] = kw[key].encode('utf-8')
    except UnicodeDecodeError:
        raise InvalidRequest('Request contains characters which are not ascii compatible.')
    
    try:
        serviceFunc = rospy.ServiceProxy(name, service)
        response = serviceFunc(**kw)
    except rospy.ROSInterruptException:
        return
    except rospy.ROSException as e:
        raise InternalError('ROS Exception: '+str(e))
    
    if response.status == _Status.INVALID:
        raise InvalidRequest(response.msg)
    elif response.status == _Status.ERROR:
        raise InternalError(response.msg)
    elif response.status == _Status.FATAL:
        raise FatalError(response.msg)
    
    return response.msg

########################################################################
# Serialization tools

_struct_I = struct.Struct('<I')

class SerializeError(Exception):
    """ General Exception class which is used for conversion errors.
    """
    pass

def serializeFiles(files):
    """ Serialize a dictionary full of lists containing cStringIO.StringO
        objects.
        
        @param files:   Files should be a dictionary containing
                        cStringIO.StringO instances as values.
        @type  files:   { str : cStringIO.StringO }
        
        @return:        Serialized form of files
        @rtype:         str
    """
    out = cStringIO.StringIO()
    
    out.write(_struct_I.pack(len(files)))
    
    for key in files:
        length = len(key)
        out.write(struct.pack('<I%ss'%length, length, str(key)))
        
        strRepr = files[key].getvalue()
        length = len(strRepr)
        out.write(struct.pack('<I%ss'%length, length, strRepr))
    
    return out.getvalue()

def deserializeFiles(str):
    """ Deserialize a string to a dictionary full of lists containing
        cStringIO.StringO objects.
        
        @param str:     Serialized form of files
        @type  str:     str
        
        @return:        The files in a dictionary which contains
                        cStringIO.StringO instances as values.
        @rtype:         { str : cStringIO.StringO }
    """
    out = {}
    
    try:
        end = 0
        
        start = end
        end += 4
        (lengthDict,) = _struct_I.unpack(str[start:end])
        
        for iDict in xrange(lengthDict):
            start = end
            end += 4
            (length,) = _struct_I.unpack(str[start:end])
            start = end
            end += length
            key = str[start:end]
            
            start = end
            end += 4
            (length,) = _struct_I.unpack(str[start:end])
            start = end
            end += length
            fileObj = cStringIO.StringIO()
            fileObj.write(str[start:end])
            
            out[key] = fileObj
    except struct.error as e:
        raise SerializeError(e)
    
    return out
