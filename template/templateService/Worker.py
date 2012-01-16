#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       Worker.py
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

from threading import Thread

class WorkerError(Exception):
    """ General Exception in connection with a Worker.
    """
    pass

class ObjectWorker(object):
    """ Generic base class for a Worker who can be used in conjunction
        with the Request class for a ROS service. This Worker should be
        used for serial execution.
    """
    CONF_FIELDS=[]
    DATA_FIELDS={}
    
    def __init__(self, **kargs):
        """ The given arguments have to be keyworded arguments. Relevant
            arguments are data, target, debug, info, warning, error, fatal.
            The value for a key should be a callable object except for data.
            If no value for a possible key can be found None is assigned.
            The data for the worker should be in a dictionary which is the
            value of the keyworded argument data. This dictionary should
            contains at least the keys in the list DATA_FIELDS, else an
            error message is dispatched.
        """
        if len(self.__class__.DATA_FIELDS) != 0:
            try:
                data=kargs['data']
            except KeyError:
                raise WorkerError("The keyworded arguments miss the key 'data'.")
            
            self._assign(self, data, self.__class__.DATA_FIELDS, 'data')
        
        for key in ['target', 'debug', 'info', 'warning', 'error', 'fatal']:
            if key not in kargs:
                func=None
            else:
                func=kargs[key]
            
            if not callable(func):
                func=None
            
            setattr(self, '_{0}Func'.format(key), func)
    
    def run(self):
        """ Overwrite this method, else a NotImplementedError is raised.
            This method is executed when the request has to be processed.
        """
        raise NotImplementedError('The method run for the used Worker is not implemented.')
    
    def appendData(self, data):
        """ This method is used to send the data to the Request instance.
        """
        self._targetFunc(data)
    
    def formatMsg(self, msg, func):
        """ Overwite this method if you want to change the appearance of
            the message sent. func should be one of the five message
            methods debug, info, warning, error or fatal. If the
            method is overwitten make sure to check whether the given
            func is None or not.
        """
        if func is not None:
            func('[{0}] {1}'.format(self.__class__.__name__,  msg))
    
    def debug(self, msg):
        """ Use this method to send a debug message.
        """
        self.formatMsg(msg, self._debugFunc)
    
    def info(self, msg):
        """ Use this method to send a info message.
        """
        self.formatMsg(msg, self._infoFunc)
    
    def warning(self, msg):
        """ Use this method to send a warning message.
        """
        self.formatMsg(msg, self._warningFunc)
    
    def error(self, msg):
        """ Use this method to send a error message.
        """
        self.formatMsg(msg, self._errorFunc)
    
    def fatal(self, msg):
        """ Use this method to send a fatal message.
        """
        self.formatMsg(msg, self._fatalFunc)
    
    def _assign(self, dictToSave, dataDict, dictToKeyNames, fieldName):
        """ Used to check if input in data field is correct.
        """
        try:
            for key in dictToKeyNames:
                if dictToKeyNames[key] is None:
                    if dictToSave == self:
                        setattr(self, key, dataDict[key])
                    else:
                        dictToSave[key]=dataDict[key]
                elif isinstance(dictToKeyNames[key], list):
                    if dictToKeyNames[key][0] is None:
                        if dictToSave == self:
                            setattr(self, key, dataDict[key])
                        else:
                            dictToSave[key]=dataDict[key]
                    elif isinstance(dictToKeyNames[key][0], dict):
                        if dictToSave == self:
                            setattr(self, key, [{}]*len(dataDict[key]))
                            listToSave=getattr(self, key)
                        else:
                            dictToSave[key]=[{}]*len(dataDict[key])
                            listToSave=dictToSave[key]
                        
                        for i in xrange(len(dataDict[key])):
                            self._assign(listToSave[i], dataDict[key][i], dictToKeyNames[key][0], key)
                    else:
                        raise WorkerError('Format of DATA_FIELD is wrong.')
                elif isinstance(dictToKeyNames[key], dict):
                    if dictToSave == self:
                        setattr(self, key, {})
                        self._assign(getattr(self, key), dataDict[key], dictToKeyNames[key], key)
                    else:
                        dictToSave[key]={}
                        self._assign(dictToSave[key], dataDict[key], dictToKeyNames[key], key)
                else:
                    raise WorkerError('Format of DATA_FIELD is wrong.')
        except KeyError:
            raise WorkerError("The {0} misses the key '{1}'.".format(fieldName, key))
        except Exception:
            raise WorkerError('Format of DATA_FIELD is wrong.')
    
    @classmethod
    def configure(cls, config):
        """ This method allows to configure a Worker. config has to be
            a dictionary which contains at least the keys in the list
            CONF_FIELDS, else an error message is dispatched.
        """
        try:
            for field in cls.CONF_FIELDS:
                setattr(cls, '_{0}'.format(field.upper()), config[field])
        except KeyError:
            raise WorkerError("The configuration file misses the key '{0}' for '{1}'.".format(key, cls.__name__))

class ThreadWorker(ObjectWorker, Thread):
    """ Generic base class for a Worker who can be used in conjunction
        with the Request class for a ROS service. This Worker should be
        used for parallel execution.
    """
    def __init__(self, **kargs):
        Thread.__init__(self)
        ObjectWorker.__init__(self, **kargs)
