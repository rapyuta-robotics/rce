#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       ParameterParser.py
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

# Python specific imports
import os

# Custom imports
from Exceptions import InvalidRequest, InternalError
import ComponentDefinition
from Parameter import IntParam, StrParam, FloatParam, BoolParam, FileParam

class _ParameterParser(object):
    """ Abstract base class which represents and provides a factory method
        for the node configuration parameters. This class is used to parse
        the received request data which is then forwarded to the correct
        environment where a ParameterBase instance is created based on the
        parsed information.
    """
    _CLASS = None
    
    def __init__(self, name, opt=False, default=None):
        """ Constructor which initializes the basic fields of a parameter.

            Should not be used directly. Instead use the provided function
            createParameter().

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

        if isinstance(name, unicode):
            try:
                name = str(name)
            except UnicodeEncodeError:
                raise InternalError('The parameter name {0} is not valid.'.format(name))

        self._name = name
        self._opt = opt
        self._default = default
        self._value = None

    def parse(self, config, namespace):
        """ Parses the given configuration parameter.

            @param config:  Dictionary which contains all given parameters as keys and
                            their corresponding values as values. Binary files are
                            represented as strings.
            @type  config:  { str : ... }

            @param namespace:   Namespace in which the parameter should be added.s
            @type  namespace:   str
            
            @return:    Parameter instance which can be passed to the ParameterMonitor.
            @rtype:     Parameter

            @raise:     InvalidRequest in case of a parsing error or else
                        InternalError.
        """
        # Check whether the parameter has already been parsed
        if not hasattr(self, '_opt') or not hasattr(self, '_default'):
            return

        # Try to load the value or the default value
        if self._name in config:
            self._value = config[self._name]
        elif self._opt:
            self._value = self._default

        # Make sure that there is a value
        if self._value is None:
            raise InvalidRequest('Parameter {0} is missing.'.format(self._name))

        # Validate value
        try:
            self._validate()
        except (InvalidRequest, InternalError):
            raise
        except Exception as e:
            raise InternalError('{0}: {1}'.format(e.__class__.__name__, str(e)))

        # Delete the opt flag and default value to signal successful parsing
        del self._opt
        del self._default

        # Also change _validate to prevent undesired behavior
        def _validate(): pass
        self._validate = _validate
        
        if self._CLASS:
            return self._CLASS('{0}/{1}'.format(namespace, self._name), self._value)
        else:
            raise NotImplementedError('Can not parse a Parameter without a type.')

    def _validate(self):
        """ This method is used to validate the parsed parameter and has
            to be implemented for the different parameter types.

            @raise:     InvalidRequest or InternalError for compliance
                        with declaration of parse()
        """
        raise NotImplementedError('Can not parse a Parameter without a type.')

class IntParamParser(_ParameterParser):
    """ Used to create and parse an int parameter.
    """
    IDENTIFIER = ComponentDefinition.PARAM_INT
    _CLASS = IntParam

    def _validate(self):
        try:
            self._value = int(self._value)
        except ValueError:
            raise InvalidRequest('Configuration parameter {0} is not a valid int.'.format(self._name))

    _validate.__doc__ = _ParameterParser._validate.__doc__

class StrParamParser(_ParameterParser):
    """ Used to create and parse a str parameter.
    """
    IDENTIFIER = ComponentDefinition.PARAM_STR
    _CLASS = StrParam

    def _validate(self):
        if isinstance(self._value, unicode):
            self._value = self._value.encode('utf-8')
        else:
            self._value = str(self._value)

    _validate.__doc__ = _ParameterParser._validate.__doc__

class FloatParamParser(_ParameterParser):
    """ Used to create and parse a float parameter.
    """
    IDENTIFIER = ComponentDefinition.PARAM_FLOAT
    _CLASS = FloatParam

    def _validate(self):
        try:
            self._value = float(self._value)
        except ValueError:
            raise InvalidRequest('Configuration parameter {0} is not a valid float.'.format(self._name))

    _validate.__doc__ = _ParameterParser._validate.__doc__

class BoolParamParser(_ParameterParser):
    """ Used to create and parse a bool parameter.
    """
    IDENTIFIER = ComponentDefinition.PARAM_BOOL
    _CLASS = BoolParam

    def _validate(self):
        if isinstance(self._value, str) or isinstance(self._value, unicode):
            value = self._value.lower()

            if value == 'true':
                self._value = True
            elif value == 'false':
                self._value = False
            else:
                raise InvalidRequest('Configuration parameter {0} is not a valid bool.'.format(self._name))
        else:
            self._value = bool(self._value)

    _validate.__doc__ = _ParameterParser._validate.__doc__

class FileParamParser(_ParameterParser):
    """ Used to create and parse a file parameter.
    """
    IDENTIFIER = ComponentDefinition.PARAM_FILE
    _CLASS = FileParam

    def _validate(self):
        if self._value == self._default:
            if not os.path.isfile(self._value):
                raise InternalError('Could not find the default file.')

            with open(self._value, 'r') as f:
                self._value = f.read()

    _validate.__doc__ = _ParameterParser._validate.__doc__
