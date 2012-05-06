#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       _ParameterParser.py
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
from struct import error as StructError

try:
    from cStringIO import StringIO
except ImportError:
    from StringIO import StringIO

# Custom imports
from Exceptions import InvalidRequest, InternalError, SerializationError
from ContentDefinition import ContentDefinition, STRUCT_I
from Converter.Utility import resolveReference

from ._ParserBase import ParserBase

class ParameterBaseParser(ParserBase):
    """ Abstract base class which represents and provides a factory method
        for the node configuration parameters. This class is used to parse
        the received request data which is then forwarded to the correct
        environment where a ParameterBase instance is created based on the
        parsed information.
    """
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

        self.name = name
        self.opt = opt
        self.default = default
        self.value = None
        self.rosName = None

    def setNamespace(self, namespace):
        """ Set the namespace in which the parameter and its parent node
            lives.

            @param namespace:   Namespace under which the parameter should
                                be saved.
            @type  namespace:   str
        """
        self.rosName = '{0}/{1}'.format(namespace, self.name)

        if isinstance(self.rosName, unicode):
            self.rosName = self.rosName.encode('utf-8')

    def parse(self, config, files):
        """ Parses the given configuration parameter.

            @param config:  Dictionary which contains all given parameters
                            as keys and their corresponding values as values.
            @type  config:  { str : ... }

            @param files:   Dictionary which contains the cStringIO.StringO
                            objects which contain the binary files sent with
                            the request.
            @type  files:   { str : cStringIO.StringO }

            @raise:     InvalidRequest in case of a parsing error or else
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

        # Delete the opt flag and default value to signal successful parsing
        del self.opt
        del self.default

        # Also change _validate to prevent undesired behavior
        def _validate(files): pass
        self._validate = _validate

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

    def serialize(self):
        """ Serialize the Parameter instance.

            @raise:     SerializationError
        """
        if hasattr(self, 'opt') or hasattr(self, 'default') or self.value is None or self.rosName is None:
            raise SerializationError('Can not send the parameter before the configuration has been parsed and the ROS name has been generated.')

        buf = StringIO()

        try:
            buf.write(self.IDENTIFIER)
            buf.write(STRUCT_I.pack(len(self.value)))
            buf.write(self.value)
            buf.write(STRUCT_I.pack(len(self.rosName)))
            buf.write(self.rosName)
        except (StructError, TypeError) as e:
            raise SerializationError('Could not serialize Parameter: {0}'.format(e))

        return buf.getvalue()

class IntParamParser(ParameterBaseParser):
    """ Used to create and parse an int parameter.
    """
    IDENTIFIER = ContentDefinition.PARAM_INT

    def _validate(self, files):
        try:
            self.value = int(self.value)
        except ValueError:
            raise InvalidRequest('Configuration parameter {0} is not a valid int.'.format(self.name))

    _validate.__doc__ = ParameterBaseParser._validate.__doc__

class StrParamParser(ParameterBaseParser):
    """ Used to create and parse a str parameter.
    """
    IDENTIFIER = ContentDefinition.PARAM_STR

    def _validate(self, files):
        if isinstance(self.value, unicode):
            self.value = self.value.encode('utf-8')

        self.value = str(self.value)

    _validate.__doc__ = ParameterBaseParser._validate.__doc__

class FloatParamParser(ParameterBaseParser):
    """ Used to create and parse a float parameter.
    """
    IDENTIFIER = ContentDefinition.PARAM_FLOAT

    def _validate(self, files):
        try:
            self.value = float(self.value)
        except ValueError:
            raise InvalidRequest('Configuration parameter {0} is not a valid float.'.format(self.name))

    _validate.__doc__ = ParameterBaseParser._validate.__doc__

class BoolParamParser(ParameterBaseParser):
    """ Used to create and parse a bool parameter.
    """
    IDENTIFIER = ContentDefinition.PARAM_BOOL

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

    _validate.__doc__ = ParameterBaseParser._validate.__doc__

class FileParamParser(ParameterBaseParser):
    """ Used to create and parse a file parameter.
    """
    IDENTIFIER = ContentDefinition.PARAM_FILE

    def _validate(self, files):
        if self.value == self.default:
            if not os.path.isfile(self.value):
                raise InternalError('Could not find file.')

            with open(self.value, 'r') as f:
                self.value = f.read()
        else:
            try:
                self.value = resolveReference(self.value)
            except ValueError:
                pass
            else:
                self.value = files[self.value]
                self.value.seek(0)
                self.value = self.value.read()

    _validate.__doc__ = ParameterBaseParser._validate.__doc__
