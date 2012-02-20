#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       ServiceAPI.py
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

import json
import httplib
import urllib

import cStringIO
import uuid
import mimetypes

_HOST = 'localhost'
#_HOST = '50.56.194.140'
_BASE_ADRESS = '/api/ros_service'
_REFERENCE_PREFIX = 'REF:'

class RequestError(Exception):
    """General error which is raised when HTTP Header indicates a problem."""
    
    def __init__(self, code, reason, msg):
        self.code = code
        self.reason = reason
        self.msg = msg
    
    def __str__(self):
        return '{0} {1}\n{2}'.format(self.code, self.reason, self.msg)

class FileHandle(object):
    """Convenience class to store file related information."""
    
    def __init__(self, filename, fileHandle=None, mimetype=None):
        self.filename = filename
        
        if fileHandle:
            self.fileHandle = fileHandle
        else:
            self.fileHandle = open(filename)
        
        self.mimetype = mimetype
    
    def __del__(self):
        try:
            self.fileHandle.close()
        except AttributeError:
            pass
    
    def convertToTuple(self, fieldname):
        return (fieldname, self.filename, self.fileHandle, self.mimetype)

class _MultiPartForm(object):
    """Accumulate the data to be used when posting a form."""
    
    def __init__(self):
        self.form_fields = []
        self.files = []
        self.boundary = '--x-x-{0}-x-x'.format(uuid.uuid4())
        return
    
    def getContentType(self):
        return 'multipart/form-data; boundary={0}'.format(self.boundary)
    
    def addField(self, name, value):
        """Add a simple field to the form data."""
        self.form_fields.append((name, value))
    
    def addFile(self, fieldname, filename, fileHandle, mimetype=None):
        """Add a file to be uploaded."""
        fileHandle.seek(0)
        body = fileHandle.read()
        
        if mimetype is None:
            mimetype = mimetypes.guess_type(filename)[0] or 'application/octet-stream'
        
        self.files.append((fieldname, filename, mimetype, body))
    
    def __str__(self):
        """Return a string representing the form data, including attached files."""
        out = cStringIO.StringIO()
        part_boundary = '--{0}\r\n'.format(self.boundary)
        
        # Add the form fields
        for (name, value) in self.form_fields:
            out.write(part_boundary)
            out.write('Content-Disposition: form-data; name="{0}"\r\n'.format(name))
            out.write('\r\n')
            out.write('{0}\r\n'.format(value))
        
        # Add the files to upload
        for (field_name, filename, content_type, body) in self.files:
            out.write(part_boundary)
            out.write('Content-Disposition: form-data; name="{0}"; filename="{1}"\r\n'.format(field_name, filename))
            out.write('Content-Type: {0}\r\n'.format(content_type))
            out.write('\r\n')
            out.write('{0}\r\n'.format(body))
        
        out.write('--{0}--\r\n'.format(self.boundary))
        out.write('\r\n')
        
        return out.getvalue()

def _processGET(url):
    """ Process a GET request.
        
        @param url:     url where the GET request is sent to.
        @type  url:     str
    """
    conn = httplib.HTTPConnection(_HOST)
    
    if len(url) and url[0] != '/':
        url = '/{0}'.format(url)
    
    conn.request('GET', url)
    r = conn.getresponse()
    
    if r.status != 200:
        raise RequestError(r.status, r.reason, r.read())
    
    return json.loads(r.read())

def _processPOST(url, data, files):
    """ Process a POST request.
        
        @param url:     url where the POST request is sent to.
        @type  url:     str
        
        @param data:    Dictionary which contains all key/value pairs
                        which should be sent with the POST request.
        @type  data:    { str : str }
        
        @param files:   List of file descriptions which are tuples of the
                        form (fieldname, filename, fileHandle, mimetype)
        @type  files:   (str, str, FileHandle instace, str)
    """
    conn = httplib.HTTPConnection(_HOST)
    
    if files:
        form = _MultiPartForm()
        
        for key in data:
            form.addField(key, data[key])
        
        for (fieldname, filename, fileHandle, mimetype) in files:
            form.addFile(fieldname, filename, fileHandle, mimetype)
        
        body = str(form)
        headers = { 'Content-Type': form.getContentType() }
    elif data:
        body = urllib.urlencode(data)
        headers = { 'Content-Type': 'application/x-www-form-urlencoded' }
    else:
        body = None
        headers = {}
    
    if len(url) and url[0] != '/':
        url = '/{0}'.format(url)
    
    conn.request('POST', url, body, headers)
    r = conn.getresponse()
    
    if r.status != 200:
        raise RequestError(r.status, r.reason, r.read())
    
    return json.loads(r.read())

def _processDELETE(url):
    """ Process a DELETE request.
        
        @param url:     url where the DELETE request is sent to.
        @type  url:     str
    """
    conn = httplib.HTTPConnection(_HOST)
    
    if len(url) and url[0] != '/':
        url = '/{0}'.format(url)
    
    conn.request('DELETE', url)
    r = conn.getresponse()
    
    if r.status != 204:
        raise RequestError(r.status, r.reason, r.read())

def getAvailableServices():
    """ Get available nodes and services.
        
        @return:    Dictionary containing all available nodes. As key
                    the node name is used (which is needed to add/remove
                    a node). As value a dictionary containing information
                    about the node is returned.
        @rtype:     { str : {} }
    """
    return _processGET(_BASE_ADRESS)

def addEnv():
    """ Create a new environment.
        
        @return:    New environment ID
        @rtype:     str
    """
    return _processPOST(_BASE_ADRESS, {}, [])['envID']

def changeEnv(envID=None, nodesToAdd=[], nodesToRemove=[]):
    """ Change the nodes in the environment.
        
        @param envID:   Environment ID which should be changed.
                        If no ID is given a new environment is created.
        @type  envID:   str
        
        @param nodesToAdd:  List of nodes with their configuration which
                            should be added to the environment. A list
                            entry should be a tuple of the for
                            (nodeName, config)
        @param nodesToAdd:  [(str, { str: bool/int/float/str/fileHandle })]
        
        @param nodesToRemove:   List of nodes which should be removed from
                                the environment. A list entry should be a
                                string containing the nodeName.
        @param nodesToReomve:   [str]
        
        @return:    Environment ID where the nodes where added/removed.
        @rtype:     str
    """
    # Is a request necessary?
    if not (nodesToAdd or nodesToRemove):
        if envID:
            return envID
        else:
            return addEnv()
    
    # Request is necessary; prepare the data
    data = {}
    files = []
    
    if nodesToAdd:
        add = {}
        
        for (nodeName, config) in nodesToAdd:
            if nodeName in add:
                raise ValueError('Can not add the same node twice.')
            
            if config:
                (subMsg, subFiles) = _processMessage(config, '/{0}/'.format(nodeName))
                add[nodeName] = subMsg
                files += subFiles
            else:
                add[nodeName] = {}
        
        data['add'] = add
    
    if nodesToRemove:
        data['remove'] = nodesToRemove
    
    body = { 'data' : json.dumps(data) }
    
    if envID:
        body['ID'] = envID
    
    return _processPOST(_BASE_ADRESS, body, files)['envID']

def getEnv(envID):
    """ Get running nodes in environment.
        
        @param envID:       Environment ID to analyse.
        @type  envID:       str
        
        @return:    Dictionary containing all nodes in the environment
                    as keys and their status as values.
        @rtype:     { str : str }
    """
    return _processGET('{0}/{1}'.format(_BASE_ADRESS, envID))

def removeEnv(envID):
    """ Remove an environment.
        
        @param envID:       Environment ID to delete.
        @type  envID:       str
    """
    _processDELETE('{0}/{1}'.format(_BASE_ADRESS, envID))

def addTask(envID, service, message):
    """ Add a task.
        
        @param service:     The service which should be used.
        @type  service:     str
        
        @param message:     Dictionary containing strings as keys and
                            other dictionaries, lists, fileHandle or
                            base types as values. A fileHandle has to
                            have the method read().
        @type  message:     { str : bool/int/float/str/[]/{}/fileHandle }
        
        @return:        Task ID
        @rtype:         str
        
        @raise:     TypeError if the message is not valid.
    """
    (msg, files) = _processMessage(message)
    body = {'data' : json.dumps({'service' : service, 'msg' : msg }) }
    return _processPOST('{0}/{1}'.format(_BASE_ADRESS, envID), body, files)['taskID']

def getTask(envID, taskID):
    """ Get the status/result of a task.
        
        @param envID:       Environment ID in which the task is.
        @type  envID:       str
        
        @param taskID:      Task ID to retrieve.
        @type  taskID:      str
        
        @return:    Tuple containing the information about the task:
                    (status, result)
                    
                    If the status of the task is not yet 'completed'
                    than result will be None.
                    The returned result is the json formatted dictionary.
        @rtype:     (str, {})
    """
    response = _processGET('{0}/{1}/{2}'.format(_BASE_ADRESS, envID, taskID))
    
    if isinstance(response['data'], unicode):
        response['data'] = json.loads(response['data'].encode('utf-8'))
    
    return (response['status'], response['data'])

def removeTask(envID, taskID):
    """ Remove a task.
        
        @param envID:       Environment ID in which the task is.
        @type  envID:       str
        
        @param taskID:      Task ID to delete.
        @type  taskID:      str
    """
    _processDELETE('{0}/{1}/{2}'.format(_BASE_ADRESS, envID, taskID))

def _processMessage(message, basename='/'):
    """ Helper function to process a message.
    """
    data = {}
    files = []
    
    if not isinstance(message, dict):
        raise TypeError('Received message is not a dict.')
    
    for key in message:
        field = message[key]
        
        if isinstance(field, dict):
            (data[key], subFiles) = _processMessage(field, '{0}{1}/'.format(basename, key))
            
            for key in subFiles:
                if key in files:
                    raise ValueError('Multiple values for the same key.')
                
                files[key] = subFiles[key]
        elif isinstance(field, list):
            data[key] = [None]*len(field)
            for i in xrange(len(field)):
                if isinstance(field[i], dict):
                    (data[key][i], subFiles) = _processMessage(field[i], '{0}{1}_{2}/'.format(basename, key, i))
                    
                    for key in subFiles:
                        if key in files:
                            raise ValueError('Multiple values for the same key.')
                        
                        files[key] = subFiles[key]
                elif hasattr(field[i], 'convertToTuple'):
                    data[key][i] = '{3}{0}{1}_{2}/file'.format(basename, key, i, _REFERENCE_PREFIX)
                    files.append(field[i].convertToTuple('{0}{1}_{2}/file'.format(basename, key, i)))
                else:
                    data[key][i] = field[i]
        elif hasattr(field, 'convertToTuple'):
            data[key] = '{2}{0}{1}/file'.format(basename, key, _REFERENCE_PREFIX)
            files.append(field.convertToTuple('{0}{1}/file'.format(basename, key)))
        else:
            data[key] = field
    
    return (data, files)
