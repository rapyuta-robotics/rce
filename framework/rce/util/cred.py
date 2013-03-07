#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     cred.py
#
#     This file is part of the RoboEarth Cloud Engine framework.
#
#     This file was originally created for RoboEearth
#     http://www.roboearth.org/
#
#     The research leading to these results has received funding from
#     the European Union Seventh Framework Programme FP7/2007-2013 under
#     grant agreement no248942 RoboEarth.
#
#     Copyright 2012 RoboEarth
#
#     Licensed under the Apache License, Version 2.0 (the "License");
#     you may not use this file except in compliance with the License.
#     You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
#     Unless required by applicable law or agreed to in writing, software
#     distributed under the License is distributed on an "AS IS" BASIS,
#     WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#     See the License for the specific language governing permissions and
#     limitations under the License.
#
#     \author/s: Dhananjay Sathe
#
#
import os
import fileinput
from zope.interface import implements

from twisted.internet import defer
from twisted.python import failure
from twisted.cred import error, credentials
from twisted.cred.checkers import ICredentialsChecker
from hashlib import sha512
import re

_RE = r'(\w+):(\w+)\s+'
_PASS_RE = r'^.*(?=.{4,10})(?=.*[a-z])(?=.*[A-Z])(?=.*[\d])(?=.*[\W]).*$'
password_fail = 'Password must be between 4-10 Digits one each of uppercase,lowercase, digit and special character '
class CredentialError(Exception):
    pass

class RCECredChecker:
    """The RCE file-based, text-based username/password database.
    """

    implements(ICredentialsChecker)

    cache = False
    _credCache = None
    _cacheTimestamp = 0

    def __init__(self, filename):
        """
        @type filename: C{str}
        @param filename: The name of the file from which to read username and
        password information.
        """
        self.filename = filename
        self.hash = lambda x : sha512(x).hexdigest()
        self.credentialInterfaces = credentials.IUsernamePassword
        self.scanner = re.compile(_RE)
        pass_re = re.compile(_PASS_RE)
        self.pass_validator = lambda x : True if pass_re.match(x) else False
        
        # Run some basic tests to check if the settings file is valid
        
        if self.filename is None :
                print('Settings variable PASSWORD_FILE not set')
                exit()
        
        try:
            open(self.filename,'w').close()
        except IOError:
            print('The user lacks privileges to access/modify the password file.')
            exit()
        
        # Provision for first run needs to be initialized with the default admin user creds
        if not os.path.exists(self.filename):
            self.addUser('admin','admin', provision= True)
            # TODO : Remove this user later , temporarily inserting for maintaining compat with current test classes.
            self.addUser('testUser','testUser', provision= True)
            Warning('Please reset the admin password using the console utility. The Default password is admin ')
            
    def __getstate__(self):
        d = dict(vars(self))
        for k in '_credCache', '_cacheTimestamp':
            try:
                del d[k]
            except KeyError:
                pass
        return d


    def _cbPasswordMatch(self, matched, username):
        if matched:
            return username
        else:
            return failure.Failure(error.UnauthorizedLogin())


    def _loadCredentials(self):
        with open(self.filename) as f :
            for line in f:
                parts = self.scanner.match(line).groups()
                yield parts
    
    def getUser(self, username):
        if self._credCache is None or os.path.getmtime(self.filename) > self._cacheTimestamp:
            self._cacheTimestamp = os.path.getmtime(self.filename)
            self._credCache = dict(self._loadCredentials())
        return username, self._credCache[username]

    def requestAvatarId(self, c):
        try:
            u, p = self.getUser(c.username)
        except KeyError:
            return defer.fail(error.UnauthorizedLogin())
        else:
            up = credentials.IUsernamePassword(c, None)
            if up is not None:
                h = self.hash(up.password)
                if h == p:
                    return defer.succeed(u)
            return defer.fail(error.UnauthorizedLogin())


    def addUser(self, username, password, provision= False):
        if not self.pass_validator(password) or provision:
            raise CredentialError(password_fail)
        try: 
            self.getUser(username)
            raise CredentialError('Given user already exists')
        except KeyError:
            with open(self.filename, 'a') as f :
                f.writelines((':'.join((username,self.hash(password)))+'\n'))
            return True

    def removeUser(self,username):
        try: 
            self.getUser(username)# why bother reading the file if the user doesn't even exist ! 
            for line in fileinput.input(self.filename, inplace=1):
                if self.scanner.match(line).groups()[0] != username:
                    print line[:-1]
            return True
        except KeyError:
            raise CredentialError('No such user') 

    def passwd(self,username, password): 
        if not self.pass_validator(password):
            raise CredentialError(password_fail)
        try: 
            self.getUser(username) # why bother reading the file if the user doesn't even exist ! 
            for line in fileinput.input(self.filename, inplace=1):
                if self.scanner.match(line).groups()[0] != username:
                    print line[:-1]
                else:
                    print ':'.join((username,self.hash(password)))
            return True
        except KeyError:
            raise CredentialError('No such user') 
