#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     rce-core/rce/util/cred.py
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
#     Copyright 2013 RoboEarth
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

# global imports
import os
import fileinput
import re
import base64
from hashlib import sha256
from Crypto.Cipher import AES

# twisted specific imports
from zope.interface import implements
from twisted.internet import defer
from twisted.python import failure
from twisted.cred import error
from twisted.cred.credentials import IUsernameHashedPassword
from twisted.cred.checkers import ICredentialsChecker


_RE = r'(\w+):(.+)'
_PASS_RE = r'^.*(?=.{4,20})(?=.*[a-z])(?=.*[A-Z])(?=.*[\d])(?=.*[\W]).*$'
_PASSWORD_FAIL = ('Password must be between 4-20 digits long and has to '
                  'contain at least one uppercase, lowercase, digit, and '
                  'special character.')

### AES Encryption Stuff
# AES Encryptors strength depends on input password length, ensure it with
# appropriate hash
# the block size for the cipher object; must be 32 for AES 256
_BLOCK_SIZE = 32

# the character used for padding
#   with a block cipher such as AES, the value you encrypt must be a multiple
#   of BLOCK_SIZE in length. This character is used to ensure that your value
#   is always a multiple of BLOCK_SIZE
_PADDING = '{'

# one-liner to sufficiently pad the text to be encrypted
pad = lambda s: s + (_BLOCK_SIZE - len(str(s)) % _BLOCK_SIZE) * _PADDING

# one-liners to encrypt/encode and decrypt/decode a string
# encrypt with AES, encode with base64
EncodeAES = lambda c, s: base64.b64encode(c.encrypt(pad(s)))
cipher = lambda passwd: AES.new(passwd)
salter = lambda u, p: sha256(u+p).digest()


class CredentialError(Exception):
    """ TODO: Add doc
    """


class RCECredChecker(object):
    """The RCE file-based, text-based username/password database.
    """
    implements(ICredentialsChecker)

    cache = False
    _credCache = None
    _cacheTimestamp = 0

    def __init__(self, filename, dev_mode=False):
        """
            @param filename:    The creds file to be usd
            @type  filename:    str
            
            @param dev_mode:    Indicate if developer mode is enabled.
            @type  dev_mode:    bool
        """
        self.filename = filename
        self.credentialInterfaces = (IUsernameHashedPassword,)
        self.scanner = re.compile(_RE)
        pass_re = re.compile(_PASS_RE)
        self.pass_validator = lambda x: True if pass_re.match(x) else False

        # Run some basic tests to check if the settings file is valid
        if self.filename is None :
            print('Settings variable PASSWORD_FILE not set')
            exit()

        if not os.access(os.path.dirname(self.filename), os.W_OK):
            print('The user lacks privileges to access/modify '
                  'the password file.')
            exit()

        # Provision required users
        self.required_users = {'admin':'admin', 'adminInfra':'admin'}
        
        if dev_mode:
            # TODO : Remove testUser later, temporarily inserting for
            #        maintaining compat with current test classes.
            self.required_users['testUser'] = 'testUser'
            for username in self.required_users.iterkeys():
                try:
                    self.getUser(username)
                except (KeyError, OSError, AttributeError):
                    self.addUser(username, self.required_users[username],
                                 provision=True)
        else:
            init_flag = True
            for username in self.required_users.iterkeys():
                try:
                    self.getUser(username)
                except (KeyError, OSError, AttributeError):
                    if init_flag:
                        print('It appears this is your first run or your '
                              'credentials database has changed or is '
                              'incomplete. You must set the passwords for the '
                              'Admin and Admin-Infrastructure accounts.')
                        init_flag = False
                    self.addUser(username, self.get_new_password(username),
                                 provision=True)
    
    def get_new_password(self, user):
        print('\nNote: The password must be between 4-20 characters long and '
              'contain at least one'
              '\n\t* lowercase,'
              '\n\t* uppercase,'
              '\n\t* digit'
              '\n\t* special character\n')
        
        msg_pw = "Enter a password for the user '{0}': ".format(user)
        msg_cf = "Please confirm the password for the user '{0}': ".format(user)
        
        while True:
            passwd = raw_input(msg_pw).strip()
            if passwd == raw_input(msg_cf).strip():
                if self.pass_validator(passwd):
                    return passwd
                else:
                    print "Password does not contain appropriate characters."
            else:
                print "Passwords do not match"

    def __getstate__(self):
        d = dict(vars(self))
        for k in '_credCache', '_cacheTimestamp':
            try:
                del d[k]
            except KeyError:
                pass
        return d

    def _cbPasswordMatch(self, matched, username):
        """ Internal method in case of success
        """
        if matched:
            return username
        else:
            return failure.Failure(error.UnauthorizedLogin())

    def _loadCredentials(self):
        """ Internal method to read file.
        """
        with open(self.filename) as f :
            for line in f:
                parts = self.scanner.match(line).groups()
                yield parts

    def getUser(self, username):
        """ Fetch username from db or cache, internal method
        """
        if (self._credCache is None or
            os.path.getmtime(self.filename) > self._cacheTimestamp):
            self._cacheTimestamp = os.path.getmtime(self.filename)
            self._credCache = dict(self._loadCredentials())
        return username, self._credCache[username]

    def requestAvatarId(self, c):
        try:
            u, p = self.getUser(c.username)
        except KeyError:
            return defer.fail(error.UnauthorizedLogin())
        else:
            return defer.maybeDeferred(c.checkPassword, p
                        ).addCallback(self._cbPasswordMatch, u)

    def addUser(self, username, password, provision=False):
        """ Change password for the username:
        
            @param username:    username
            @type  username:    str
            
            @param password:    password
            @type  password:    str
        
            @param provision:   Special flag to indicate provisioning mode
            @type  provision:   bool
        """
        if not (self.pass_validator(password) or provision):
            raise CredentialError(_PASSWORD_FAIL)
        
        if provision:
            with open(self.filename, 'a') as f:
                f.writelines('{0}:{1}\n'.format(username,
                                                sha256(password).digest()))
            return True
        
        try:
            self.getUser(username)
            raise CredentialError('Given user already exists')
        except KeyError:
            with open(self.filename, 'a') as f:
                f.writelines('{0}:{1}\n'.format(username,
                                                sha256(password).digest()))
            return True

    def removeUser(self, username):
        """ Remove the given users
        
            @param username:    username
            @type  username:    str
        """
        try:
            # why bother reading the file if the user doesn't even exist !
            self.getUser(username)
            
            for line in fileinput.input(self.filename, inplace=1):
                if self.scanner.match(line).groups()[0] != username:
                    print(line[:-1])
            return True
        except KeyError:
            raise CredentialError('No such user')

    def passwd(self,username, new_password, control_mode):
        """ Change password for the username.
            
            In admin mode you need to set the boolean indicating admin Mode.
            In case of a normal user you need to pass the old password for
            the user.
            
            Note : In admin mode, the admin is free to set any password he
                   likes as the strict password strength validator is turned
                   off in this case.
        
            @param username:        username
            @type  username:        str
            
            @param new_password:    new password
            @type  new_password:    str
            
            @param control_mode:    in user mode:  old password
                                    in admin mode: True
            @type  control_mode:    in user mode:  str
                                    in admin mode: bool
        """
        if type(control_mode)== bool :
            admin = True
            old_password = ''
        else:
            admin = False
            old_password = control_mode
            if not self.pass_validator(new_password):
                raise CredentialError(_PASSWORD_FAIL)
        try:
            # why bother reading the file if the user doesn't even exist !
            if (sha256(old_password).digest() == self.getUser(username)[1]
                or admin):
                for line in fileinput.input(self.filename, inplace=1):
                    if self.scanner.match(line).groups()[0] != username:
                        print(line[:-1])
                    else:
                        print('{0}:{1}'.format(username,
                                               sha256(new_password).digest()))
                return True
            else:
                raise CredentialError('Invalid User Password')
        except KeyError:
            raise CredentialError('No such user')


class RCEInternalChecker(RCECredChecker):
    """ TODO: Add doc
    """
    def add_checker(self,method):
        """ TODO: Add doc
        """
        self.checkUidValidity = method

    def requestAvatarId(self, c):
        try:
            if c.username in ('container','robot'):
                p = self.getUser('adminInfra')[1]
                user = c.username
            else: # it is the environment uuid
                try:
                    # this method is set by the real object instance
                    self.checkUidValidity(c.username)
                except CredentialError:
                    return defer.fail(error.UnauthorizedLogin())

                infra = self.getUser('adminInfra')[1]
                main = self.getUser('admin')[1]
                p = EncodeAES(cipher(main), salter(c.username,infra))
                user = 'environment'
        except KeyError:
            return defer.fail(error.UnauthorizedLogin())
        else:
            return defer.maybeDeferred(c.checkPassword, p
                        ).addCallback(self._cbPasswordMatch, user)
