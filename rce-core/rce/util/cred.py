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

# Python specific imports
import os
import fileinput
import re
import base64
from hashlib import sha256
from Crypto.Cipher import AES
from collections import namedtuple

# zope specific imports
from zope.interface import implements

# twisted specific imports
from twisted.internet import defer
from twisted.python import failure
from twisted.cred import error
from twisted.cred.credentials import IUsernameHashedPassword
from twisted.cred.checkers import ICredentialsChecker

# rce specific imports
from rce.util.name import validateName, IllegalName


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
encodeAES = lambda c, s: base64.b64encode(c.encrypt(pad(s)))
cipher = lambda passwd: AES.new(passwd.decode('hex'))
salter = lambda u, p: sha256(u + p).hexdigest()

# one-liner to format user-info to write to the credentials file
formatUser = lambda name, pw, mode, groups: '\t'.join((name, pw, mode,
                                                       ':'.join(groups)))

# Cloud engine specific Types and Declarations
UserInfo = namedtuple('UserInfo', 'password mode groups')

# User mode mask length, modify these for future adaptations
_MODE_LENGTH = 1
_DEFAULT_USER_MODE = '1'  # should be as many digits as the above e.g.: 1, 01

# default groups a user belongs to
_DEFAULT_GROUPS = ('user',)

# Used Regex patterns
_RE = r'(\w+)\s([0-9a-fA-F]{64})\s(\d{' + str(_MODE_LENGTH) + '})\s([\w:]+)$'
_PASS_RE = r'^.*(?=.{4,20})(?=.*[a-z])(?=.*[A-Z])(?=.*[\d])(?=.*[\W]).*$'

# Used doc strings
_PASSWORD_FAIL = ('Password must be between 4-20 digits long and has to '
                  'contain at least one uppercase, lowercase, digit, and '
                  'special character. No whitespace allowed.')
_FIRST_RUN_MSG = ('It appears this is your first run or your credentials '
                  'database has changed or is incomplete. You must set the '
                  'passwords for the Admin and Admin-Infrastructure accounts.')
_NEW_PASS_PROMPT = ('\nNote: The password must be between 4-20 characters long '
                    'and contain no whitespace and at least one'
                    '\n\t* lowercase,'
                    '\n\t* uppercase,'
                    '\n\t* digit'
                    '\n\t* special character\n')


class CredentialError(Exception):
    """ Exception is raised if the CredChecker encountered an error.
    """


class RCECredChecker(object):
    """The RCE file-based, text-based username/password database.
    """
    implements(ICredentialsChecker)

    credentialInterfaces = (IUsernameHashedPassword,)

    cache = False
    _credCache = None
    _cacheTimestamp = 0

    def __init__(self, pw_file, provision=False):
        """ Initialize the credentials checker for the RoboEarth Cloud Engine.

            @param pw_file:     Path to the credentials database.
            @type  pw_file:     str

            @param provision:   Flag which is set if the database is going to
                                be provisioned.
            @type  provision:   bool
        """
        self.filename = pw_file
        self.scanner = re.compile(_RE)
        pass_re = re.compile(_PASS_RE)
        self.pass_validator = lambda x: True if pass_re.match(x) else False

        # Run some basic tests to check if the settings file is valid
        if self.filename is None:
            print('Settings variable PASSWORD_FILE not set')
            exit()

        if not os.access(os.path.dirname(self.filename), os.W_OK):
            print('The user lacks privileges to access/modify '
                  'the password file.')
            exit()

        if not provision:
            if not os.path.exists(self.filename):
                print('Credential file missing please run the provision script '
                      'first.')
                exit()

    def get_new_password(self, user):
        """ Method which is used to interactively get a new password from the
            user.

            @param user:        User ID of the user for which a new password
                                has to be entered.
            @type  user:        str

            @return:            The new password for the user.
            @rtype:             str
        """
        print (_NEW_PASS_PROMPT)
        msg_pw = "Enter a password for the user '{0}': ".format(user)
        msg_cf = "Please confirm the password for the user '{0}': ".format(user)

        while True:
            passwd = raw_input(msg_pw).strip()
            if passwd == raw_input(msg_cf).strip():
                if ' ' not in passwd and self.pass_validator(passwd):
                    return passwd
                else:
                    print('Password does not contain appropriate characters.')
            else:
                print('Passwords do not match.')

    def _cbPasswordMatch(self, matched, username):
        """ Internal method which is called in case the password could be
            successfully matched.
        """
        if matched:
            return username
        else:
            return failure.Failure(error.UnauthorizedLogin())

    def _loadCredentials(self):
        """ Internal method to read the credentials database.
        """
        with open(self.filename) as f:
            for line in f:
                try:
                    parts = self.scanner.match(line).groups()
                except AttributeError:
                    raise CredentialError('Credential database corrupted')
                try:
                    yield parts[0], UserInfo(parts[1], int(parts[2]),
                                             set(parts[3].split(':')))
                except KeyError:
                    raise CredentialError('Credential database corrupted')

    def getUser(self, username):
        """ Fetch username from db or cache. (Internal method)
        """
        if (self._credCache is None or
            os.path.getmtime(self.filename) > self._cacheTimestamp):
            self._cacheTimestamp = os.path.getmtime(self.filename)
            self._credCache = dict(self._loadCredentials())
        return self._credCache[username]

    def getUserMode(self, username):
        """ Fetch mode for a user.

            @param username:    username for who the mode is to be set.
            @type  username:    str

            @return:            Mode
            @rtype:             int
        """
        return self.getUser(username).mode

    def getUserGroups(self, username):
        """ Fetch groups for a user.

            @param username:    username for who the mode is to be set.
            @type  username:    str

            @return:            groups
            @rtype:             set
        """
        return self.getUser(username).groups

    def userMemebership(self, username, group):
        """ Test if a user is member of a certain group

            @param username:    username of the user
            @type  username:    str

            @param group:       group for which membership is to be tested
                                of the user
            @type  group:       str

            @return:            Result indicating membership
            @rtype:             bool
        """
        return group in self.getUserGroups(username)

    def requestAvatarId(self, c):
        try:
            passwd = self.getUser(c.username).password
        except KeyError:
            return defer.fail(error.UnauthorizedLogin())
        else:
            return defer.maybeDeferred(c.checkPassword, passwd
                        ).addCallback(self._cbPasswordMatch, c.username)

    def setUserMode(self, username, mode):
        """ Set the mode for a user

            Default mode for a user is 1. Like in Unix Admin mode is 0.
            The rest are open to custom extensions later in the engine.

            @param username:    username for who the mode is to be set.
            @type  username:    str

            @param mode:        Mode length is set by the _MODE_LENGTH [1]
                                parameter (1 for user; 0 for admin)
            @type  mode:        int

            @return:            Result of Operation
            @rtype:             bool
        """
        # mode sanity check and ensure only a single digit
        mode = str(mode)
        if len(mode) != _MODE_LENGTH:
            raise CredentialError('Invalid Mode Length')
        try:
            props = self.getUser(username)
        except KeyError:
            raise CredentialError('No such user')
        # Now process the file in the required mode
        for line in fileinput.input(self.filename, inplace=1):
                if  self.scanner.match(line).groups()[0] != username:
                    print(line[:-1])
                else:
                    print(formatUser(username, props.password, mode,
                                     props.groups))
        return True

    def addUserGroups(self, username, *groups):
        """ Add Group membership to certain groups.

            @param username:    username for who the mode is to be set.
            @type  username:    str

            @param groups:      groups membership to add to user
            @type  groups:      csv strings, e.g. group1,group2

            @return:            Result of Operation
            @rtype:             bool
        """
        # mode sanity check and ensure only a single digit
        try:
            props = self.getUser(username)
        except KeyError:
            raise CredentialError('No such user')
        groups = set(groups).union(props.groups)
        # Now process the file in the required mode
        for line in fileinput.input(self.filename, inplace=1):
                if  self.scanner.match(line).groups()[0] != username:
                    print(line[:-1])
                else:
                    print(formatUser(username, props.password, str(props.mode),
                                     groups))
        return True

    def removeUserGroups(self, username, *groups):
        """ Remove Group membership to certain groups.

            @param username:    username for who the mode is to be set.
            @type  username:    str

            @param groups:      groups membership to remove from the user
            @type  groups:      csv strings eg : group1,group2

            @return:            Result of Operation
            @rtype:             bool
        """
        # mode sanity check and ensure only a single digit
        try:
            props = self.getUser(username)
        except KeyError:
            raise CredentialError('No such user')
        groups = props.groups - set(groups)
        # Now process the file in the required mode
        for line in fileinput.input(self.filename, inplace=1):
                if  self.scanner.match(line).groups()[0] != username:
                    print(line[:-1])
                else:
                    print(formatUser(username, props.password, str(props.mode),
                                     groups))
        return True

    def addUser(self, username, password, provision=False):
        """ Change password for the username:

            @param username:    username
            @type  username:    str

            @param password:    password
            @type  password:    str

            @param provision:   Special flag to indicate provisioning mode
            @type  provision:   bool

            @return:            Result of Operation
            @rtype:             bool
        """
        try:
            validateName(username)
        except IllegalName as e:
            raise CredentialError(str(e))

        if not (self.pass_validator(password) or provision):
            raise CredentialError(_PASSWORD_FAIL)

        if provision:
            with open(self.filename, 'a') as f:
                f.write(formatUser(username, sha256(password).hexdigest(),
                                   _DEFAULT_USER_MODE, _DEFAULT_GROUPS))
                f.write('\n')
            return True

        try:
            self.getUser(username)
            raise CredentialError('Given user already exists')
        except KeyError:
            with open(self.filename, 'a') as f:
                f.write(formatUser(username, sha256(password).hexdigest(),
                                   _DEFAULT_USER_MODE, _DEFAULT_GROUPS))
                f.write('\n')
            return True

    def removeUser(self, username):
        """ Remove the given users

            @param username:    username
            @type  username:    str

            @return:            Result of Operation
            @rtype:             bool
        """
        try:
            self.getUser(username)
            for line in fileinput.input(self.filename, inplace=1):
                if self.scanner.match(line).groups()[0] != username:
                    print(line[:-1])
        except KeyError:
                raise CredentialError('No such user')

    def passwd(self, username, new_password, control_mode):
        """ Change password for the username.

            In admin mode you need to set the boolean indicating admin Mode.
            In case of a normal user you need to pass the old password for
            the user.

            Note : In admin mode, the admin is free to set any password he
                   likes as the strict password strength validation is turned
                   off in this case.

            @param username:        username
            @type  username:        str

            @param new_password:    new password
            @type  new_password:    str

            @param control_mode:    in user mode:  old password
                                    in admin mode: True
            @type  control_mode:    in user mode:  str
                                    in admin mode: bool

            @return:                Result of Operation
            @rtype:                 bool
        """
        try:
            props = self.getUser(username)
            if isinstance(control_mode, str):
                if props.password != sha256(control_mode).hexdigest():
                    raise CredentialError('Invalid Password')
                if not self.pass_validator(new_password):
                    raise CredentialError(_PASSWORD_FAIL)
        except KeyError:
            raise CredentialError('No such user')
        # Now process the file in the required mode
        for line in fileinput.input(self.filename, inplace=1):
            if  self.scanner.match(line).groups()[0] != username:
                print(line[:-1])
            else:
                print(formatUser(username, sha256(new_password).hexdigest(),
                                 str(props.mode), props.groups))
        return True


class RCEInternalChecker(object):
    """ RCE Internal Auth system
    """
    implements(ICredentialsChecker)

    def __init__(self, cred_checker):
        """
            @param cred_checker:    Cred Checker used to authenticate the cloud
                                    engine
            @type  cred_checker:    rce.util.cred.RCECredChecker
        """
        self._root_checker = cred_checker
        self.credentialInterfaces = (IUsernameHashedPassword,)

    def add_checker(self, method):
        """ Add a method to check the validity of a UUID.

            This method belongs to the RCE Realm Object, has access to the
            valid uuid and checks them, and raises a CredentialError if absent.
        """
        self.checkUidValidity = method

    def _cbPasswordMatch(self, matched, username):
        """ Internal method in case of success
        """
        if matched:
            return username
        else:
            return failure.Failure(error.UnauthorizedLogin())

    def requestAvatarId(self, c):
        try:
            if c.username in ('container', 'robot'):
                p = self._root_checker.getUser('adminInfra').password
                user = c.username
            else:  # it is the environment uuid
                try:
                    # this method is set by the real object instance
                    self.checkUidValidity(c.username)
                except CredentialError:
                    return defer.fail(error.UnauthorizedLogin())

                infra = self._root_checker.getUser('adminInfra').password
                main = self._root_checker.getUser('admin').password
                p = encodeAES(cipher(main), salter(c.username, infra))
                user = 'environment'
        except KeyError:
            return defer.fail(error.UnauthorizedLogin())
        else:
            return defer.maybeDeferred(c.checkPassword, p
                        ).addCallback(self._cbPasswordMatch, user)

