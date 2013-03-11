#!/usr/bin/env python
# -*- coding: utf-8 -*-
#     
#     consoleclient.py
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
#     \author/s: Mayank Singh
#     
#     

#python specific imports
from sys import stdout
from urllib import urlencode
from urllib2 import urlopen, HTTPError
import sys, os, termios, tty, json

#twisted specific imports
from twisted.python.log import err, startLogging
from twisted.cred.credentials import UsernamePassword
from twisted.internet import reactor
from twisted.spread.pb import PBClientFactory
from twisted.internet import stdio
from twisted.conch.recvline import HistoricRecvLine
from twisted.conch.insults.insults import ServerProtocol
from twisted.python import usage

class userAddOptions(usage.Options):
    optParameters = [
        ["username", "u", None, "Username"],
        ["password", "p", None, "Password"]
    ]

class userRemoveOptions(usage.Options):
    optParameters = [
        ["username", "u", None, "Username"]
    ]

class userOptions(usage.Options):
    subCommands = [['add', None, userAddOptions, "Add User"],
                   ['remove', None, userRemoveOptions, "Remove User"],
                   ['update', None, userAddOptions, "Update User"]
    ]

class containerOptions(usage.Options):
    optParameters = [
        ["start", "s", None, "Start a Container"],
        ["stop", "t", None, "Stop a Container"],
        ["services", "v", None, "List services running on the container"],
        ["topics", "o", None, "List topics running on the container"]
    ]
    
class nodeStartOptions(usage.Options):
    optParameters = [
        ["ctag", "c", None, "Container tag"],
        ["ntag", "n", None, "Node tag"],
        ["pkg", "p", None, "Package"],
        ["exe", "e", None, "Executable"],
        ["args", "a", None, "Arguments"]
    ]

class nodeStopOptions(usage.Options):
    optParameters = [
        ["ctag", "c", None, "Container tag"],
        ["ntag", "n", None, "Node tag"]
    ]

class nodeOptions(usage.Options):
    subCommands = [['start', None, nodeStartOptions, "Start Node"],
                   ['stop', None, nodeStopOptions, "Stop Node"]]
                   
class parameterAddOptions(usage.Options):
    optParameters = [
        ["ctag", "c", None, "Container tag"],
        ["name", "n", None, "Name of parameter"],
        ["value", "v", None, "Value of parameter"]
    ]
    
class parameterRemoveOptions(usage.Options):
    optParameters = [
        ["ctag", "c", None, "Container tag"],
        ["name", "n", None, "Name of parameter"]
    ]

class parameterOptions(usage.Options):
    subCommands = [['add', None, parameterAddOptions, 'Add parameter'],
                   ['remove', None, parameterRemoveOptions, 'Remove parameter']]
                   
class interfaceAddOptions(usage.Options):
    optParameters = [
        ["etag", "e", None, "Endpoint tag"],
        ["itag", "i", None, "Interface tag"],
        ["itype", "t", None, "Interface type"],
        ["icls", "c", None, "Interface Class"],
        ["addr", "a", None, "Address"]
    ]

class interfaceRemoveOptions(usage.Options):
    optParameters = [
        ["etag", "e", None, "Endpoint tag"],
        ["itag", "i", None, "Interface tag"]
    ]

class interfaceOptions(usage.Options):
    subCommands = [['add', None, interfaceAddOptions, 'Add interface'],
                   ['remove', None, interfaceRemoveOptions, 'Remove interface']]


class connectionSubOptions(usage.Options):
    optParameters = [
        ["tag1", "1", None, "First Interface"],
        ["tag2", "2", None, "Second Interface"]
    ]

class connectionOptions(usage.Options):
    subCommands = [['add', None, connectionSubOptions, 'Connect Interfaces'],
                   ['remove', None, connectionSubOptions, 'Disconnect Interfaces']]

class robotOptions(usage.Options):
    optParameters = [
        ["start", "s", None, "Start a Robot"],
        ["stop", "t", None, "Stop a Robot"],
    ]
    optFlags = [
        ["list", "l", "List all Robots"]
    ]
                   
class ConsoleClient(HistoricRecvLine):
    from os import linesep as delimiter

    def __init__(self, masterIP):
        self._user = None
        self._masterIP = masterIP
        self._mode = "Username"
        self._username = None
        self._password = None
        self._factory = None
        self._connectedrosapinodes = {}

    def showPrompt(self):
        self.terminal.nextLine()
        self.terminal.write(self.ps[self.pn])

    def connectionMade(self):
        HistoricRecvLine.connectionMade(self)
        self._factory = PBClientFactory()

        reactor.connectTCP(self._masterIP, 8800, self._factory)
        self.terminal.write("Username: ")

    def parseInputLine(self, line):
        if line is not None and line is not '':
            func = getattr(self, 'cmd_' + line.split()[0].upper(), None)
            if func is not None:
                func(line.split()[1:])
            else:
                self.terminal.write('No such command')
        self.showPrompt()
    
    def callToRosProxy(self, command, parameter):
        def perform_action((url,key)):
            self._connectedrosapinodes[parameter] = (url,key)
            argList = [('userID', self._username), ('action', command), 
                             ('key', key)]
            try:
                f = urlopen('{0}?{1}'.format(url, urlencode(argList)))
                response = json.loads(f.read())
                self.terminal.write(str(response['key']))
            except HTTPError as e:
                msg = e.read()
                if msg:
                    msg = ' - {0}'.format(msg)

                self.terminal.write('HTTP Error {0}: '
                                    '{1}{2}'.format(e.getcode(), 
                                     e.msg, msg))

        try:
            url, key = self._connectedrosapinodes[parameter]
            perform_action((url, key))
        except KeyError:
            d = self._user.callRemote('get_rosapi_connect_info',
                                        parameter)
            d.addCallback(perform_action)
            d.addErrback(lambda err: self.terminal.write("Problem"
                                " in connection with master"+str(err)))
           
    def cmd_EXIT(self, line):
        reactor.stop()


    def cmd_USER(self, line):
        config = userOptions()
        try:
            config.parseOptions(line)
            if config.subCommand == 'add':
                self._user.callRemote('add_user', config.subOptions['username'],
                                       config.subOptions['password'])

            elif config.subCommand == 'remove':
                self._user.callRemote('remove_user', config.subOptions['username'])
            
            elif config.subCommand == 'update':
                self._user.callRemote('update_user', config.subOptions['username'],
                                       config.subOptions['password'])

        except usage.UsageError, errortext:
            self.terminal.write("BUG in usage: "+str(errortext))


    def cmd_CONTAINER(self, line):
        config = containerOptions()
        try:
            config.parseOptions(line)
            if config['start'] is not None:
                self._user.callRemote('start_container', config['start'])

            elif config['stop'] is not None:
                self._user.callRemote('stop_container', config['stop'])

            elif config['services'] is not None:
                self.callToRosProxy('services', config['services'])
            
            elif config['topics'] is not None:
                self.callToRosProxy('topics', config['topics'])

        except usage.UsageError, errortext:
            self.terminal.write("BUG in usage: "+str(errortext))
        
    def cmd_NODE(self, line):
        config = nodeOptions()
        try:
            config.parseOptions(line)
            if config.subCommand == 'start':

                if config.subOptions['args']:
                    self._user.callRemote('start_node', config.subOptions['ctag'], \
                    config.subOptions['ntag'], config.subOptions['pkg'], \
                    config.subOptions['exe'], config.subOptions['args'])
                else:
                    self._user.callRemote('start_node', config.subOptions['ctag'], \
                    config.subOptions['ntag'], config.subOptions['pkg'], \
                    config.subOptions['exe'])

            elif config.subCommand == 'stop':
                self._user.callRemote('stop_node', config.subOptions['ctag'], \
                config.subOptions['ntag'])

        except usage.UsageError, errortext:
            self.terminal.write("BUG in usage: "+str(errortext))

    def cmd_PARAMETER(self, line):
        config = parameterOptions()
        try:
            config.parseOptions(line)
            if config.subCommand == 'add':
                self._user.callRemote('add_parameter', \
                config.subOptions['ctag'], config.subOptions['name'], \
                config.subOptions['value'])
            elif config.subCommand == 'remove':
                self._user.callRemote('remove_parameter', \
                config.subOptions['ctag'], config.subOptions['name'])

        except usage.UsageError, errortext:
            self.terminal.write("BUG in usage: "+str(errortext))
    
    def cmd_INTERFACE(self, line):
        config = interfaceOptions()
        try:
            config.parseOptions(line)
            if config.subCommand == 'add':
            
                if config.subOptions['addr']:
                    self._user.callRemote('add_interface', \
                    config.subOptions['etag'], config.subOptions['itag'], \
                    config.subOptions['itype'], config.subOptions['icls'], \
                    config.subOptions['addr'])
                else:
                    self._user.callRemote('add_interface', \
                    config.subOptions['etag'], config.subOptions['itag'], \
                    config.subOptions['itype'], config.subOptions['icls'])
                    
            elif config.subCommand == 'remove':
                self._user.callRemote('remove_interface', \
                config.subOptions['etag'], config.subOptions['itag'])
                
        except usage.UsageError, errortext:
            self.terminal.write("BUG in usage: "+str(errortext))
    
    def cmd_CONNECTION(self, line):
        config = connectionOptions()
        try:
            config.parseOptions(line)
            if config.subCommand == 'add':
                self._user.callRemote('add_connection', \
                config.subOptions['tag1'], config.subOptions['tag2'])
            elif config.subCommand == 'remove':
                self._user.callRemote('remove_connection', \
                config.subOptions['tag1'], config.subOptions['tag2'])
        except usage.UsageError, errortext:
            self.terminal.write("BUG in usage: "+str(errortext))
                
    
    def cmd_ROBOT(self, line):
        config = robotOptions()
        try:
            config.parseOptions(line)
            if config['start'] is not None:
                self._user.callRemote('start_robot', config['start'])
            elif config['stop'] is not None:
                self._user.callRemote('stop_robot', config['stop'])
            elif config['list']:
                d = self._user.callRemote('list_robots')
                d.addCallback(lambda robots: str(self.terminal.write(robots)))
        except usage.UsageError, errortext:
            self.terminal.write("BUG in usage: "+str(errortext))

    def cmd_ROSTOPIC(self, line):
        config = rostopicOptions()
        try:
            config.parseOptions(line)
        except usage.UsageError, errortext:
            self.terminal.write("BUG in usage: "+str(errortext))

    def lineReceived(self, line):
        
        def cbError(why, msg):
            err(why, msg)
            reactor.stop()

        def cbConnected(perspective):
            self._user = perspective
            self.terminal.write('Connection to Master Established.'+str(perspective))
            self.showPrompt()
            
        if self._mode == 'Username':
            self._mode = 'Password'
            self._username = line
            self.terminal.write('Password: ')

        elif self._mode == 'Password':
            self._mode = 'Terminal'
            self._password = line
            usernameLogin = self._factory.login(UsernamePassword(self._username, self._password))
            usernameLogin.addCallback(cbConnected)
            usernameLogin.addErrback(cbError, "Username/password login failed")

        else:
            self.parseInputLine(line)

def runWithProtocol(klass, masterIP):
    fd = sys.stdin.fileno()
    oldSettings = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    try:
        p = ServerProtocol(klass, masterIP)
        stdio.StandardIO(p)
        reactor.run()
    finally:
        termios.tcsetattr(fd, termios.TCSANOW, oldSettings)
        os.write(fd, "\r\x1bc\r")

  
def _get_argparse():
    from argparse import ArgumentParser

    parser = ArgumentParser(prog='console',
                            description='RCE Monitoring terminal.')

    parser.add_argument('ipMaster', help='IP address of master process.',
                        type=str)

    return parser

def main():
    startLogging(sys.stdout)
    args = _get_argparse().parse_args()
    runWithProtocol(ConsoleClient, args.ipMaster)

    
if __name__ == '__main__':
    main()
