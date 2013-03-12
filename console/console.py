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
import getopt
from os import path

#twisted specific imports
from twisted.python.log import err, startLogging
from twisted.cred.credentials import UsernamePassword
from twisted.internet import reactor
from twisted.spread.pb import PBClientFactory
from twisted.internet import stdio
from twisted.conch.recvline import HistoricRecvLine
from twisted.conch.insults.insults import ServerProtocol
from twisted.python import usage
from twisted.python import reflect, text, util

class customOptions(usage.Options):
    def __init__(self, terminal):
        super(customOptions, self).__init__()
        self.terminal = terminal

    def parseOptions(self, options=None):
        """
        The guts of the command-line parser.
        """

        if options is None:
            options = sys.argv[1:]

        try:
            opts, args = getopt.getopt(options,
                                       self.shortOpt, self.longOpt)
        except getopt.error, e:
            raise usage.UsageError(str(e))

        for opt, arg in opts:
            if opt[1] == '-':
                opt = opt[2:]
            else:
                opt = opt[1:]

            optMangled = opt
            if optMangled not in self.synonyms:
                optMangled = opt.replace("-", "_")
                if optMangled not in self.synonyms:
                    raise usage.UsageError("No such option '%s'" % (opt,))

            optMangled = self.synonyms[optMangled]
            if isinstance(self._dispatch[optMangled], usage.CoerceParameter):
                self._dispatch[optMangled].dispatch(optMangled, arg)
            else:
                self._dispatch[optMangled](optMangled, arg)

        if (getattr(self, 'subCommands', None)
            and (args or self.defaultSubCommand is not None)):
            if not args:
                args = [self.defaultSubCommand]
            sub, rest = args[0], args[1:]
            for (cmd, short, parser, doc) in self.subCommands:
                if sub == cmd or sub == short:
                    self.subCommand = cmd
                    self.subOptions = parser(self.terminal)
                    self.subOptions.parent = self
                    self.subOptions.parseOptions(rest)
                    break
            else:
                raise usage.UsageError("Unknown command: %s" % sub)
        else:
            try:
                self.parseArgs(*args)
            except TypeError:
                raise usage.UsageError("Wrong number of arguments.")

        self.postOptions()

    def getSynopsis(self):
        """
        Returns a string containing a description of these options and how to
        pass them to the executed file.
        """

        if self.parent is None:
            command = self.__class__.__name__
            offset = command.find('Options')
            default = "Usage: %s%s" % (command[:offset],
                                       (self.longOpt and " [options]") or '')
        else:
            default = '%s' % ((self.longOpt and "[options]") or '')

        synopsis = getattr(self, "synopsis", default)

        synopsis = synopsis.rstrip()

        if self.parent is not None:
            synopsis = ' '.join((self.parent.getSynopsis(),
                                 self.parent.subCommand, synopsis))

        return synopsis

    def opt_help(self):
        self.terminal.write(self.__str__())

    def opt_version(self):
        self.terminal.write('vestigial option')
            
class userAddOptions(customOptions):
    optParameters = [
        ["username", "u", None, "Username"],
        ["password", "p", None, "Password"]
    ]

class userRemoveOptions(customOptions):
    optParameters = [
        ["username", "u", None, "Username"]
    ]

class userOptions(customOptions):
    subCommands = [['add', None, userAddOptions, "Add User"],
                   ['remove', None, userRemoveOptions, "Remove User"],
                   ['update', None, userAddOptions, "Update User"]
    ]
    optFlags = [
        ["list", "l", "List all Users"]
    ]
    
class containerOptions(customOptions):
    optParameters = [
        ["start", "s", None, "Start a Container"],
        ["stop", "t", None, "Stop a Container"],
        ["services", "v", None, "List services running on the container"],
        ["topics", "o", None, "List topics running on the container"],
        ["username", "u", None, "List containers by username"]
    ]
    optFlags = [
        ["list", "l", "List all containers of the user logged in"]
    ]
    
class nodeStartOptions(customOptions):
    optParameters = [
        ["ctag", "c", None, "Container tag"],
        ["ntag", "n", None, "Node tag"],
        ["pkg", "p", None, "Package"],
        ["exe", "e", None, "Executable"],
        ["args", "a", None, "Arguments"]
    ]

class nodeStopOptions(customOptions):
    optParameters = [
        ["ctag", "c", None, "Container tag"],
        ["ntag", "n", None, "Node tag"]
    ]

class nodeOptions(customOptions):
    subCommands = [['start', None, nodeStartOptions, "Start Node"],
                   ['stop', None, nodeStopOptions, "Stop Node"]]
                   
class parameterAddOptions(customOptions):
    optParameters = [
        ["ctag", "c", None, "Container tag"],
        ["name", "n", None, "Name of parameter"],
        ["value", "v", None, "Value of parameter"]
    ]
    
class parameterRemoveOptions(customOptions):
    optParameters = [
        ["ctag", "c", None, "Container tag"],
        ["name", "n", None, "Name of parameter"]
    ]

class parameterOptions(customOptions):
    subCommands = [['add', None, parameterAddOptions, 'Add parameter'],
                   ['remove', None, parameterRemoveOptions, 'Remove parameter']]
                   
class interfaceAddOptions(customOptions):
    optParameters = [
        ["etag", "e", None, "Endpoint tag"],
        ["itag", "i", None, "Interface tag"],
        ["itype", "t", None, "Interface type"],
        ["icls", "c", None, "Interface Class"],
        ["addr", "a", None, "Address"]
    ]

class interfaceRemoveOptions(customOptions):
    optParameters = [
        ["etag", "e", None, "Endpoint tag"],
        ["itag", "i", None, "Interface tag"]
    ]


class interfaceOptions(customOptions):
    subCommands = [['add', None, interfaceAddOptions, 'Add interface'],
                   ['remove', None, interfaceRemoveOptions, 'Remove interface']]


class connectionSubOptions(customOptions):
    optParameters = [
        ["tag1", "1", None, "First Interface"],
        ["tag2", "2", None, "Second Interface"]
    ]


class connectionOptions(customOptions):
    subCommands = [['add', None, connectionSubOptions, 'Connect Interfaces'],
                   ['remove', None, connectionSubOptions, 'Disconnect Interfaces']]


class robotOptions(customOptions):
    optParameters = [
        ["username", "u", None, "List Robots by Username"]
    ]
    optFlags = [
        ["list", "l", "List all Robots"]
    ]


class machineOptions(customOptions):
    optParameters = [
        ["stats", "s", None, "Statistics of Machine by IP"],
        ["containers", "c", None, "List Containers by Machine's IP"]
    ]
    optFlags = [
        ["list", "l", "List all Machines"]
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
    
    
    def callToUser(self, command, *args):
        self._user.callRemote(command, *args)
    
    
    def callToUserAndDisplay(self, command, *args):
        d = self._user.callRemote(command, *args)
        d.addCallback(lambda result: self.terminal.write(str(result)))


    def cmd_EXIT(self, line):
        reactor.stop()


    def cmd_USER(self, line):
        config = userOptions(self.terminal)
        try:
            config.parseOptions(line)
            if config.subCommand == 'add':
                if config.subOptions['username'] and \
                   config.subOptions['password']:
                    self.callToUser('add_user', config.subOptions['username'],
                                           config.subOptions['password'])

            elif config.subCommand == 'remove':
                if config.subOptions['username']:
                    self.callToUser('remove_user', config.subOptions['username'])
            
            elif config.subCommand == 'update':
                if config.subOptions['username'] and \
                config.subOptions['password']:
                    self.callToUser('update_user', config.subOptions['username'],
                                           config.subOptions['password'])
                                       
            elif config['list']:
                self.callToUserAndDisplay('list_users')

        except usage.UsageError, errortext:
            self.terminal.write("BUG in usage: "+str(errortext))


    def cmd_CONTAINER(self, line):
        config = containerOptions(self.terminal)
        try:
            config.parseOptions(line)
            if config['start'] is not None:
                self.callToUser('start_container', config['start'])

            elif config['stop'] is not None:
                self.callToUser('stop_container', config['stop'])

            elif config['services'] is not None:
                self.callToRosProxy('services', config['services'])
            
            elif config['topics'] is not None:
                self.callToRosProxy('topics', config['topics'])
            
            elif config['list']:
                self.callToUserAndDisplay('list_containers')

            elif config['username'] is not None:
                self.callToUserAndDisplay('list_containers_by_user', 
                                           config['username'])

        except usage.UsageError, errortext:
            self.terminal.write("BUG in usage: "+str(errortext))

        
    def cmd_NODE(self, line):
        config = nodeOptions(self.terminal)
        try:
            config.parseOptions(line)
            if config.subCommand == 'start':

                if config.subOptions['args'] and config.subOptions['ctag'] and \
                   config.subOptions['ntag'] and config.subOptions['pkg'] and \
                   config.subOptions['exe']:
                    self.callToUser('start_node', config.subOptions['ctag'], 
                    config.subOptions['ntag'], config.subOptions['pkg'], 
                    config.subOptions['exe'], config.subOptions['args'])

                elif config.subOptions['ctag'] and config.subOptions['ntag'] and\
                     config.subOptions['pkg'] and config.subOptions['exe']:
                    self.callToUser('start_node', config.subOptions['ctag'], 
                    config.subOptions['ntag'], config.subOptions['pkg'], 
                    config.subOptions['exe'])

            elif config.subCommand == 'stop':
                if config.subOptions['ctag'] and config.subOptions['ntag']:
                    self.callToUser('stop_node', config.subOptions['ctag'], 
                    config.subOptions['ntag'])

        except usage.UsageError, errortext:
            self.terminal.write("BUG in usage: "+str(errortext))

    def cmd_PARAMETER(self, line):
        config = parameterOptions(self.terminal)
        try:
            config.parseOptions(line)
            if config.subCommand == 'add':
                if config.subOptions['ctag'] and config.subOptions['name'] and\
                config.subOptions['value']:
                    self.callToUser('add_parameter', 
                    config.subOptions['ctag'], config.subOptions['name'], 
                    config.subOptions['value'])

            elif config.subCommand == 'remove':
                if config.subOptions['ctag'] and config.subOptions['name']:
                    self.callToUser('remove_parameter', 
                    config.subOptions['ctag'], config.subOptions['name'])

        except usage.UsageError, errortext:
            self.terminal.write("BUG in usage: "+str(errortext))
    
    def cmd_INTERFACE(self, line):
        config = interfaceOptions(self.terminal)
        try:
            config.parseOptions(line)
            if config.subCommand == 'add':
            
                if config.subOptions['addr'] and config.subOptions['etag'] and\
                   config.subOptions['itag'] and config.subOptions['itype']\
                   and config.subOptions['icls']:
                    self.callToUser('add_interface', 
                    config.subOptions['etag'], config.subOptions['itag'], 
                    config.subOptions['itype'], config.subOptions['icls'], 
                    config.subOptions['addr'])

                elif config.subOptions['etag'] and config.subOptions['itag'] and\
                     config.subOptions['itype'] and config.subOptions['icls']:
                    self.callToUser('add_interface', 
                    config.subOptions['etag'], config.subOptions['itag'], 
                    config.subOptions['itype'], config.subOptions['icls'])
                    
            elif config.subCommand == 'remove':
                if config.subOptions['etag'] and config.subOptions['itag']:
                    self.callToUser('remove_interface', 
                    config.subOptions['etag'], config.subOptions['itag'])
                
        except usage.UsageError, errortext:
            self.terminal.write("BUG in usage: "+str(errortext))
    
    def cmd_CONNECTION(self, line):
        config = connectionOptions(self.terminal)
        try:
            config.parseOptions(line)
            if config.subCommand == 'add':
                if config.subOptions['tag1'] and config.subOptions['tag2']:
                    self.callToUser('add_connection', 
                    config.subOptions['tag1'], config.subOptions['tag2'])

            elif config.subCommand == 'remove':
                if config.subOptions['tag1'] and config.subOptions['tag2']:
                    self.callToUser('remove_connection', 
                    config.subOptions['tag1'], config.subOptions['tag2'])

        except usage.UsageError, errortext:
            self.terminal.write("BUG in usage: "+str(errortext))
                
    
    def cmd_ROBOT(self, line):
        config = robotOptions(self.terminal)
        try:
            config.parseOptions(line)

            if config['list']:
                self.callToUserAndDisplay('list_robots')
                
            elif config['username'] is not None:
                self.callToUserAndDisplay('list_robots_by_user', 
                                           config['username'])

        except usage.UsageError, errortext:
            self.terminal.write("BUG in usage: "+str(errortext))

    def cmd_MACHINE(self, line):
        config = machineOptions(self.terminal)
        try:
            config.parseOptions(line)
            if config['list']:
                self.callToUserAndDisplay('list_machines')
            
            elif config['stats'] is not None:
                self.callToUserAndDisplay('stats_machine', config['stats'])
            
            elif config['containers'] is not None:
                self.callToUserAndDisplay('machine_containers', config['containers'])

        except usage.UsageError, errortext:
            self.terminal.write("BUG in usage: "+str(errortext))

    def cmd_HELP(self, line):
        configs = [userOptions(self.terminal), containerOptions(self.terminal),
                   nodeOptions(self.terminal), parameterOptions(self.terminal),
                   interfaceOptions(self.terminal), connectionOptions(self.terminal),
                   robotOptions(self.terminal), machineOptions(self.terminal)]
        for config in configs:
            self.terminal.nextLine()
            config.opt_help()


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
