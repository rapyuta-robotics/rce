#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     rce-console/rce/console/console.py
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

# Python specific imports
import sys
import os
import termios
import tty
import json
import getopt
from urllib import urlencode
from urllib2 import urlopen, HTTPError
from hashlib import sha256

# twisted specific imports
from twisted.python import usage
from twisted.python.log import err, startLogging
from twisted.cred.credentials import UsernamePassword
from twisted.internet import reactor
from twisted.internet import stdio
from twisted.spread.pb import PBClientFactory
from twisted.conch.recvline import HistoricRecvLine
from twisted.conch.insults.insults import ServerProtocol


class CustomOptions(usage.Options):
    """ Custom Class to override functionality of usage.Options
    """
    def __init__(self, terminal):
        """ Initialize the CustomOptions Class

            @param terminal:    Reference to the running terminal
            @type  terminal:    ConsoleClient.terminal
        """
        super(CustomOptions, self).__init__()
        self.longdesc = ""
        self.terminal = terminal

    def parseOptions(self, options=None):
        """
        The guts of the command-line parser.
        """
        if options is None:
            options = sys.argv[1:]

        try:
            opts, args = getopt.getopt(options, self.shortOpt, self.longOpt)
        except getopt.error as e:
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
                    raise usage.UsageError("No such option '{0}'".format(opt))

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
            for (cmd, short, parser, _) in self.subCommands:
                if sub == cmd or sub == short:
                    self.subCommand = cmd
                    self.subOptions = parser(self.terminal)
                    self.subOptions.parent = self
                    self.subOptions.parseOptions(rest)
                    break
            else:
                raise usage.UsageError("Unknown command: {0}".format(sub))
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
            default = "Usage: {0}{1}".format(command[:offset],
                                       (self.longOpt and " [options]") or '')
        else:
            default = (self.longOpt and " [options]") or ''

        synopsis = getattr(self, "synopsis", default).rstrip()

        if self.parent is not None:
            synopsis = ' '.join((self.parent.getSynopsis(),
                                 self.parent.subCommand, synopsis))

        return synopsis

    def opt_help(self):
        """help option"""
        self.terminal.write(self.__str__())

    def opt_version(self):
        self.terminal.write('vestigial option')


# Various Option Classes follow
class UserAddOptions(CustomOptions):
    """
        Parameters for user add.
    """
    optParameters = (
        ("username", "u", None, "Username"),
        ("password", "p", None, "Password"),
    )


class UserRemoveOptions(CustomOptions):
    """
        Parameters for user remove.
    """
    optParameters = (
        ("username", "u", None, "Username"),
    )


class UserUpdateOptions(CustomOptions):
    """
        Parameters for user update.
    """
    optParameters = (
        ("username", "u", None, "Username"),
        ("password", "p", None, "New Password"),
    )

class UserChangePasswordOptions(CustomOptions):
    """
        Parameters for changing password by a non-admin.
    """
    optParameters = (
        ("new", "p", None, "New Password"),
        ("old", "o", None, "Old Password"),
    )

class UserOptions(CustomOptions):
    """
        Options for user command.
    """
    subCommands = (
        ('add', None, UserAddOptions, "Add User"),
        ('remove', None, UserRemoveOptions, "Remove User"),
        ('update', None, UserUpdateOptions, "Update User"),
        ('passwd', None, UserChangePasswordOptions, "Change Password"),
    )
    optFlags = (
        ("list", "l", "List all Users"),
    )


class ContainerStartOptions(CustomOptions):
    """
        Parameters for container start.
    """
    optParameters = (
        ("name", "n", None , "Container Name"),
        ("group", "g", None, "Container Group"),
        ("groupIp", "a", None , "Container Group IPv4 address"),
        ("size", "s", None , "Container Size"),
        ("cpu", "c", None , "CPU options"),
        ("memory", "m", None , "memory options"),
        ("bandwidth", "b", None , "Bandwidth options"),
        ("specialopts", "o", None , "Special features options"),
    )


class ContainerOptions(CustomOptions):
    """
        Options for container command.
    """
    optParameters = (
        ("stop", "t", None, "Stop a Container"),
        ("services", "v", None, "List services running on the container"),
        ("topics", "o", None, "List topics running on the container"),
        ("username", "u", None, "List containers by username"),
    )
    optFlags = (
        ("list", "l", "List all containers of the user logged in"),
    )

    subCommands = (
        ("start", None, ContainerStartOptions, "Start a Container"),
    )


class NodeStartOptions(CustomOptions):
    """
        Parameters for node start.
    """
    optParameters = (
        ("ctag", "c", None, "Container tag"),
        ("ntag", "n", None, "Node tag"),
        ("pkg", "p", None, "Package"),
        ("exe", "e", None, "Executable"),
        ("args", "a", None, "Arguments"),
    )


class NodeStopOptions(CustomOptions):
    """
        Parameters for node stop.
    """
    optParameters = (
        ("ctag", "c", None, "Container tag"),
        ("ntag", "n", None, "Node tag"),
    )


class NodeOptions(CustomOptions):
    """
        Options for node command.
    """
    subCommands = (
        ('start', None, NodeStartOptions, "Start Node"),
        ('stop', None, NodeStopOptions, "Stop Node"),
    )


class ParameterAddOptions(CustomOptions):
    """
        Parameters for adding ROS parameters.
    """
    optParameters = (
        ("ctag", "c", None, "Container tag"),
        ("name", "n", None, "Name of parameter"),
        ("value", "v", None, "Value of parameter"),
    )


class ParameterRemoveOptions(CustomOptions):
    """
        Parameters for removing ROS parameters.
    """
    optParameters = (
        ("ctag", "c", None, "Container tag"),
        ("name", "n", None, "Name of parameter"),
    )


class ParameterOptions(CustomOptions):
    """
        Options for parameter command.
    """
    subCommands = (
        ('add', None, ParameterAddOptions, 'Add parameter'),
        ('remove', None, ParameterRemoveOptions, 'Remove parameter'),
    )


class InterfaceAddOptions(CustomOptions):
    """
        Parameters for adding interfaces.
    """
    optParameters = (
        ("etag", "e", None, "Endpoint tag"),
        ("itag", "i", None, "Interface tag"),
        ("itype", "t", None, "Interface type"),
        ("icls", "c", None, "Interface Class"),
        ("addr", "a", None, "Address"),
    )


class InterfaceRemoveOptions(CustomOptions):
    """
        Parameters for removing interfaces.
    """
    optParameters = (
        ("etag", "e", None, "Endpoint tag"),
        ("itag", "i", None, "Interface tag"),
    )


class InterfaceOptions(CustomOptions):
    """
        Options for interface command.
    """
    subCommands = (
        ('add', None, InterfaceAddOptions, 'Add interface'),
        ('remove', None, InterfaceRemoveOptions, 'Remove interface'),
    )


class ConnectionSubOptions(CustomOptions):
    """
        Sub options for connection subcommands.
    """
    optParameters = (
        ("tag1", "1", None, "First Interface"),
        ("tag2", "2", None, "Second Interface"),
    )


class ConnectionOptions(CustomOptions):
    """
        Options for connection command.
    """
    subCommands = (
        ('add', None, ConnectionSubOptions, 'Connect Interfaces'),
        ('remove', None, ConnectionSubOptions, 'Disconnect Interfaces'),
    )


class RobotOptions(CustomOptions):
    """
        Options for robot command.
    """
    optParameters = (
        ("username", "u", None, "List Robots by Username"),
    )
    optFlags = (
        ("list", "l", "List all Robots"),
    )


class MachineOptions(CustomOptions):
    """
        Options for machine command.
    """
    optParameters = (
        ("stats", "s", None, "Statistics of Machine by IP"),
        ("containers", "c", None, "List Containers by Machine's IP"),
    )
    optFlags = (
        ("list", "l", "List all Machines"),
    )


def _errorHandle(func):
        def call(self, *args, **kwargs):
            try:
                func(self, *args, **kwargs)
            except AttributeError:
                self.terminal.write("Cannot use that command.")
        return call


class ConsoleClient(HistoricRecvLine):
    """ The class creates the terminal and manages connections with Master
        and ROSAPI servers on specific containers
    """
    def __init__(self, masterIP, consolePort):
        """ Initialize the ConsoleClient.

            @param masterIP:        The IP of the master server
            @type  masterIP:        string

            @param consolePort:     Port of the master server
            @type  consolePort:     int
        """
        self._user = None
        self._masterIP = masterIP
        self._console_port = consolePort
        self._mode = "Username"
        self._username = None
        self._password = None
        self._factory = None
        self._connected_rosapi_nodes = {}
        self._privilege = None

    def showPrompt(self):
        """ Show the prompt >>>
        """
        self.terminal.nextLine()
        self.terminal.write(self.ps[self.pn])

    def connectionMade(self):
        """ Create a PBClientFactory and connect to master when ConsoleClient
            connected to StandardIO. Prompt user for Username
        """
        HistoricRecvLine.connectionMade(self)
        self._factory = PBClientFactory()

        reactor.connectTCP(self._masterIP, self._console_port, self._factory)  #@UndefinedVariable
        self.terminal.write("Username: ")

    def lineReceived(self, line):
        """ Manage state/mode after connection. Code uses states to take
            credential input and then starts terminal input.

            @param line:    line typed on terminal
            @type  line:    string
        """
        def _cbError(why, msg):
            err(why, msg)
            reactor.stop()  #@UndefinedVariable

        def _cbConnectionSuccess(view):
            self._user = view

            if isinstance(self._user, dict):
                self._privilege = 'console'
            else:
                self._privilege = 'admin'

            self.terminal.write('Connection to Master Established.')
            self.showPrompt()

        if self._mode == 'Username':
            self._mode = 'Password'
            self._username = line
            self.terminal.write('Password: ')
        elif self._mode == 'Password':
            self._mode = 'Terminal'
            self._password = line
            cred = UsernamePassword(self._username,
                                    sha256(self._password).hexdigest())
            d = self._factory.login(cred)
            d.addCallback(lambda p: p.callRemote("getUserView", True))
            d.addCallback(_cbConnectionSuccess)
            d.addErrback(_cbError, "Username/password login failed")
        else:
            self.parseInputLine(line)

    def parseInputLine(self, line):
        """ A function to route various commands entered via Console.

            @param line:    The text entered on the Console
            @type  line:    string
        """
        if line is not None and line is not '':
            func = getattr(self, 'cmd_' + line.split()[0].upper(), None)
            if func is not None:
                func(line.split()[1:])
            else:
                self.terminal.write('No such command')
        self.showPrompt()

    @_errorHandle
    def callToRosProxy(self, command, parameter):
        """ Function to handle call to ROSAPI Proxy Server.

            @param command:      The command to execute in ROS environment.
            @type  command:      string

            @param parameter:    A parameter for the command.
            @type  parameter:    string
        """
        def perform_action((url, key)):
            self._connected_rosapi_nodes[parameter] = (url, key)
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
                                    '{1}{2}'.format(e.getcode(), e.msg, msg))

        try:
            url, key = self._connected_rosapi_nodes[parameter]
            perform_action((url, key))
        except KeyError:
            d = self._user['console'].callRemote('get_rosapi_connect_info',
                                                 parameter)
            d.addCallback(perform_action)
            d.addErrback(lambda err: self.terminal.write("Problem "
                                "in connection with master: "
                                "{0}".format(err)))

    @_errorHandle
    def callToUser(self, command, domain, *args):
        """ A wrapper function for call to remote user.

            @param command:    The command to be executed
            @type  command:    string
        """
        if domain == 'admin':
            self._user.callRemote(command, *args)
        else:
            self._user[domain].callRemote(command, *args)

    @_errorHandle
    def callToUserAndDisplay(self, command, domain, *args):
        """ A wrapper function around call to user and displaying the result

            @param command:    The command to be executed
            @type  command:    string
        """
        if domain == 'admin':
            d = self._user.callRemote(command, *args)
            d.addCallback(lambda result: self.terminal.write(str(result)))
        else:
            d = self._user[domain].callRemote(command, *args)
            d.addCallback(lambda result: self.terminal.write(str(result)))

    # Various commands follow
    def cmd_EXIT(self, line):
        """ Handler for exit command.

            @param line:    line input from terminal.
            @type  line:    string
        """
        reactor.stop()  # @UndefinedVariable

    def cmd_USER(self, line):
        """ Handler for user command.

            @param line:    line input from terminal.
            @type  line:    string
        """
        config = UserOptions(self.terminal)

        try:
            config.parseOptions(line)
            cmd = config.subCommand
            opts = config.subOptions if hasattr(config, 'subOptions') else {}
        except usage.UsageError as errortext:
            self.terminal.write("BUG in usage: {0}".format(errortext))
        else:
            if cmd == 'add':
                if opts['username'] and opts['password']:
                    self.callToUser('add_user', 'admin', opts['username'],
                                    opts['password'])
            elif cmd == 'remove':
                if opts['username']:
                    self.callToUser('remove_user', 'admin', opts['username'])
            elif cmd == 'update':
                if opts['username'] and opts['password']:
                    self.callToUser('update_user', 'admin',
                                    opts['username'], opts['password'])
            elif cmd == 'passwd':
                if opts['new'] and opts['old']:
                    self.callToUser('update_user', 'console', opts['new'],
                                    opts['old'])
            elif config['list']:
                self.callToUserAndDisplay('list_users', 'admin')

    def cmd_CONTAINER(self, line):
        """ Handler for container command.

            @param line:    line input from terminal.
            @type  line:    string
        """
        config = ContainerOptions(self.terminal)

        try:
            config.parseOptions(line)
            cmd = config.subCommand
            opts = config.subOptions if hasattr(config, 'subOptions') else {}
        except usage.UsageError as errortext:
            self.terminal.write("BUG in usage: {0}".format(errortext))
        else:
            if cmd == 'start':
                if (opts['name']):
                    data = {}
                    if opts.get('group'):
                        data['group'] = opts['group']
                    if opts.get('groupIp'):
                        data['groupIp'] = opts['groupIp']
                    if opts.get('size'):
                        data['size'] = opts['size']
                    if opts.get('bandwidth'):
                        data['bandwidth'] = opts['bandwidth']
                    if opts.get('memory'):
                        data['memory'] = opts['memory']
                    if opts.get('specialopts'):
                        data['specialFeatures'] = opts['specialopts']
                    self.callToUser('createContainer', 'robot', opts['name'],
                                    data)

            elif config['stop']:
                self.callToUser('destroyContainer', 'robot', config['stop'])
            elif config['services']:
                self.callToRosProxy('services', config['services'])
            elif config['topics']:
                self.callToRosProxy('topics', config['topics'])
            elif config['list']:
                self.callToUserAndDisplay('list_containers', 'console')
            elif config['username']:
                self.callToUserAndDisplay('list_containers_by_user', 'admin',
                                          config['username'])

    def cmd_NODE(self, line):
        """ Handler for node command.

            @param line:    line input from terminal.
            @type  line:    string
        """
        config = NodeOptions(self.terminal)

        try:
            config.parseOptions(line)
            cmd = config.subCommand
            opts = config.subOptions if hasattr(config, 'subOptions') else {}
        except usage.UsageError as errortext:
            self.terminal.write("BUG in usage: {0}".format(errortext))
        else:
            if cmd == 'start':
                if (opts['args'] and opts['ctag'] and opts['ntag']
                    and opts['pkg'] and opts['exe']):
                    self.callToUser('addNode', 'robot', opts['ctag'],
                                    opts['ntag'], opts['pkg'], opts['exe'],
                                    opts['args'])
                elif (opts['ctag'] and opts['ntag']  and opts['pkg']
                      and opts['exe']):
                    self.callToUser('addNode', 'robot', opts['ctag'],
                                    opts['ntag'], opts['pkg'], opts['exe'])
            elif cmd == 'stop':
                if opts['ctag'] and opts['ntag']:
                    self.callToUser('removeNode', 'robot', opts['ctag'],
                                    opts['ntag'])

    def cmd_PARAMETER(self, line):
        """ Handler for parameter command.

            @param line:    line input from terminal.
            @type  line:    string
        """
        config = ParameterOptions(self.terminal)

        try:
            config.parseOptions(line)
            cmd = config.subCommand
            opts = config.subOptions if hasattr(config, 'subOptions') else {}
        except usage.UsageError as errortext:
            self.terminal.write("BUG in usage: {0}".format(errortext))
        else:
            if cmd == 'add':
                if opts['ctag'] and opts['name'] and opts['value']:
                    self.callToUser('addParameter', 'robot', opts['ctag'],
                                    opts['name'], opts['value'])
            elif cmd == 'remove':
                if opts['ctag'] and opts['name']:
                    self.callToUser('removeParameter', 'robot', opts['ctag'],
                                    opts['name'])

    def cmd_INTERFACE(self, line):
        """ Handler for interface command.

            @param line:    line input from terminal.
            @type  line:    string
        """
        config = InterfaceOptions(self.terminal)

        try:
            config.parseOptions(line)
            cmd = config.subCommand
            opts = config.subOptions if hasattr(config, 'subOptions') else {}
        except usage.UsageError as errortext:
            self.terminal.write("BUG in usage: {0}".format(errortext))
        else:
            if cmd == 'add':
                if (opts['addr'] and opts['etag'] and opts['itag']
                    and opts['itype'] and opts['icls']):
                    self.callToUser('addInterface', 'robot', opts['etag'],
                                    opts['itag'], opts['itype'], opts['icls'],
                                    opts['addr'])
                elif (opts['etag'] and opts['itag'] and opts['itype'] and
                      opts['icls']):
                    self.callToUser('addInterface', 'robot', opts['etag'],
                                    opts['itag'], opts['itype'], opts['icls'])
            elif cmd == 'remove':
                if opts['etag'] and opts['itag']:
                    self.callToUser('removeInterface', 'robot', opts['etag'],
                                    opts['itag'])

    def cmd_CONNECTION(self, line):
        """ Handler for connection command.

            @param line:    line input from terminal.
            @type  line:    string
        """
        config = ConnectionOptions(self.terminal)

        try:
            config.parseOptions(line)
            cmd = config.subCommand
            opts = config.subOptions if hasattr(config, 'subOptions') else {}
        except usage.UsageError as errortext:
            self.terminal.write("BUG in usage: {0}".format(errortext))
        else:
            if cmd == 'add':
                if opts['tag1'] and opts['tag2']:
                    self.callToUser('addConnection', 'robot', opts['tag1'],
                                    opts['tag2'])
            elif cmd == 'remove':
                if opts['tag1'] and opts['tag2']:
                    self.callToUser('removeConnection', 'robot', opts['tag1'],
                                    opts['tag2'])

    def cmd_ROBOT(self, line):
        """ Handler for robot command.

            @param line:    line input from terminal.
            @type  line:    string
        """
        config = RobotOptions(self.terminal)
        try:
            config.parseOptions(line)
        except usage.UsageError as errortext:
            self.terminal.write("BUG in usage: {0}".format(errortext))
        else:
            if config['list']:
                self.callToUserAndDisplay('list_robots', 'console')
            elif config['username']:
                self.callToUserAndDisplay('list_robots_by_user', 'admin',
                                          config['username'])

    def cmd_MACHINE(self, line):
        """ Handler for machine command.

            @param line:    line input from terminal.
            @type  line:    string
        """
        config = MachineOptions(self.terminal)
        try:
            config.parseOptions(line)
        except usage.UsageError as errortext:
            self.terminal.write("BUG in usage: {0}".format(errortext))
        else:
            if config['list']:
                self.callToUserAndDisplay('list_machines', 'admin')
            elif config['stats']:
                self.callToUserAndDisplay('stats_machine', 'admin',
                                          config['stats'])
            elif config['containers']:
                self.callToUserAndDisplay('machine_containers', 'admin',
                                          config['containers'])

    def cmd_HELP(self, line):
        """ Handler for help command.

            @param line:    line input from terminal.
            @type  line:    string
        """
        configs = [UserOptions(self.terminal), ContainerOptions(self.terminal),
                   NodeOptions(self.terminal), ParameterOptions(self.terminal),
                   InterfaceOptions(self.terminal),
                   ConnectionOptions(self.terminal),
                   RobotOptions(self.terminal), MachineOptions(self.terminal)]

        for config in configs:
            self.terminal.nextLine()
            config.opt_help()


def runWithProtocol(klass, masterIP, port):
    """ Function overridden from twisted.conch.stdio to allow Ctrl+C interrupt

        @param klass:     A callable which will be invoked with
                          *a, **kw and should return an ITerminalProtocol
                          implementor. This will be invoked when a connection
                          to this ServerProtocol is established.

        @param masterIP:  IP of the master server.
        @type  masterIP:  string
    """
    fd = sys.stdin.fileno()
    oldSettings = termios.tcgetattr(fd)
    tty.setcbreak(fd)

    try:
        p = ServerProtocol(klass, masterIP, port)
        stdio.StandardIO(p)
        reactor.run()  #@UndefinedVariable
    finally:
        termios.tcsetattr(fd, termios.TCSANOW, oldSettings)
        os.write(fd, "\r\x1bc\r")


def main(ip, port):
    startLogging(sys.stdout)
    runWithProtocol(ConsoleClient, ip, port)
