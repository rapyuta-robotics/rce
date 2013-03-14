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
from urllib import urlencode
from urllib2 import urlopen, HTTPError
import sys, os, termios, tty, json
import getopt

#twisted specific imports
from twisted.python.log import err, startLogging
from twisted.cred.credentials import UsernamePassword
from twisted.internet import reactor
from twisted.spread.pb import PBClientFactory
from twisted.internet import stdio
from twisted.conch.recvline import HistoricRecvLine
from twisted.conch.insults.insults import ServerProtocol
from twisted.python import usage


class CustomOptions(usage.Options):
    """ Custom Class to override functionality of usage.Options
    """
    
    def __init__(self, terminal):
        """ Initialize the CutsomOptions Class
        
            @param terminal:    Reference to the running terminal
            @type  terminal:    ConsoleClient.terminal
        """
        super(CustomOptions, self).__init__()
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
        """
        Function overridden to prevent program exiting after showing help.
        """
        self.terminal.write(self.__str__())

    def opt_version(self):
        self.terminal.write('vestigial option')


#Various Option Classes follow
class UserAddOptions(CustomOptions):
    """
        Parameters for user add.
    """
    optParameters = [
        ["username", "u", None, "Username"],
        ["password", "p", None, "Password"]
    ]


class UserRemoveOptions(CustomOptions):
    """
        Parameters for user remove.
    """
    optParameters = [
        ["username", "u", None, "Username"]
    ]


class UserOptions(CustomOptions):
    """
        Options for user command.
    """
    subCommands = [['add', None, UserAddOptions, "Add User"],
                   ['remove', None, UserRemoveOptions, "Remove User"],
                   ['update', None, UserAddOptions, "Update User"]
    ]
    optFlags = [
        ["list", "l", "List all Users"]
    ]

    
class ContainerOptions(CustomOptions):
    """
        Options for container command.
    """
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

    
class NodeStartOptions(CustomOptions):
    """
        Parameters for node start.
    """
    optParameters = [
        ["ctag", "c", None, "Container tag"],
        ["ntag", "n", None, "Node tag"],
        ["pkg", "p", None, "Package"],
        ["exe", "e", None, "Executable"],
        ["args", "a", None, "Arguments"]
    ]


class NodeStopOptions(CustomOptions):
    """
        Parameters for node stop.
    """
    optParameters = [
        ["ctag", "c", None, "Container tag"],
        ["ntag", "n", None, "Node tag"]
    ]


class NodeOptions(CustomOptions):
    """
        Options for node command.
    """
    subCommands = [['start', None, NodeStartOptions, "Start Node"],
                   ['stop', None, NodeStopOptions, "Stop Node"]]


class ParameterAddOptions(CustomOptions):
    """
        Parameters for adding ROS parameters.
    """
    optParameters = [
        ["ctag", "c", None, "Container tag"],
        ["name", "n", None, "Name of parameter"],
        ["value", "v", None, "Value of parameter"]
    ]

    
class ParameterRemoveOptions(CustomOptions):
    """
        Parameters for removing ROS parameters.
    """
    optParameters = [
        ["ctag", "c", None, "Container tag"],
        ["name", "n", None, "Name of parameter"]
    ]


class ParameterOptions(CustomOptions):
    """
        Options for parameter command.
    """
    subCommands = [['add', None, ParameterAddOptions, 'Add parameter'],
                   ['remove', None, ParameterRemoveOptions, 'Remove parameter']]

                   
class InterfaceAddOptions(CustomOptions):
    """
        Parameters for adding interfaces.
    """
    optParameters = [
        ["etag", "e", None, "Endpoint tag"],
        ["itag", "i", None, "Interface tag"],
        ["itype", "t", None, "Interface type"],
        ["icls", "c", None, "Interface Class"],
        ["addr", "a", None, "Address"]
    ]


class InterfaceRemoveOptions(CustomOptions):
    """
        Parameters for removing interfaces.
    """
    optParameters = [
        ["etag", "e", None, "Endpoint tag"],
        ["itag", "i", None, "Interface tag"]
    ]


class InterfaceOptions(CustomOptions):
    """
        Options for interface command.
    """
    subCommands = [['add', None, InterfaceAddOptions, 'Add interface'],
                   ['remove', None, InterfaceRemoveOptions, 'Remove interface']
    ]


class ConnectionSubOptions(CustomOptions):
    """
        Sub options for connection subcommands.
    """
    optParameters = [
        ["tag1", "1", None, "First Interface"],
        ["tag2", "2", None, "Second Interface"]
    ]


class ConnectionOptions(CustomOptions):
    """
        Options for connection command.
    """
    subCommands = [['add', None, ConnectionSubOptions, 'Connect Interfaces'],
                   ['remove', None, ConnectionSubOptions, 
                   'Disconnect Interfaces']
    ]


class RobotOptions(CustomOptions):
    """
        Options for robot command.
    """
    optParameters = [
        ["username", "u", None, "List Robots by Username"]
    ]
    optFlags = [
        ["list", "l", "List all Robots"]
    ]


class MachineOptions(CustomOptions):
    """
        Options for machine command.
    """
    optParameters = [
        ["stats", "s", None, "Statistics of Machine by IP"],
        ["containers", "c", None, "List Containers by Machine's IP"]
    ]
    optFlags = [
        ["list", "l", "List all Machines"]
    ]
    

class ConsoleClient(HistoricRecvLine):
    """ The class creates the terminal and manages connections with Master
        and ROSAPI servers on specific containers
    """
    def __init__(self, masterIP, consolePort):
        """ Initialize the ConsoleClient. 

            @param masterIP:    The IP of the master server
            @type  masterIP:    string
        """
        self._user = None
        self._masterIP = masterIP
        self._console_port = consolePort
        self._mode = "Username"
        self._username = None
        self._password = None
        self._factory = None
        self._connectedrosapinodes = {}

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

        reactor.connectTCP(self._masterIP, self._console_port, self._factory)
        self.terminal.write("Username: ")

    def parseInputLine(self, line):
        """
            A function to route various commands entered via Console.
            
            @param line:    The text entered on the Console
            @type line:    string
        """
        if line is not None and line is not '':
            func = getattr(self, 'cmd_' + line.split()[0].upper(), None)
            if func is not None:
                func(line.split()[1:])
            else:
                self.terminal.write('No such command')
        self.showPrompt()

    def callToRosProxy(self, command, parameter):
        """ 
            Function to handle call to ROSAPI Proxy Server.
            
            @param command:      The command to execute in ROS environment.
            @type command:       string
            
            @param parameter:    A parameter for the command.
            @type parameter:     string
        """
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
            d.addErrback(lambda err: self.terminal.write("Problem "
                                "in connection with master: "
                                "{0}".format(str(err))))
        
    def callToUser(self, command, *args):
        """
            A wrapper function for call to remote user.
            
            @param command:    The command to be executed
            @type command:     string
        """
        self._user.callRemote(command, *args)
        
    def callToUserAndDisplay(self, command, *args):
        """
            A wrapper function around call to user and displaying the result
            @param command:    The command to be executed
            @type command:     string
        """
        d = self._user.callRemote(command, *args)
        d.addCallback(lambda result: self.terminal.write(str(result)))

    #Various commands follow
    def cmd_EXIT(self, line):
        """
            Handler for exit command.
            @param line:    line input from terminal.
            @type line:     string
        """
        reactor.stop()

    def cmd_USER(self, line):
        """
            Handler for user command.
            @param line:    line input from terminal.
            @type line:     string
        """
        config = UserOptions(self.terminal)
        try:
            config.parseOptions(line)
            if config.subCommand == 'add':
                if (config.subOptions['username'] and 
                   config.subOptions['password']):
                    self.callToUser('add_user', config.subOptions['username'],
                                           config.subOptions['password'])

            elif config.subCommand == 'remove':
                if config.subOptions['username']:
                    self.callToUser('remove_user', 
                                    config.subOptions['username'])
            
            elif config.subCommand == 'update':
                if (config.subOptions['username'] and 
                config.subOptions['password']):
                    self.callToUser('update_user', 
                                    config.subOptions['username'],
                                    config.subOptions['password'])
                                       
            elif config['list']:
                self.callToUserAndDisplay('list_users')

        except usage.UsageError as errortext:
            self.terminal.write("BUG in usage: {0}".format(str(errortext)))

    def cmd_CONTAINER(self, line):
        """
            Handler for container command.
            @param line:    line input from terminal.
            @type line:     string
        """
        config = ContainerOptions(self.terminal)
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

        except usage.UsageError as errortext:
            self.terminal.write("BUG in usage: {0}".format(str(errortext)))
        
    def cmd_NODE(self, line):
        """
            Handler for node command.
            @param line:    line input from terminal.
            @type line:     string
        """
        config = NodeOptions(self.terminal)
        try:
            config.parseOptions(line)
            if config.subCommand == 'start':

                if (config.subOptions['args'] and config.subOptions['ctag'] and 
                   config.subOptions['ntag'] and config.subOptions['pkg'] and 
                   config.subOptions['exe']):
                    self.callToUser('start_node', config.subOptions['ctag'], 
                    config.subOptions['ntag'], config.subOptions['pkg'], 
                    config.subOptions['exe'], config.subOptions['args'])

                elif (config.subOptions['ctag'] and config.subOptions['ntag'] 
                     and
                     config.subOptions['pkg'] and config.subOptions['exe']):
                    self.callToUser('start_node', config.subOptions['ctag'], 
                    config.subOptions['ntag'], config.subOptions['pkg'], 
                    config.subOptions['exe'])

            elif config.subCommand == 'stop':
                if config.subOptions['ctag'] and config.subOptions['ntag']:
                    self.callToUser('stop_node', config.subOptions['ctag'], 
                    config.subOptions['ntag'])

        except usage.UsageError as errortext:
            self.terminal.write("BUG in usage: {0}".format(str(errortext)))

    def cmd_PARAMETER(self, line):
        """
            Handler for parameter command.
            @param line:    line input from terminal.
            @type line:     string
        """
        config = ParameterOptions(self.terminal)
        try:
            config.parseOptions(line)
            if config.subCommand == 'add':
                if (config.subOptions['ctag'] and config.subOptions['name'] and
                config.subOptions['value']):
                    self.callToUser('add_parameter', 
                    config.subOptions['ctag'], config.subOptions['name'], 
                    config.subOptions['value'])

            elif config.subCommand == 'remove':
                if config.subOptions['ctag'] and config.subOptions['name']:
                    self.callToUser('remove_parameter', 
                    config.subOptions['ctag'], config.subOptions['name'])

        except usage.UsageError as errortext:
            self.terminal.write("BUG in usage: {0}".format(str(errortext)))
    
    def cmd_INTERFACE(self, line):
        """
            Handler for interface command.
            @param line:    line input from terminal.
            @type line:     string
        """
        config = InterfaceOptions(self.terminal)
        try:
            config.parseOptions(line)
            if config.subCommand == 'add':
            
                if (config.subOptions['addr'] and config.subOptions['etag'] and
                   config.subOptions['itag'] and config.subOptions['itype']
                   and config.subOptions['icls']):
                    self.callToUser('add_interface', 
                    config.subOptions['etag'], config.subOptions['itag'], 
                    config.subOptions['itype'], config.subOptions['icls'], 
                    config.subOptions['addr'])

                elif (config.subOptions['etag'] and config.subOptions['itag'] 
                     and
                     config.subOptions['itype'] and config.subOptions['icls']):
                    self.callToUser('add_interface', 
                    config.subOptions['etag'], config.subOptions['itag'], 
                    config.subOptions['itype'], config.subOptions['icls'])
                    
            elif config.subCommand == 'remove':
                if config.subOptions['etag'] and config.subOptions['itag']:
                    self.callToUser('remove_interface', 
                    config.subOptions['etag'], config.subOptions['itag'])
                
        except usage.UsageError as errortext:
            self.terminal.write("BUG in usage: {0}".format(str(errortext)))
    
    def cmd_CONNECTION(self, line):
        """
            Handler for connection command.
            @param line:    line input from terminal.
            @type line:     string
        """
        config = ConnectionOptions(self.terminal)
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

        except usage.UsageError as errortext:
            self.terminal.write("BUG in usage: {0}".format(str(errortext)))
                    
    def cmd_ROBOT(self, line):
        """
            Handler for robot command.
            @param line:    line input from terminal.
            @type line:     string
        """
        config = RobotOptions(self.terminal)
        try:
            config.parseOptions(line)

            if config['list']:
                self.callToUserAndDisplay('list_robots')
                
            elif config['username'] is not None:
                self.callToUserAndDisplay('list_robots_by_user', 
                                           config['username'])

        except usage.UsageError as errortext:
            self.terminal.write("BUG in usage: {0}".format(str(errortext)))

    def cmd_MACHINE(self, line):
        """
            Handler for machine command.
            @param line:    line input from terminal.
            @type line:     string
        """
        config = MachineOptions(self.terminal)
        try:
            config.parseOptions(line)
            if config['list']:
                self.callToUserAndDisplay('list_machines')
            
            elif config['stats'] is not None:
                self.callToUserAndDisplay('stats_machine', config['stats'])
            
            elif config['containers'] is not None:
                self.callToUserAndDisplay('machine_containers', 
                                           config['containers'])

        except usage.UsageError as errortext:
            self.terminal.write("BUG in usage: {0}".format(str(errortext)))

    def cmd_HELP(self, line):
        """
            Handler for help command.
            @param line:    line input from terminal.
            @type line:     string
        """
        configs = [UserOptions(self.terminal), ContainerOptions(self.terminal),
                   NodeOptions(self.terminal), ParameterOptions(self.terminal),
                   InterfaceOptions(self.terminal), 
                   ConnectionOptions(self.terminal),
                   RobotOptions(self.terminal), MachineOptions(self.terminal)]
        for config in configs:
            self.terminal.nextLine()
            config.opt_help()

    def lineReceived(self, line):
        """
            Manage state/mode after connection. Code uses states to take 
            credential input and then starts terminal input.
             
            @param line:    line typed on terminal
            @type line:     string
        """
        
        def _cbError(why, msg):
            err(why, msg)
            reactor.stop()

        def _cbConnected(perspective):
            self._user = perspective
            self.terminal.write('Connection to Master Established.')
            self.showPrompt()
            
        if self._mode == 'Username':
            self._mode = 'Password'
            self._username = line
            self.terminal.write('Password: ')

        elif self._mode == 'Password':
            self._mode = 'Terminal'
            self._password = line
            usernameLogin = self._factory.login(UsernamePassword(self._username,
                                                self._password))
            usernameLogin.addCallback(_cbConnected)
            usernameLogin.addErrback(_cbError, "Username/password login failed")

        else:
            self.parseInputLine(line)

def runWithProtocol(klass, masterIP):
    """
        Function overridden from twisted.conch.stdio to allow Ctrl+C interrupt
        @param klass:     A callable which will be invoked with
                          *a, **kw and should return an ITerminalProtocol 
                          implementor. This will be invoked when a connection
                          to this ServerProtocol is established.

        @param masterIP:  IP of the master server.  
        @type masterIP:   string
    """
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
    
    parser.add_argument('--port', help='Console port to connect to.',
                        type=int, default= 8081)

    return parser

def main():
    startLogging(sys.stdout)
    args = _get_argparse().parse_args()
    runWithProtocol(ConsoleClient, args.ipMaster, args.port)
  
if __name__ == '__main__':
    main()
