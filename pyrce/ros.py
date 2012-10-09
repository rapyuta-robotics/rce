#!/usr/bin/env python
# -*- coding: utf-8 -*-
#     
#     ros.py
#     
#     This file is part of the RoboEarth Cloud Engine pyrce client.
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
#     \author/s: Dominique Hunziker 
#     
#     

# Python specific imports
import json

# ROS specific imports
try:
    import rospy
except ImportError:
    print('Can not find ROS components.')
    exit(1)

# twisted specific imports
from twisted.internet.defer import Deferred

# Custom imports
from connection import ROSConnection


_MAP = {#'ServiceConverter' : '???',
        'ServiceProviderConverter' : 'service',
        'PublisherConverter' : 'subscriber',
        'SubscriberConverter' : 'publisher'}


class Environment(object):
    def __init__(self, reactor, conn, config):
        self._reactor = reactor
        self._conn = conn
        
        self._containers = config.get('containers', [])
        self._nodes = config.get('nodes', [])
        self._interfaces = config.get('interfaces', [])
        self._connections = config.get('connections', [])
        
        self._ifs = []
    
    def run(self, _):
        for container in self._containers:
            self._conn.createContainer(**container)
        
        for node in self._nodes:
            self._conn.addNode(**node)
        
        for interface in self._interfaces:
            self._conn.addInterface(**interface)
        
        for connection in self._connections:
            self._conn.addConnection(**connection)
        
        for ros in self._interfaces:
            iType = ros['iType']
            
            if iType in _MAP:
                self._ifs.append(getattr(self._conn, _MAP[iType])(ros['iTag'],
                                                                  ros['iCls'],
                                                                  ros['addr']))
    
    def terminate(self):
        for container in self._containers:
            self._conn.destroyContainer(**container)
        
        self._ifs = []
        
        self._reactor.callLater(1, self._reactor.callFromThread,
                                self._reactor.stop)
        

def main(config, reactor):
    rospy.init_node('RCE_ROS_Client')
    
    try:
        userID = config['userID']
        robotID = config['robotID']
        url = config['url']
    except KeyError as e:
        print('Configuration is missing the key {0}.'.format(e))
        return 1
    
    conn = ROSConnection(userID, robotID, reactor)
    env = Environment(reactor, conn, config)
    
    deferred = Deferred()
    deferred.addCallback(env.run)
    
    conn.connect(url, deferred)
    
    rospy.on_shutdown(env.terminate)
    
    reactor.run(installSignalHandlers=False)
    
    return 0


def _get_argparse():
    from argparse import ArgumentParser, FileType

    parser = ArgumentParser(prog='ROS Client',
                            description='Client for the RoboEarth Cloud Engine'
                                        ' providing an Interface for ROS based'
                                        ' communications.')

    parser.add_argument('config', help='Configuration file.',
                        type=FileType('r'))

    return parser

if __name__ == '__main__':
    from twisted.internet import reactor
    
    args = _get_argparse().parse_args()
    fh = args.config
    
    try:
        config = json.load(fh)
    except ValueError:
        print('Configuration file is not in proper JSON format.')
        exit(1)
    finally:
        fh.close()
    
    main(config, reactor)
