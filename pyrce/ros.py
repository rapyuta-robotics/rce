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

global rosIF

def main(config, reactor):
    global rosIF
    rospy.init_node('RCE_ROS_Client')
    
    rosIF = []
    
    try:
        userID = config['userID']
        robotID = config['robotID']
        url = config['url']
    except KeyError as e:
        print('Configuration is missing the key {0}.'.format(e))
        return 2
    
    containers = config.get('containers', [])
    nodes = config.get('nodes', [])
    interfaces = config.get('interfaces', [])
    connections = config.get('connections', [])
    
    def setup(conn):
        print('Setup...')
        global rosIF
        print(rosIF)
        for container in containers:
            conn.createContainer(**container)
        
        for node in nodes:
            conn.addNode(**node)
        
        for interface in interfaces:
            conn.addInterface(**interface)
        
        for connection in connections:
            conn.addConnection(**connection)
        
        for ros in interfaces:
            iType = ros['iType']
            
            if iType in _MAP:
                ifc = getattr(conn, _MAP[iType])(ros['iTag'],
                                                        ros['iCls'],
                                                        ros['addr'])
                print(ifc)
                rosIF.append(ifc)
        
        print(rosIF)
        print('---')
    
    deferred = Deferred()
    deferred.addCallback(setup)
    
    conn = ROSConnection(userID, robotID, reactor)
    conn.connect(url, deferred)
    
    def terminate():
        for container in containers:
            conn.destroyContainer(**container)
        global rosIF
        del rosIF
        
        reactor.callLater(1, reactor.callFromThread, reactor.stop)
    
    rospy.on_shutdown(terminate)
    
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
    finally:
        fh.close()
    
    main(config, reactor)
