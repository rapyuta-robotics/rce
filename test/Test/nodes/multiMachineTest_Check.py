#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       multiMachineTest_Check.py
#       
#       This file is part of the RoboEarth Cloud Engine tests.
#       
#       This file was originally created for RoboEearth
#       http://www.roboearth.org/
#       
#       The research leading to these results has received funding from
#       the European Union Seventh Framework Programme FP7/2007-2013 under
#       grant agreement no248942 RoboEarth.
#       
#       Copyright 2012 RoboEarth
#       
#       Licensed under the Apache License, Version 2.0 (the "License");
#       you may not use this file except in compliance with the License.
#       You may obtain a copy of the License at
#       
#       http://www.apache.org/licenses/LICENSE-2.0
#       
#       Unless required by applicable law or agreed to in writing, software
#       distributed under the License is distributed on an "AS IS" BASIS,
#       WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#       See the License for the specific language governing permissions and
#       limitations under the License.
#       
#       \author/s: Dominique Hunziker 
#       
#       

class Request(object):
    def __init__(self, counter, data):
        self.counter = counter
        self.data = data
    
    def __str__(self):
        return 'Counter: {0}, Data: {1}'.format(self.counter, self.data)
        

def printResult(req):
    print(req)
    print('Max Int: {0}'.format(2**(4*8)-1))
    exit(0)

def decrement(req):
    old = req.data
    req.data = old - 1
    print('{0} - 1 = {1}'.format(old, req.data))
    
    if req.counter:
        req.counter -= 1
        return 'double', req
    else:
        return 'printResult', req

def double(req):
    old = req.data
    req.data = old * 2
    print('{0} * 2 = {1}'.format(old, req.data))
    
    if req.counter:
        req.counter -= 1
        return 'decrement', req
    else:
        return 'printResult', req

def main(funcName, req):
    globalDict = globals()
    
    while 1:
        funcName, req = globalDict[funcName](req)

def _get_argparse():
    from argparse import ArgumentParser

    parser = ArgumentParser(prog='multiMachineTest_Check',
                            description='Check output of multi machine check.')
    
    parser.add_argument('start', help='First node which is called.', type=str,
                        choices=('double', 'decrement'))
    parser.add_argument('counter', help='Initial value of counter.', type=int)
    parser.add_argument('data', help='Initial value of data.', type=int)

    return parser

if __name__ == '__main__':
    args = _get_argparse().parse_args()
    
    main(args.start, Request(args.counter, args.data))
