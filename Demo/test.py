#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       test.py
#       
#       Copyright 2012 dominique hunziker <dominique.hunziker@gmail.com>
#       
#       This program is free software; you can redistribute it and/or modify
#       it under the terms of the GNU General Public License as published by
#       the Free Software Foundation; either version 2 of the License, or
#       (at your option) any later version.
#       
#       This program is distributed in the hope that it will be useful,
#       but WITHOUT ANY WARRANTY; without even the implied warranty of
#       MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#       GNU General Public License for more details.
#       
#       You should have received a copy of the GNU General Public License
#       along with this program; if not, write to the Free Software
#       Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#       MA 02110-1301, USA.
#       
#       

import sys
import os

import time
import ServiceAPI

DELTA_SUM = 10
DELTA_READ = 120

INFO = True
ADD_REMOVE_ENV = True
CHANGE_ENV = True
READ = True
SUM = True

def _validateImg(imgStr):
    import PIL.Image

    try:
        imgStr.reset()
        img = PIL.Image.open(imgStr)
        img.verify()

        imgStr.reset()
        img = PIL.Image.open(imgStr)
        img.load()
    except:
        raise ValueError('Image content could not be verified.')

    return True

def main():
    if len(sys.argv) != 2 or not os.path.isfile(sys.argv[1]):
        print 'Usage: test.py [path to image to read text from]'
        exit(0)

    print 'start...'

    try:
        if INFO:
            print 'get Available Services:'
            print ServiceAPI.getAvailableServices()
            time.sleep(0.1)
            print ' '

        if ADD_REMOVE_ENV:
            print 'add Environment:'
            envID = ServiceAPI.addEnv()
            print 'added environment {0}'.format(envID)
            time.sleep(1)
            print ' '

            print 'remove Environment:'
            ServiceAPI.removeEnv(envID)
            print 'removed environment {0}'.format(envID)
            time.sleep(1)
            print ' '

        if CHANGE_ENV:
            print 'change Environment (implicit addEnv):'
            envID = ServiceAPI.changeEnv(nodesToAdd=[('BarCodeService/WebDB.py', None)])
            print 'added node WebDB to {0}'.format(envID)
            time.sleep(1)
            print ' '

            print 'change Environment:'
            tmp = ServiceAPI.changeEnv(envID=envID, nodesToAdd=[('ReadTextService/ReadText', None), ('Test/Test.py', None)], nodesToRemove=['BarCodeService/WebDB.py'])
            if tmp != envID:
                raise ValueError('envIDs do not match.')
            print 'added nodes Test and TextReader. Removed node WebDB from {0}'.format(envID)
            time.sleep(0.1)
            print ' '

            print 'get Environment info:'
            print ServiceAPI.getEnv(envID)
            time.sleep(0.1)
            print ' '

        if READ or SUM or TOPIC:
            print 'create Environment:'
            envID = ServiceAPI.changeEnv(nodesToAdd=[('ReadTextService/ReadText', {}), ('Test/Test.py', None), ('Test/TopicTest.py', None)])
            print 'created environment with nodes Test and TextReader'
            time.sleep(0.1)
            print ' '

        if READ:
            print 'add Task:'
            taskID = ServiceAPI.addTask(envID, 'ReadTextService/ReadText', {'image' : ServiceAPI.FileHandle(sys.argv[1])})
            print 'added task {0}'.format(taskID)
            time.sleep(0.1)
            print ' '

            print 'get Task:'
            print 'status/result of task {0}:'.format(taskID)
            (status, result) = ServiceAPI.getTask(envID, taskID, DELTA_READ)
            print status
            print result
            time.sleep(0.1)
            print ' '

        if SUM:
            a = 9
            b = -5
            print 'add Task:'
            taskID = ServiceAPI.addTask(envID, 'Test/test', {'a' : a, 'b' : b})
            print 'added task {0}'.format(taskID)
            time.sleep(0.1)
            print ' '

            print 'wait for result...'
            (status, result) = ServiceAPI.getTask(envID, taskID, DELTA_SUM)
            print status
            print '({0}) + ({1}) = {2}'.format(a, b, result['sum'])
            if _validateImg(ServiceAPI.getFile(envID, taskID, result['img'])):
                print 'image is valid'

            time.sleep(0.1)
            print ' '

            print 'remove Task:'
            ServiceAPI.removeTask(envID, taskID)
            print 'removed task {0}'.format(taskID)
            time.sleep(0.1)
            print ' '

            a = 7
            b = 5
            c = -2
            s = a + b + c
            print 'add Numbers:\n{0}, {1}, {2}'.format(a, b, c)
            ServiceAPI.addTask(envID, 'Test/addInt', {'data' : a})
            time.sleep(0.1)
            ServiceAPI.addTask(envID, 'Test/addInt', {'data' : b})
            time.sleep(0.1)
            ServiceAPI.addTask(envID, 'Test/addInt', {'data' : c})
            time.sleep(0.1)
            print ' '

            print 'getSum:'
            time.sleep(0.4)
            taskID = ServiceAPI.addTask(envID, 'Test/getSum', {})
            (status, result) = ServiceAPI.getTask(envID, taskID, DELTA_SUM)

            if result['sum'] != s:
                raise ValueError('Sum is not correct. Received value: {0}'.format(result['sum']))
            else:
                print 'sum correct'
    except (ServiceAPI.RequestError, ValueError) as e:
        print 'Error: {0}'.format(e)
    else:
        print 'all ok!'

if __name__ == '__main__':
    main()
