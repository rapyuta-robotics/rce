#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     testRunner.py
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
#     \author/s: Dominique Hunziker
#
#

# Python specific imports
import json

# ROS specific imports
import roslib; roslib.load_manifest('Test')
import rospy

# rce test specific imports
from center import SIZES, TestCenter


PASSES = 3


def main(passes, testType, filename):
    rospy.init_node('testRunner')

    times = []
    testCenter = TestCenter()

    for _ in xrange(passes):
        times.append(testCenter.runTest(testType))

    with open(filename, 'w') as f:
        f.write(json.dumps(SIZES))
        f.write('\n')

        f.write(json.dumps(times))
        f.write('\n')


def _get_argparse():
    from argparse import ArgumentParser

    parser = ArgumentParser(prog='testRunner',
                            description='Run communication measurement for RCE '
                                        'between two ROS nodes using '
                                        'a string message.')

    parser.add_argument('--passes', type=int, help='Number of passes to do.',
                        default=PASSES)
    parser.add_argument('test', type=str, help='Test which should be run.',
                        choices=['service', 'topic'])
    parser.add_argument('output', type=str,
                        help='Filename to which the data should be written.')

    return parser


if __name__ == "__main__":
    args = _get_argparse().parse_args()

    main(args.passes, args.test, args.output)
