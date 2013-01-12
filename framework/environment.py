#!/usr/bin/env python
# -*- coding: utf-8 -*-


import sys

if len(sys.argv) != 3:
    from os.path import basename
    print('Usage: {0} [masterIP] [uid]'.format(basename(sys.argv[0])))
    exit(1)

from twisted.internet import reactor
from twisted.cred.credentials import UsernamePassword

from rce.environment import main

cred = UsernamePassword('environment', 'environment')

main(reactor, cred, '10.0.3.1', 8080, 10010, sys.argv[2])
