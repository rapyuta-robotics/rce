'''
Created on Jan 1, 2013

@author: dominique
'''

# Before we start to import everything check if the script can be executed
import os

if os.getuid() != 0:
    print('newContainer has to be run as super user.')
    exit(1)

from twisted.internet import reactor
from twisted.cred.credentials import UsernamePassword

import settings
from rce.container import main

cred = UsernamePassword('container', 'container')

main(reactor, cred, 'localhost', 8080, settings.ROOTFS, settings.CONF_DIR,
     settings.DATA_DIR, settings.ROOT_SRC_DIR, settings.ROOT_PKG_DIR)
