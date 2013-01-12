'''
Created on Jan 1, 2013

@author: dominique
'''


from twisted.internet import reactor
from twisted.cred.checkers import InMemoryUsernamePasswordDatabaseDontUse

from rce.master.core import main

cred = InMemoryUsernamePasswordDatabaseDontUse(robot='robot',
                                               container='container',
                                               environment='environment')

main(reactor, cred, 8080, 9000)
