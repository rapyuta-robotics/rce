'''
Created on Jan 1, 2013

@author: dominique
'''


from twisted.internet import reactor
from twisted.cred.credentials import UsernamePassword

from rce.robot import main

cred = UsernamePassword('robot', 'robot')

main(reactor, cred, 'localhost', 8080, 9010)
