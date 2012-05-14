#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       ContainerProcessor.py
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

# Custom imports
from ProcessorBase import ProcessorBase
from TypeBase import MessageTypes as MsgTypes

class CreateEnvProcessor(ProcessorBase):
    """ Message processor to create an environment.
    """
    IDENTIFIER = MsgTypes.ENV_CREATE
    
    def processMessage(self, msg):
        self.manager.createContainer(msg.content['commID'], msg.content['home'])

class DestroyEnvProcessor(ProcessorBase):
    """ Message processor to destroy an environment.
    """
    IDENTIFIER = MsgTypes.ENV_DESTROY
    
    def processMessage(self, msg):
        self.manager.destroyContainer(msg.content['commID'])

class StartContainerProcessor(ProcessorBase):
    """ Message processor to start a container.
    """
    IDENTIFIER = MsgTypes.CONTAINER_START
    
    def processMessage(self, msg):
        pass # TODO: Add necessary code here

class StopContainerProcessor(ProcessorBase):
    """ Message processor to stop a container.
    """
    IDENTIFIER = MsgTypes.CONTAINER_STOP
    
    def processMessage(self, msg):
        pass # TODO: Add necessary code here

class ContainerStatusProcessor(ProcessorBase):
    """ Message processor to # TODO: ???
    """
    IDENTIFIER = MsgTypes.CONTAINER_STOP
    
    def processMessage(self, msg):
        pass # TODO: Add necessary code here
