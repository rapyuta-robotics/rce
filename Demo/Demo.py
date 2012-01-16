#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       Demo.py
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

import operator
import time
import PIL.Image
import cStringIO

import ServiceAPI

class Env(object):
    def __init__(self):
        self.env = None
    
    def __del__(self):
        if self.env:
            ServiceAPI.removeEnv(self.env)
    
    def scanImage(self, path):
        if not self.env:
            self._setup()
        
        img = PIL.Image.open(path)
        out = cStringIO.StringIO()
        img.save(out, 'PNG')
        
        fh = ServiceAPI.FileHandle('barcode.png', out)
        task = ServiceAPI.addTask(self.env, 'ROS_Services/Scanner', {'image' : fh})
        
        return self._waitForResult(task, 10)
    
    def analyseBarcode(self, barcode):
        if not self.env:
            self._setup()
        
        task = ServiceAPI.addTask(self.env, 'ROS_Services/WebDB', {'gtin' : barcode})
        (status, result) = self._waitForResult(task, 10)
        
        if status != 'completed':
            return (status, result)
        
        name = self._buildName([entry['product'] for entry in result['data'] if entry['language'] == 'en'])
        
        task = ServiceAPI.addTask(self.env, 'ROS_Services/Semantic', result)
        (status, result) = self._waitForResult(task, 10)
        
        return (status, result, name)
    
    def _setup(self):
        self.env = ServiceAPI.changeEnv(nodesToAdd=[('ROS_Services/Semantic.py', None),
                                                    ('ROS_Services/Scanner.py', None),
                                                    ('ROS_Services/WebDB.py', None)])
    
    def _waitForResult(self, task, delta):
        limit = time.time() + delta
        
        while time.time() < limit:
            time.sleep(0.1)
            (status, result) = ServiceAPI.getTask(self.env, task)
            
            if status != 'running':
                break
        
        return (status, result)
    
    def _buildName(self, names):
        endSign = '!end%Sign$'
        
        firstWord = {}
        wordDict = { endSign : -1 }
        
        for name in names:
            wordList = name.split()
            
            try:
                firstWord[wordList[0]] += 1
            except KeyError:
                firstWord[wordList[0]] = 1
            
            for i in xrange(len(wordList)):
                try:
                    newWord = wordList[i+1]
                except IndexError:
                    newWord = endSign
                
                try:
                    words = wordDict[wordList[i]]
                except KeyError:
                    words = {}
                
                try:
                    words[newWord] += 1
                except KeyError:
                    words[newWord] = 1
                
                wordDict[wordList[i]] = words
        
        name = []
        
        firstWord = sorted(firstWord.iteritems(), key=operator.itemgetter(1), reverse=True)
        name.append(firstWord[0][0])
        
        while True:
            words = sorted(wordDict[name[-1]].iteritems(), key=operator.itemgetter(1), reverse=True)
            word = words[0][0]
            
            if word == endSign or word in name:
                break
            
            name.append(word)
        
        return ' '.join(name)
