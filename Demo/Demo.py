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
import os.path

import ServiceAPI

class Env(object):
    def __init__(self):
        self.env = None

    def __del__(self):
        try:
            if self.env:
                try:
                    ServiceAPI.removeEnv(self.env)
                except ServiceAPI.RequestError:
                    pass
        except AttributeError:
            pass

    def debug(self, a, b):
        task = ServiceAPI.addTask(self.env, 'Test/test', {'a' : a, 'b' : b})

        (status, result) = ServiceAPI.getTask(self.env, task, 10)
        img = ServiceAPI.getFile(self.env, task, result['img'])

        with open('ros.png', 'w') as f:
            f.write(img.read())

        print status
        print '{0} + {1} = {2}'.format(a, b, result['sum'])
        print 'img -> ros.png'

    def readImage(self, path):
        fh = ServiceAPI.FileHandle(path)
        task = ServiceAPI.addTask(self.env, 'ReadTextService/ReadText', {'image' : fh})

        return ServiceAPI.getTask(self.env, task, 120)

    def convertToAudio(self, text):
        task = ServiceAPI.addTask(self.env, 'TTSService/TTSService', {'text' : text})
        (status, result) = ServiceAPI.getTask(self.env, task, 30)

        if status != 'completed':
            print (status, result)
            return

        wav = ServiceAPI.getFile(self.env, task, result['wavFile']['content'])

        with open(result['wavFile']['name'], 'w') as f:
            f.write(wav.read())

        print status
        print 'wav -> {0}'.format(result['wavFile']['name'])

    def scanImage(self, path):
        fh = ServiceAPI.FileHandle(path)
        task = ServiceAPI.addTask(self.env, 'BarCodeService/Scanner', {'image' : fh})

        return ServiceAPI.getTask(self.env, task, 10)

    def analyseBarcode(self, barcode):
        task = ServiceAPI.addTask(self.env, 'BarCodeService/WebDB', {'gtin' : barcode})
        (status, result) = ServiceAPI.getTask(self.env, task, 10)

        if status != 'completed':
            return (status, result)

        name = self._buildName([entry['product'] for entry in result['data'] if entry['language'] == 'en'])

        task = ServiceAPI.addTask(self.env, 'BarCodeService/Semantic', result)
        (status, result) = ServiceAPI.getTask(self.env, task, 10)

        return (status, result, name)

    def startDebug(self):
        self.env = ServiceAPI.changeEnv(self.env, nodesToAdd=[('Test/Test.py', None)])

    def startReader(self, correlation, wordList):
        config = {}

        if correlation and os.path.isfile(correlation):
            config['correlation'] = ServiceAPI.FileHandle(correlation)

        if wordList and os.path.isfile(wordList):
            config['wordList'] = ServiceAPI.FileHandle(wordList)

        self.env = ServiceAPI.changeEnv(self.env, nodesToAdd=[ ('ReadTextService/ReadText', config) ])

    def startTextToSpeech(self):
        self.env = ServiceAPI.changeEnv(self.env, nodesToAdd=[ ('TTSService/TTS.py', None) ])

    def startChain(self):
        self.env = ServiceAPI.changeEnv(self.env, nodesToAdd=[  ('BarCodeService/Semantic.py', None),
                                                                ('BarCodeService/Scanner.py', None),
                                                                ('BarCodeService/WebDB.py', None) ])

    def _buildName(self, names):
        endSign = '!end%Sign$'

        firstWord = {}
        wordDict = { endSign :-1 }

        for name in names:
            wordList = name.split()

            try:
                firstWord[wordList[0]] += 1
            except KeyError:
                firstWord[wordList[0]] = 1

            for i in xrange(len(wordList)):
                try:
                    newWord = wordList[i + 1]
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

if __name__ == '__main__':
    try:
        env = Env()
        env.startDebug()
        env.startChain()
        env.startReader(None, None)
        env.startTextToSpeech()
        r = env.readImage('test_image.jpg')
        print r
        env.convertToAudio('. '.join(r[1]['text']))
        r = env.scanImage('IMAG0061.jpg')
        print r
        print env.analyseBarcode(r[1]['barcode'][0]['gtin'])
        env.debug(-6, 10)
    except Exception as e:
        print e
