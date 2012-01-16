#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       MaximumOccurrence.py
#       
#       Copyright 2011 dominique hunziker <dominique.hunziker@gmail.com>
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

from __future__ import print_function

# Import general
from SemanticBase import SemanticBase

# Import MaximumOccurrence
from string import lower, digits, punctuation

# nltk
try:
    from nltk.corpus import wordnet
except ImportError:
    print('Requires nltk.\n')
    exit(0)

class MaximumOccurrence(SemanticBase):
    """ Very simple search technique for the suggestion of the synset
        which describes the object. The returned synset is the word
        which is the most frequently used in the rawData.
    """
    _FILTER = [ 'an', 'all', 'and', 'any', 'are', 'as', 'at',
                'by',
                'color',
                'down',
                'for', 'from',
                'in', 'into', 'is', 'it', 'if',
                'less',
                'max', 'min', 'more',
                'new',
                'of', 'on', 'our',
                'some',
                'the', 'these', 'they', 'this', 'to',
                'up',
                'we', 'with',
                'you', 'your']
    
    _ARTIFACT=wordnet.synset('artifact.n.01')
    
    def process(self):
        """ Overwrites the necessary method from the base class.
            This function is used to run very simple maximum search
            on the available data for the suggestion.
        """
        countedData=[]
        
        for dataDict in self.data:
            data=self._rawToWord(dataDict)
            
            if data is not None:
                countedData.append(data)
        
        if len(countedData) == 0:
            self.result=['No data in english available!']
            return
        
        wordNetResult=[]
        
        for data in countedData:
            wordNetResult.append(self._searchWordNet(data))
        
        self.result=self._evaluateWordNetResult(wordNetResult)
    
    def _rawToWord(self, dataDict):
        """ Counts the rawData. As an input is a dict (from a WebDB)
            necessary. It returns the counted data in the form of a
            dictionary containing lists of counted words.
        """
        countedData={}
        
        if dataDict['language'] != 'en':
            return None
        
        for key in 'product', 'type', 'description', 'additional_info':
            if key not in dataDict:
                continue
            
            # get text
            text=dataDict[key]
            
            if text is None or len(text) < 2:
                continue
            
            # prepare data
            words=text.split()
            words=map(lower, words)
            
            # count
            countedData[key]=self._countWords(words)
        
        return countedData
    
    def _searchWordNet(self, countedData):
        """ Searches the countedData (a dictionary ocontaining lists of
            counted words as values) from the highest counted words to
            the lowest ones for each key until a group is found which
            contains at least one Artifact. It returns the dict with
            transformed values containing only a list the Artifacts from
            the prviously found group.
        """
        result={}
        
        for key in countedData:
            result[key]=None
            
            for (count, wordList) in countedData[key]:
                for word in wordList:
                    if self._checkWord(word):
                        if result[key] is None:
                            result[key]=(count,[word])
                        else:
                            result[key][1].append(word)
                
                if result[key] is not None:
                    break
        
        return result
    
    def _evaluateWordNetResult(self, wordNetResult):
        """ Summarizes the list of results of the return values of
            the method _countWords to a single list of the single
            most common words.
        """
        wordList=[]
        
        for result in wordNetResult:
            for key in result:
                if result[key] is not None:
                    wordList+=result[key][1]
        
        return self._countWords(wordList)[0][1]
    
    def _countWords(self, wordList):
        """ Count the given list of words. The method returns a list
            containing tuples of the form (count, [list of words with count]).
        """
        # count and order words
        wordDict={}
        for word in wordList:
            if len(word) > 1 and word not in MaximumOccurrence._FILTER and not self._containsNumbers(word):
                for symbol in punctuation:
                    word=word.replace(symbol, '')
                
                if word[-1:] == 's':
                    word=word[:-1]
                
                if word in wordDict:
                    wordDict[word]+=1
                else:
                    wordDict[word]=1
        
        sortedData=list((word, wordDict[word]) for word in sorted(wordDict, key=wordDict.get, reverse=True))
        
        # reorder sorted words
        countedWords=[]
        oldCount=-1
        sameCount=[]
        
        for (word, count) in sortedData:
            if oldCount != count:
                if oldCount != -1:
                    countedWords.append((oldCount, sameCount))
                
                sameCount=[]
                oldCount=count
            
            sameCount.append(word)
        
        countedWords.append((oldCount, sameCount))
        
        return countedWords
    
    def _containsNumbers(self, word):
        """ Check if a word contains any numbers and returns a bool.
        """
        for letter in word:
            if letter in digits:
                return True
        
        return False
    
    def _checkWord(self, word):
        """ Checks if a word is an Artifact and returns a bool.
        """
        synsets=wordnet.synsets(word)
        
        for synset in synsets:
            if self._isArtifact(synset):
                return True
        
        return False
    
    def _isArtifact(self, synset):
        """ Checks if the given synset is an Arifact and returns a bool.
        """
        seen = set()
        todo = [synset]
        
        while todo:
            next_synset = todo.pop()
            
            if next_synset not in seen:
                seen.add(next_synset)
                next_hypernyms = next_synset.hypernyms() + next_synset.instance_hypernyms()
                
                if MaximumOccurrence._ARTIFACT in next_hypernyms:
                    return True
                else:
                    todo.extend(next_hypernyms)
        
        return False
