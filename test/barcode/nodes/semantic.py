#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       TermFrequency.py
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

import itertools
from string import lower

import roslib; roslib.load_manifest('barcode')
import rospy

# nltk
try:
    from nltk.corpus import wordnet
except ImportError:
    print('Requires nltk wordnet corpus.\n')
    exit(0)

from barcode.msg import DBInfo, BarcodeInfo


# Measure for generalisation while searching for common substitutes
_H = 3

# Root node which is used
_ROOT = wordnet.synset('entity.n.01')

# Simple filter of words
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

# Weights for summarizing the results of a single WebDBEntry
_WEIGHTS = { 'product' : 1,
             'type' : 0,
             'description' : 2,
             'additional_info' : 0 }

# Artifact, i.e. the type of wordnet object we are looking for
_ARTIFACT = wordnet.synset('artifact.n.01')


class LSOError(Exception):
    """ General Exception in connection with LSO.
    """
    pass


class SynsetPair(object):
    """ This class is used for the Term Frequency analysis.
    """
    def __init__(self, synsetPair):
        """ Initializes the SynsetPair object. The argument has to be a tuple of
            two instances of wordnet.Synset.
        """
        self._synsetPair = synsetPair
        s1, s2 = synsetPair
        self._lowestCommonSynsets = s1.lowest_common_hypernyms(s2)
        
        self._dist1 = max(0, s1.min_depth()-1)
        self._dist2 = max(0, s2.min_depth()-1)
        self._distLowestCommon = max(0,
                                     self._lowestCommonSynsets[0].min_depth()-1)
        
        self.dist = self._dist1 + self._dist2 - 2 * self._distLowestCommon + 1
    
    def _buildPath(self, synset, depth):
        """ Internally used for recursion.
        """
        hypernyms = synset.hypernyms() + synset.instance_hypernyms()
        
        if not depth:
            if not hypernyms:
                return [[synset]]
            else:
                return None
        
        paths = []
        
        for hypernym in hypernyms:
            path = self._buildPath(hypernym, depth-1)
            
            if path:
                paths += path
        
        for path in paths:
            path.append(synset)
        
        return paths
    
    def buildLSOSets(self):
        """ Buid shortest path from lowest common synset to root.
        """
        paths = []
        depth = self._lowestCommonSynsets[0].min_depth()
        
        for synset in self._lowestCommonSynsets:
            paths += self._buildPath(synset, depth)
        
        if len(paths) > 1:
            rospy.loginfo('More than one possible path found.')
        
        self._lsoSets = [path[-_H:] for path in paths]
        self.lsoSet = set()
        
        for lsoSet in self._lsoSets:
            self.lsoSet.update(set(lsoSet))
            lsoSet.reverse()
    
    def substitute(self, clso):
        """ Substitutes the synsetPair using the given common LSO set and
            calculates the score of the LSO.

            Score = dist(synset1, root) + dist(synset2, root)
                    - 2*dist(lso, root) + 1
        """
        self.lso = None
        
        for i, lsoSet in enumerate(zip(*self._lsoSets)):
            for lso in lsoSet:
                if lso in clso:
                    if self.lso:
                        rospy.loginfo('Multiple LSO possible.')
                    
                    self.lso = lso
                    try:
                        self.score = 1.0 / (self._dist1 + self._dist2
                                     - 2*(self._distLowestCommon-i) + 1)
                    except ZeroDivisionError:
                        self.score = 1.0
            
            if self.lso:
                break
        else:
            if len(self._lsoSets) > 1:
                rospy.loginfo('Multiple LSO possible.')
            
            self.lso = self._lsoSets[0][0]
            
            try:
                self.score = 1.0 / self.dist
            except ZeroDivisionError:
                self.score = 1.0


def process(req, pub):
    """ Overwrites the necessary method from the base class.
        This function is used to run very a term frequency analysis
        on the available data for the suggestion.
    """
    rospy.loginfo('Process...')
    resultDBEntry = []
    
    for dbEntryMsg in req.dbInfo:
        # Process the data of a single DBEntry
        iters = _msgToPairIterator(dbEntryMsg)
        
        if not iters:
            continue
        
        weightedLSO = {}
        
        for key in iters:
            # Process the word pairs by filtering for the most related senses
            synsetPairs = _getSynsetPairs(iters[key])
            
            # Process the most common senses (synset pairs) by substituting
            # the common senses with lowest super ordiantes (LSO)
            _getLSOs(synsetPairs)
            
            # Weight the lowest super ordinates
            weightedLSO[key] = _weight(synsetPairs)
            
        # Now weight and summarize the different fields of a DBEntry
        resultDBEntry.append(_reduceDBEntry(weightedLSO))
    
    if len(resultDBEntry) == 0:
        pub.publish(req.barcode, [], [])
    
    synset, score = _reduceToResponse(resultDBEntry)
    pub.publish(req.barcode, synset, score)


def _msgToPairIterator(dbEntry):
    """ Prepares the DBEntry message. It returns a dictionary with the key of
        _WEIGHTS and as values iterators returning all possible combinations
        for the text.
    """
    pairIter = {}
    
    if dbEntry.language != 'en':
        return None
    
    for key in _WEIGHTS:
        if not _WEIGHTS[key]:
            continue
        
        if not hasattr(dbEntry, key):
            continue
        
        text = getattr(dbEntry, key)
        
        if not text or len(text) < 2:
            continue
        
        words = text.split()
        words = map(lower, words)
        pairIter[key] = itertools.combinations(words, 2)
    
    return pairIter


def _getSynsetPairs(pairIter):
    """ Evaluate all word pairs given by the iterator and return a list of
        synset pairs of the most common related sense for each word pair.
    """
    synsetPairs = []
    
    for wordPair in pairIter:
        if wordPair[0] in _FILTER or wordPair[1] in _FILTER:
            continue
        
        wordPair = (wordnet.morphy(wordPair[0], wordnet.NOUN),
                    wordnet.morphy(wordPair[1], wordnet.NOUN))
        
        if not wordPair[0] or not wordPair[1]:
            continue
        
        synsetPairs += _getMostRelatedSense(wordPair)
    
    return synsetPairs


def _getMostRelatedSense(wordPair):
    """ Get the most related sense of two nouns.
        Returns the synset pair with the most related sense.
    """
    mostRelatedSenses = []
    
    for synsetPair in itertools.product(wordnet.synsets(wordPair[0],
                                                        wordnet.NOUN),
                                        wordnet.synsets(wordPair[1],
                                                        wordnet.NOUN)):
        pair = SynsetPair(synsetPair)
        
        if not mostRelatedSenses or mostRelatedSenses[0].dist > pair.dist:
            mostRelatedSenses = [pair]
        elif mostRelatedSenses[0].dist == pair.dist:
            mostRelatedSenses.append(pair)
    
    if len(mostRelatedSenses) > 1:
        rospy.loginfo('Found multiple possibilities for most related sense.')
    
    for mostRelatedSense in mostRelatedSenses:
        mostRelatedSense.buildLSOSets()
    
    return mostRelatedSenses


def _getLSOs(synsetPairs):
    """ Substitute the synset pairs with a common LSO.
    """
    clso = set()
    pendingCLSO = set()
    
    for synsetPair in synsetPairs:
        candidates = synsetPair.lsoSet - clso.intersection(synsetPair.lsoSet)
        matches = pendingCLSO.intersection(candidates)
        clso.update(matches)
        pendingCLSO -= matches
        pendingCLSO.update(candidates - matches)
    
    for synsetPair in synsetPairs:
        synsetPair.substitute(clso)


def _weight(synsetPairs):
    """ Weight the LSO.
    """
    weightedLSO = {}
    
    for synsetPair in synsetPairs:
        lso = synsetPair.lso
        
        if lso not in weightedLSO:
            weightedLSO[lso] = 0.0
        
        weightedLSO[lso] += synsetPair.score
    
    count = len(synsetPairs)
    
    for lso in weightedLSO:
        weightedLSO[lso] /= count
    
    return weightedLSO


def _reduceDBEntry(weightedLSO):
    """ Summarizes the results of the Term Frequency analysis for a single
        DBEntry.
    """
    sortedData = {}
    sumOfWeights = 0
    
    for key in _WEIGHTS:
        if not _WEIGHTS[key] or key not in weightedLSO:
            continue
        
        sumOfWeights += _WEIGHTS[key]
        
        for lso, score in weightedLSO[key].iteritems():
            if lso not in sortedData:
                sortedData[lso] = 0
            
            sortedData[lso] += score * _WEIGHTS[key]
    
    try:
        for key in sortedData:
            sortedData[key] /= sumOfWeights
    except ZeroDivisionError:
        return None
    
    return sortedData


def _reduceToResponse(dbEntries):
    """ Summarizes the already summarized results of the Term Frequency
        analysis of a single DBEntry for a Request.
    """
    result = []
    summarizedData = {}
    sumOfEntries = 0
    
    for entry in dbEntries:
        sumOfEntries += 1
        
        for key in entry:
            if key not in summarizedData:
                summarizedData[key] = 0
            
            summarizedData[key] += entry[key]
    
    for key in summarizedData:
        result.append((summarizedData[key] / sumOfEntries, key))
    
    result.sort(lambda x, y: cmp(x[0], y[0]))
    
    resultSynset = []
    resultScore = []
    
    for resultEntry in result:
        if _isArtifact(resultEntry[1]):
            resultSynset.append(resultEntry[1].name)
            resultScore.append(resultEntry[0])
            
            if len(resultSynset) >= 3:  # number of returned artifacts
                break
    
    return resultSynset, resultScore


def _isArtifact(synset):
    """ Checks if the given synset is an Arifact and returns a bool.
    """
    for hypernyms in synset._iter_hypernym_lists():
        for hypernym in hypernyms:
            if _ARTIFACT == hypernym: 
                return True
    
    return False


def semantic_server():
    rospy.init_node('semanticNode')
    
    pub = rospy.Publisher('barcodeInfo', BarcodeInfo)
    rospy.Subscriber('barcodeDBInfo', DBInfo, lambda req: process(req, pub))
    rospy.spin()


if __name__ == '__main__':
    semantic_server()
