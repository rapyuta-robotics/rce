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

from __future__ import print_function

# Import general
from SemanticBase import SemanticBase

# Import TermFrequency
from string import lower

# nltk
try:
    from nltk.corpus import wordnet
except ImportError:
    print('Requires nltk.\n')
    exit(0)

class TermFrequency(SemanticBase):
    """ Search technique based on frequency of a term for the suggestion
        of the synset which describes the object. The returned synset is
        the term which is the most frequently used in the rawData and
        which is a child of the synset _ROOT.
    """

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
    _WEIGHTS = {    'product' : 1,
                    'type' : 0,
                    'description' : 2,
                    'additional_info' : 0 }

    # Artifact, i.e. the type of wordnet object we are looking for
    _ARTIFACT = wordnet.synset('artifact.n.01')

    def process(self):
        """ Overwrites the necessary method from the base class.
            This function is used to run very a term frequency analysis
            on the available data for the suggestion.
        """
        resultDBEntry = []

        for dataDict in self.data:
            # Initialize the data of a single DBEntry
            # Build a list of _WordPair instances with all possible word pairs
            data = self._rawToPairs(dataDict)

            # Process the data of a single DBEntry
            if data is not None:
                # Process the word pairs by filtering for the most related senses
                data = self._evaluateWordPairs(data)

                # Process the most common senses (synset pairs) by substituting
                # the the common senses with lowest super ordiantes (LSO)
                data = self._evaluateSynsetPairs(data)

                # Summarize the lists of lowest super ordinates
                data = self._reduceLSO(data)

                # Now weight and summarize the different field of a DBEntry
                data = self._summarizeResultsForDBEntry(data)

                if data is not None:
                    resultDBEntry.append(data)

        if len(resultDBEntry) == 0:
            self.result = ['No data in english available!']
            return

        # Postprocessing
        self.result = self._summarizeResultsForRequest(resultDBEntry)

    def _rawToPairs(self, dataDict):
        """ Prepares the rawData. As an input is a dict (from a WebDB)
            necessary. It returns the prepared data in the form of a
            dictionary containing lists of _WordPair instances.
        """
        pairedData = {}

        if dataDict['language'] != 'en':
            return None

        for key in 'product', 'type', 'description', 'additional_info':
            if key not in dataDict:
                continue

            # get text
            text = dataDict[key]

            if text is None or len(text) < 2:
                continue

            # prepare data
            words = text.split()
            words = map(lower, words)

            # generate pairings
            pairedData[key] = self._generateWordPairs(words)

        return pairedData

    def _generateWordPairs(self, words):
        """ Generates a list of all possible word pairs.
        """
        pairs = []

        for i in xrange(len(words)):
            for j in xrange(i + 1, len(words)):
                wordPair = _WordPair.create(words[i], words[j])

                if wordPair is not None:
                    pairs.append(wordPair)

        return pairs

    def _evaluateWordPairs(self, data):
        """ Runs the necessary functions on the WordPairs. The argument
            should be a dictionary containing lists of _WordPair
            instances. Returns the the dictionary containing list of
            _SynsetPair instances which represent the most related senses
            to the _WordPair instances.
        """
        newData = {}

        for key in data:
            newData[key] = []

            for wordPair in data[key]:
                newData[key].append(wordPair.getMostRelatedSenses())

        return newData

    def _evaluateSynsetPairs(self, data):
        """ Runs the necessary functions on the SynsetPairs. The argument
            should be a dictionary containing lists of _SynsetPair
            instances. Returns the modified data.
        """
        for key in data:
            # Build common LSO set
            clso = set()

            for synsetPair in data[key]:
                for lso in synsetPair.lso:
                    if lso not in clso:
                        if self._isLSOCommon(lso, data[key], synsetPair):
                            clso.add(lso)

            # Now find the lowest super ordiantes (LSO)
            for synsetPair in data[key][:]:
                if len(synsetPair.lso) == 0:
                    data[key].remove(synsetPair)
                    continue

                subset = synsetPair.lso[-TermFrequency._H:]
                subset.reverse()

                for lso in subset:
                    if lso in clso:
                        synsetPair.substituteLSOAndCalcScore(lso)
                        break
                else:
                    synsetPair.substituteLSOAndCalcScore(synsetPair.lso[0])

        return data

    def _isLSOCommon(self, searchLSO, synsetPairList, currentSynsetPair):
        """ Checks if the searchLSO of the currentSynsetPair can be found
            in another SynsetPair in the synsetPairList. Returns True if a
            match is found and False if no match is found.
        """
        for synsetPair in synsetPairList:
            if synsetPair == currentSynsetPair:
                continue

            for lso in synsetPair.lso:
                if lso == searchLSO:
                    return True

        return False

    def _reduceLSO(self, data):
        """ Reduces the processed synsetPairs lists.  The argument
            should be a dictionary containing lists of _SynsetPair
            instances. Returned is the dictionary containing dictionaries
            with wordnet.synset as keys and their correpsoning frequency
            as value.
        """
        reducedLSO = {}

        for key in data:
            reducedLSO[key] = {}
            counter = 0

            for synsetPair in data[key]:
                counter += 1

                try:
                    reducedLSO[key][synsetPair.substitute] += synsetPair.score
                except KeyError:
                    reducedLSO[key][synsetPair.substitute] = synsetPair.score

            for keyLSO in reducedLSO[key]:
                reducedLSO[key][keyLSO] /= counter

        return reducedLSO

    def _summarizeResultsForDBEntry(self, data):
        """ Summarizes the results of the Term Frequency analysis for a
            single DBEntry.
        """
        sortedData = {}
        sumOfWeights = 0

        for key in data:
            sumOfWeights += TermFrequency._WEIGHTS[key]

            for keyData in data[key]:
                try:
                    sortedData[keyData] += data[key][keyData] * TermFrequency._WEIGHTS[key]
                except KeyError:
                    sortedData[keyData] = data[key][keyData] * TermFrequency._WEIGHTS[key]

        try:
            for key in sortedData:
                sortedData[key] /= sumOfWeights
        except ZeroDivisionError:
            return None

        return sortedData

    def _summarizeResultsForRequest(self, data):
        """ Summarizes the already summarized results of the Term Frequency
            analysis of a single DBEntry for a Request.
        """
        result = []
        summarizedData = {}
        sumOfEntries = 0

        for entry in data:
            sumOfEntries += 1

            for key in entry:
                try:
                    summarizedData[key] += entry[key]
                except KeyError:
                    summarizedData[key] = entry[key]

        for key in summarizedData:
            result.append((summarizedData[key] / sumOfEntries, key))

        result.sort(lambda x, y: cmp(x[0], y[0]))

        returnResult = []

        for resultEntry in result:
            if self._isArtifact(resultEntry[1]):
                returnResult.append(resultEntry[1].name)

                if len(returnResult) >= 3:  # number of returned artifacts
                    break

        return returnResult

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

                if TermFrequency._ARTIFACT in next_hypernyms:
                    return True
                else:
                    todo.extend(next_hypernyms)

        return False

class _WordPair(object):
    """ This class is used for the Term Frequency analysis.
    """
    def __init__(self, word1, word2):
        """ Initializes the WordPair object. The arguments have to be
            to strings containing each a word. Do not use directly, but
            use the classmethod create.
        """
        self.word1 = word1
        self.word2 = word2

    @classmethod
    def create(cls, word1, word2):
        """ Returns a new class instance if word1 and word2 are valid
            words else None is returned.
        """
        # do some checking before creating a _WordPair instance
        if word1 in TermFrequency._FILTER or word2 in TermFrequency._FILTER:
            return None

        word1 = wordnet.morphy(word1, wordnet.NOUN)
        word2 = wordnet.morphy(word2, wordnet.NOUN)

        if word1 is None or word2 is None:
            return None

        # return the new instance
        return cls(word1, word2)

    def getMostRelatedSenses(self):
        synsetPairs = []
        mostRelatedSense = None

        # Generate all possible senses cominations for the two words
        for synset1 in wordnet.synsets(self.word1, wordnet.NOUN):
            for synset2 in wordnet.synsets(self.word2, wordnet.NOUN):
                synsetPairs.append(_SynsetPair(synset1, synset2))

        # Calculate the relatedness
        for synsetPair in synsetPairs:
            synsetPair.calcLSOAndDistance()

        # Get the sense combination with the shortest distance, i.e.
        # highest relatedness
        for synsetPair in synsetPairs:
            if mostRelatedSense is None:
                mostRelatedSense = synsetPair
            else:
                if mostRelatedSense.dist > synsetPair.dist:
                    mostRelatedSense = synsetPair

        return mostRelatedSense

class _SynsetPairError(Exception):
    """ General Exception in connection with _SynsetPair.
    """
    pass

class _SynsetPair(object):
    """ This class is used for the Term Frequency analysis.
    """
    def __init__(self, synset1, synset2):
        """ Initializes the SynsetPair object. The arguments have to be
            to instances of wordnet.Synset.
        """
        # Init of pair
        self.synset1 = synset1
        self.synset2 = synset2

        # Relevant variables for LSO/distance calc
        self.path1 = []
        self.path2 = []
        self.lso = []
        self.dist = -1

        # Relevant variables for scoring
        self.substitute = None
        self.score = -1

    def calcLSOAndDistance(self):
        """ Calculates the lowest super ordiantes (LSO), i.e. a list of
            nodes which are part of both paths to the root node. The
            list is empty if no connection could be found and is saved
            to self.lso.
        """
        # Caluculate the shortest paths
        self.path1 = self._pathToRoot(self.synset1)
        self.path2 = self._pathToRoot(self.synset2)

        # Find common part
        for i in xrange(min(len(self.path1), len(self.path2))):
            if self.path1[i] == self.path2[i]:
                self.lso.append(self.path1[i])

        self.dist = max(0, len(self.path1) - 2) + max(0, len(self.path2) - 2) - 2 * max(0, len(self.lso) - 2) + 1

    def substituteLSOAndCalcScore(self, lso):
        """ Substitutes the synsetPair with the given LSO and calculates
            the score of the LSO.

            Score = dist(synset1, root) + dist(synset2, root)
            - 2*dist(lso, root) + 1
        """
        # Small Error Check
        if lso not in self.lso:
            raise _SynsetPairError('Intended to find a LSO which is not in the list of possible LSOs.')

        # Substitute
        self.substitute = lso

        # Calculate score
        lsoDist = 0

        for lsoIter in self.lso:
            lsoDist += 1

            if lsoIter == lso:
                break

        self.score = 1.0 / (max(0, len(self.path1) - 2) + max(0, len(self.path2) - 2) - 2 * max(0, lsoDist - 2) + 1)

    def _pathToRoot(self, loc, seen=set()):
        """ Searches for the shortest path between the given node 'loc'
            and the root node, such that the number of nodes in between
            is minimal. The returned value is a list from the root node
            to the given node. If no path could be found an empty list
            is returned.
            As the function is used recursively the function should be
            used without the argument 'seen' which is used for the
            recursion.
        """
        # Break condition of recursion if it is successful
        if loc == TermFrequency._ROOT:
            return [loc]

        # Variable which contains the shortestPath
        shortestPath = []

        # Potential paths to look at
        next_hypernyms = loc.hypernyms() + loc.instance_hypernyms()

        # First things first add the new nodes to the list of
        # investigated nodes
        seenNew = seen.copy()

        for next_hypernym in next_hypernyms:
            seenNew.add(next_hypernym)

        for next_hypernym in next_hypernyms:
            # If not seen / checked already
            if next_hypernym not in seen:
                # Check the node
                potPath = self._pathToRoot(next_hypernym, seenNew)

                # If it is a valid, shortest path remember it
                if len(potPath) != 0 and (len(shortestPath) == 0 or \
                                    len(shortestPath) > len(potPath)):
                    shortestPath = potPath

        if len(shortestPath) != 0:
            shortestPath.append(loc)

        return shortestPath

    def __repr__(self):
        return str((self.score, self.synset1, self.synset2))
