#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy
import pylab
import json


def main(fileNames, ylog, remote, plotStyle):
    if ylog:
        pylab.subplot(111, xscale='log', yscale='log')
    else:
        pylab.subplot(111, xscale='log')
    
    for fileName in fileNames:
        with open(fileName, 'r') as f:
            size = numpy.array(json.loads(f.readline())[1:])
            benchmark = map(json.loads, f.readlines())
        
        for test in benchmark:
            if not remote and test['label'].endswith('R-to-C'):
                continue
            
            rawData = test.pop('data')
            data = [d for d in rawData if -1.0 not in d]
            length = len(data)
            
            if length != len(rawData):
                print('Invalid measurment encountered in {0}.'.format(test['label']))
            
            data = numpy.array(data)
            
            if length > 1:
                mean = numpy.mean(data, 0)[1:]
                var = numpy.var(data, 0)[1:]
                minVal = numpy.min(data, 0)[1:]
                maxVal = numpy.max(data, 0)[1:]
            elif length == 1:
                mean = numpy.array(data[0][1:])
                var = None
            else:
                print('No valid measurments for {0}.'.format(test['label']))
                continue
            
            if plotStyle == 'minmax':
                pylab.errorbar(size, mean, numpy.array([minVal, maxVal]), **test)
            elif plotStyle == 'variance':
                pylab.errorbar(size, mean, var, **test)
            else:
                pylab.errorbar(size, mean, **test)
    
    if 'paper.data' in fileNames:
        pylab.figtext(0.8, 0.8, 'green: paper measurements', ha='right')
    
    pylab.xlabel('number of characters')
    pylab.ylabel('time [ms]')
    pylab.legend(loc='upper left')
    pylab.show()


def _get_argparse():
    from argparse import ArgumentParser
    
    parser = ArgumentParser(prog='benchmark',
                            description='Run communication benchmark using a '
                                        'string message for RCE.')
    
    parser.add_argument('--ylog', help='Flag to activate log scale for y axis.',
                        action='store_true')
    parser.add_argument('--show', help='Flag to activate the display the R-to-C'
                                       ' measurements.', action='store_true')
    parser.add_argument('--errorbar', help='Argument which is used to define '
                                           'the type of errorbar to display.',
                        type=str, choices=('minmax', 'variance'))
    parser.add_argument('--paper', help='Flag to add measurement from paper for'
                                        ' comparison.', action='store_true')
    
    return parser


if __name__ == '__main__':
    args = _get_argparse().parse_args()
    
    files = ['benchmark.data']
    
    if args.paper:
        files.append('paper.data')
    
    main(files, args.ylog, args.show, args.errorbar)
