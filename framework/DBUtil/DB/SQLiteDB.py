#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       SQLiteDB.py
#       
#       This file is part of the RoboEarth Cloud Engine framework.
#       
#       This file was originally created for RoboEearth - http://www.roboearth.org/
#       The research leading to these results has received funding from the European Union 
#       Seventh Framework Programme FP7/2007-2013 under grant agreement no248942 RoboEarth.
#       
#       Copyright 2012 RoboEarth
#       
#       Licensed under the Apache License, Version 2.0 (the "License");
#       you may not use this file except in compliance with the License.
#       You may obtain a copy of the License at
#       
#       http://www.apache.org/licenses/LICENSE-2.0
#       
#       Unless required by applicable law or agreed to in writing, software
#       distributed under the License is distributed on an "AS IS" BASIS,
#       WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#       See the License for the specific language governing permissions and
#       limitations under the License.
#       
#       \author/s: 
#       
#       

# Python specific imports
import os.path
import sqlite3

# Custom imports
from ThreadUtil import CentralizedLoop

class SQLiteError(Exception):
    """ This error is raised if an error occurred in conjunction with
        a SQLite database.
    """
    pass

class SQLite(CentralizedLoop):
    """ This class is used to connect to a SQLite database.
    """
    def __init__(self, reactor, layout, dbName=':memory:'):
        """ Initialize the SQLite instance.

            @param layout:  Valid sql string which is executed once before
                            the main loop is entered. It is intended to
                            set up the database.
            @type  layout:  str

            @param dbName:  Optional argument. (default=':memory:')
                            Valid path (writable) where the database will
                            be saved and from which the database is tried
                            to be loaded. If no argument is supplied the
                            created database will only be stored in RAM and
                            not on the hard-disk. Therefore, after deleting
                            this class instance the database will be lost.
            @type  dbName:  str
        """
        super(SQLite, self).__init__(reactor)

        self._dbName = dbName
        self._sqlBase = layout
        self._connection = None
        self._cursor = None

    def init(self):
        """ This method sets up the database and the connection to the database.
            It overwrites the method from the base class.
        """
        # If the database does not exist create it
        if not os.path.exists(self._dbName):
            try:
                self._connection = sqlite3.connect(self._dbName)
                self._connection.executescript(self._sqlBase)
                self._connection.commit()
            except sqlite3.Error:
                raise SQLiteError('Could not create the database {0}.'.format(self._dbName))
        else:
            try:
                self._connection = sqlite3.connect(self._dbName)
            except sqlite3.Error:
                raise SQLiteError('Could not connect to the database {0}.'.format(self._dbName))

        # Open cursor
        self._cursor = self._connection.cursor()

    def stop(self):
        """ This method terminates the connection to the database. It overwrites
            the method from the base class.
        """
        try:
            self._connection.close()
        except sqlite3.Error:
            pass

    @CentralizedLoop.job
    def query(self, query, values=()):
        """ This method is used to run a query to search for entries.

            @param query:   Valid sql query string which can have '?'
                            as place holders for values.
            @type  query:   str

            @param values:  Optional argument.
                            Tuple containing the values for the place holders
                            in @param query. Make sure that the number of
                            values and place holders match.
            @type  values:  tuple

            @return:    Deferred with a list of tuples where each tuple represents
                        one entry as callback argument.
            @rtype:     Deferred; [(...)]
        """
        self._cursor.execute(query, values)
        return self._cursor.fetchall()

    @CentralizedLoop.job
    def change(self, query, values=()):
        """ This method is used to change some entries.

            @param query:   Valid sql query string which can have '?'
                            as place holders for values.
            @type  query:   str

            @param values:  Optional argument.
                            Tuple containing the values for the place holders
                            in @param query. Make sure that the number of
                            values and place holders match.
            @type  values:  (...)
        """
        self._cursor.execute(query, values)
        self._connection.commit()
