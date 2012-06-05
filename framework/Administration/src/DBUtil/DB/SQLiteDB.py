#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       SQLiteDB.py
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

# Python specific imports
import os.path
import sqlite3

# Custom imports
from ThreadUtil import CentralizedLoop

class SQLiteError(Exception):
    """ This error is raised if an error occurred in conjunction with
        an SQLite database.
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
