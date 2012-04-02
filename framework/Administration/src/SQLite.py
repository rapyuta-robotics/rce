#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       SQLite.py
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
import sys
import os.path
import sqlite3

# Custom imports
import ThreadUtility
from MiscUtility import generateID

class SQLiteError(Exception):
    """ This error is raised if an error occurred in conjunction with
        an SQLite database.
    """
    pass

class SQLite(ThreadUtility.QueueWorker):
    """ This class is used to connect to a SQLite database.
    """ 
    def __init__(self, layout, dbName=':memory:'):
        """ Initialize the SQLite instance.
            
            @param layout:  Valid sql string which is executed once before
                            the main loop is entered. It is intended to
                            set up the database.
            @type  layout:  str
            
            @param dbName:  Optional argument. (default=':memory:')
                            Valid path (writeable) where the database will
                            be saved and from which the database is tried
                            to be loaded. If no argument is supplied the
                            created database will only be stored in RAM and
                            not on the harddisk. Therefore, after deleting
                            this class instance the database will be lost.
            @type  dbName:  str
        """
        super(SQLite, self).__init__()
        
        self._dbName = dbName
        self._sqlBase = layout
        self._connection = None
        self._cursor = None
    
    def init(self):
        """ This method sets up the database and the connection to the
            database. It overwrites the method from the base class.
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
        """ This method terminates the connection to the database. It
            overwrites the method from the base class.
        """
        try:
            self._connection.close()
        except sqlite3.Error:
            pass
    
    @ThreadUtility.QueueWorker.job
    def query(self, query, values=()):
        """ This method is used to run a query to search for entries.
            
            @param query:   Valid sql query string which can have '?'
                            as placeholders for values.
            @type  query:   str
            
            @param values:  Optional argument.
                            Tuple containing the values for the placeholders
                            in @param query. Make sure that the number of
                            values and placeholders match.
            @type  values:  tuple
            
            @return:    List of tuples where each tuple represents one entry.
            @rtype:     [(...)]
        """
        self._cursor.execute(query, values)
        return self._cursor.fetchall()
    
    @ThreadUtility.QueueWorker.job
    def change(self, query, values=()):
        """ This method is used to change some entries.
            
            @param query:   Valid sql query string which can have '?'
                            as placeholders for values.
            @type  query:   str
            
            @param values:  Optional argument.
                            Tuple containing the values for the placeholders
                            in @param query. Make sure that the number of
                            values and placeholders match.
            @type  values:  (...)
        """
        self._cursor.execute(query, values)
        self._connection.commit()
    
    @ThreadUtility.QueueWorker.job
    def newID(self, table, field):
        """ This method is used to generate a new unique ID.
            
            @param table:   Name of the table in which is searched and in
                            which the new entry with the new ID is entered.
            @type  table:   str
            
            @param field:   Field name which is used to search a unique ID
                            and in which the new ID is entered.
            @type  field:   str
            
            @return:    New unqiue ID
            @rtype:     str
        """
        while True:
            uid = generateID()
            self._cursor.execute('SELECT * FROM "{0}" WHERE "{1}" == ?'.format(table, field), (uid,))
            
            if len(self._cursor.fetchall()) == 0:
                # The given ID is not yet in the DB
                self._cursor.execute('INSERT INTO "{0}" ("{1}") VALUES (?)'.format(table, field), (uid,))
                
                self._connection.commit()
                break
        
        return uid
