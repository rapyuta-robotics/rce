# Configuration
PREFIX=$(HOME)/usr

CC=g++
CFLAGS=-c -Wall
LDFLAGS=-L$(PREFIX)/lib
INCLUDES=-I../include -I$(PREFIX)/include
LIBS=-lcurl -lboost_system -lboost_regex -lboost_random -lboost_thread

all: test

test: test.o
	$(CC) $(LDFLAGS) test.o $(LIBS) -o test

test.o: test.cxx
	$(CC) $(CFLAGS) $(INCLUDES) test.cxx

clean:
	rm -rf *.o test
