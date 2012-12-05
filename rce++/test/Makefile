# Configuration
#BOOST_INCLUDE=/home/dominique/Desktop/roomba/boost_1_52_0
#BOOST_LIB=/home/dominique/Desktop/roomba/boost_1_52_0/stage/lib

#WEBSOCKETPP_INCLUDE=/home/dominique/Desktop/roomba/websocketpp/src
#WEBSOCKETPP_LIB=/home/dominique/Desktop/roomba/websocketpp


CC=g++
CFLAGS=-c -Wall
LDFLAGS=-L/usr/local/lib -L$(BOOST_LIB) -L$(WEBSOCKETPP_LIB)
INCLUDES=-I ../include -I $(BOOST_INCLUDE) -I$(WEBSOCKETPP_INCLUDE)
LIBS=-lcurl -lwebsocketpp -lboost_system -lboost_date_time -lboost_regex -lboost_random -lboost_thread

all: test

test: test.o
	$(CC) $(LDFLAGS) test.o $(LIBS) -o test

test.o: test.cxx
	$(CC) $(CFLAGS) $(INCLUDES) test.cxx

clean:
	rm -rf *.o test
