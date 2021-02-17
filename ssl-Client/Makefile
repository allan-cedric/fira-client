CC = g++
CFLAGS = -Wall -Wextra

GLOBAL_LINKS = -L/usr/local/lib/ -lpthread -lprotobuf -lz

SOURCES = net/robocup_ssl_client.cpp net/netraw.cpp net/grSim_client.cpp net/pb/common.pb.cc
OBJECTS = robocup_ssl_client.o netraw.o grSim_client.o common.pb.o

all: main

$(OBJECTS): $(SOURCES)
	$(CC) -c $^ $(CFLAGS) $(GLOBAL_LINKS)

main.o: main.cpp
	$(CC) -c main.cpp $(CFLAGS) $(GLOBAL_LINKS)

main: $(OBJECTS) main.o
	$(CC) main.o $(OBJECTS) $(GLOBAL_LINKS) -o $@

clean: 
	rm -rf $(OBJECTS) main.o

purge: clean
	rm -rf main