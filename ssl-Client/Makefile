CC = g++
CFLAGS = -Wall -Wextra

# system library links
GLOBAL_LINKS = -L/usr/local/lib/ -lpthread -lprotobuf -lz

# local source files and objects
SOURCES = net/robocup_ssl_client.cpp net/netraw.cpp net/grSim_client.cpp net/pb/common.pb.cc header.cpp analyzer.cpp
OBJECTS = robocup_ssl_client.o netraw.o grSim_client.o common.pb.o header.o analyzer.o

# compile all
all: main

# compile all abjects
$(OBJECTS): $(SOURCES)
	$(CC) -c $^ $(CFLAGS) $(GLOBAL_LINKS)

# compile the main object
main.o: main.cpp
	$(CC) -c main.cpp $(CFLAGS) $(GLOBAL_LINKS)

# link the main file
main: $(OBJECTS) main.o
	$(CC) main.o $(OBJECTS) $(GLOBAL_LINKS) -o $@

# clean objects
clean: 
	rm -rf $(OBJECTS) main.o

# clean all non source files
purge: clean
	rm -rf main