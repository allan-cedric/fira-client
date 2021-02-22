CXX = g++
CXXFLAGS = -Wall -Wextra
LDFLAGS = -L/usr/local/lib/ -lpthread -lprotobuf -lz

OBJECTS = robocup_ssl_client.o netraw.o grSim_client.o common.pb.o path_planning.o math_operations.o analyzer.o main.o
TEST_OBJECTS = robocup_ssl_client.o netraw.o grSim_client.o common.pb.o path_planning.o math_operations.o analyzer.o test_path.o

# compile all
all: main

robocup_ssl_client.o: net/robocup_ssl_client.cpp
	$(CXX) -c $<
netraw.o: net/netraw.cpp
	$(CXX) -c $<
grSim_client.o: net/grSim_client.cpp
	$(CXX) -c $<
common.pb.o: net/pb/common.pb.cc
	$(CXX) -c $<

path_planning.o: path_planning.cpp path_planning.h
math_operations.o: math_operations.cpp math_operations.h
analyzer.o: analyzer.cpp analyzer.h
main.o: main.cpp
test_path.o: test_path.cpp

main: $(OBJECTS) header.h
	$(CXX) $(OBJECTS) $(CXXFLAGS) $(LDFLAGS) -o main

test_path: $(TEST_OBJECTS)
	$(CXX) $(TEST_OBJECTS) $(CXXFLAGS) $(LDFLAGS) -o test_path
	
# clean objects
clean: 
	rm -rf *.o

# clean all non source files
purge: clean
	rm -rf main test_path