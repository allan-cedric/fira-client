CC = g++

CFLAGS = -Wall -Wextra

LOCAL_DEPENDENCIES = net/robocup_ssl_client.cpp net/netraw.cpp net/grSim_client.cpp net/pb/common.pb.cc
GLOBAL_LINKS = -L/usr/local/lib/ -lpthread -lprotobuf -lz

main: main.cpp $(LOCAL_DEPENDENCIES)
	$(CC) main.cpp $(LOCAL_DEPENDENCIES) -o $@ $(GLOBAL_LINKS)
