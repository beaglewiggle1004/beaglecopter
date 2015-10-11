#all: 
#	g++ control.cpp dumpData.cpp itg3200.cpp PruProxy.cpp -o control

CC=g++
CFLAGS=-c -w -fPIC
#CFLAGS=
LDFLAGS=-shared -Wl,-soname,$(SHAREDOBJ)
SOURCES=control.cpp PruProxy.cpp I2Cdev.cpp MPU6050.cpp Receiver.cpp BMP085.cpp HMC5883L.cpp pid.cpp
OBJECTS=$(SOURCES:.cpp=.o)
EXECUTABLE=controls
SHAREDOBJ=libcontrols.so

#all: $(SOURCES) $(EXECUTABLE)
all: $(SOURCES) $(SHAREDOBJ)
	
$(SHAREDOBJ): $(OBJECTS) 
	$(CC) $(LDFLAGS) $(OBJECTS)  -lrt -lpthread -o $@ 

$(EXECUTABLE): $(OBJECTS) 
	$(CC) $(LDFLAGS) $(OBJECTS)  -lrt -lpthread -o $@ 

.cpp.o:
	$(CC) $(CFLAGS) $< -o $@
	
