 
include common.mk

TARGET := MPCController

SRC := src/main.cpp src/daydream/nano/motion/control/mpc.cpp src/daydream/nano/link/serialProtocol.cpp

all:
	$(CXX) $(SRC) $(CXXFLAGS) $(LDFLAGS) -o $(TARGET)

run: all
	./$(TARGET)

clean:
	rm -f $(TARGET)
 
