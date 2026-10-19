 
include common.mk

TARGET := MPCController

SRC := src/main.cpp src/motion/control/mpc.cpp src/link/serialProtocol.cpp

all:
	$(CXX) $(SRC) $(CXXFLAGS) $(LDFLAGS) -o $(TARGET)

run: all
	./$(TARGET)

clean:
	rm -f $(TARGET)
 
