 
include common.mk

TARGET := MPCController

SRC := src/microcontroller_main.cpp src/mpc.cpp src/daydream/utils/serial/serialProtocol.cpp

all:
	$(CXX) $(SRC) $(CXXFLAGS) $(LDFLAGS) -o $(TARGET)

run: all
	./$(TARGET)

clean:
	rm -f $(TARGET)
 
