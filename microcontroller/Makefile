 
include common.mk

TARGET := osqp_test

SRC := src/microcontroller_main.cpp src/mpc.cpp src/serial_protocol.cpp

all:
	$(CXX) $(SRC) $(CXXFLAGS) $(LDFLAGS) -o $(TARGET)

run: all
	./$(TARGET)

clean:
	rm -f $(TARGET)
 
