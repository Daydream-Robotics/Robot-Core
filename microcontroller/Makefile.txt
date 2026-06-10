```make
include common.mk

TARGET := osqp_test

SRC := main.cpp

all:
	$(CXX) $(SRC) $(CXXFLAGS) $(LDFLAGS) -o $(TARGET)

run: all
	./$(TARGET)

clean:
	rm -f $(TARGET)
```
