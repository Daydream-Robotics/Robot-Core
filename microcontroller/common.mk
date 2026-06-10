```make
CXX := g++

CXXFLAGS := \
    -std=c++17 \
    -O2 \
    -Iinclude/osqp

LDFLAGS := \
    firmware/libosqpstatic.a \
    -lm
```
