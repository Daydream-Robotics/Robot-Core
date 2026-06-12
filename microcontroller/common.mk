SDK_PATH := $(shell xcrun --show-sdk-path)

CXX := g++

CXXFLAGS := \
    -std=c++17 \
    -O2 \
    -Iinclude \
    -Iinclude/osqp \
    -Iinclude/eigen \
    -I$(SDK_PATH)/usr/include/c++/v1 \
    -I$(SDK_PATH)/usr/include

LDFLAGS := \
    firmware/libosqpstatic.a \
    -lm