UNAME_S := $(shell uname -s)

CXX := g++

CXXFLAGS := \
    -std=c++17 \
    -O2 \
    -Iinclude \
    -Iinclude/osqp \
    -Iinclude/eigen

# macOS needs SDK path for standard headers
ifeq ($(UNAME_S),Darwin)
    SDK_PATH := $(shell xcrun --show-sdk-path)
    CXXFLAGS += -I$(SDK_PATH)/usr/include/c++/v1 -I$(SDK_PATH)/usr/include
endif

LDFLAGS := \
    firmware/libosqpstatic.a \
    -lm
