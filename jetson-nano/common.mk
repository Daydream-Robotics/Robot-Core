UNAME_S := $(shell uname -s)

CXX := g++

CXXFLAGS := \
    -std=c++17 \
    -O2 \
    -Iinclude \
    -I../shared/include \
    -Iexternal/osqp/include/osqp \
    -Iexternal/eigen/include

# macOS needs SDK path for standard headers
ifeq ($(UNAME_S),Darwin)
    SDK_PATH := $(shell xcrun --show-sdk-path)
    CXXFLAGS += -I$(SDK_PATH)/usr/include/c++/v1 -I$(SDK_PATH)/usr/include
endif

LDFLAGS := \
    external/osqp/lib/libosqpstatic.a \
    -lm
