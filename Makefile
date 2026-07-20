# 3dt - STL/STEP viewer (Windows build, MinGW-w64, no external deps)
# Build:  mingw32-make          (release)
#         mingw32-make DEBUG=1  (debug)
# For Linux use CMake (see README.md): the wildcard below also compiles
# src/platform_x11.cpp, which is an empty translation unit on _WIN32.

CXX      := C:/mingw64/bin/g++.exe
CXXFLAGS := -std=c++17 -Wall -Wextra -municode
LDFLAGS  := -mwindows -municode -static
LIBS     := -lopengl32 -lgdi32 -luser32 -lcomdlg32 -lshell32

ifeq ($(DEBUG),1)
  CXXFLAGS += -g -O0
else
  CXXFLAGS += -O2
  LDFLAGS  += -s
endif

SRC := $(wildcard src/*.cpp)
OBJ := $(patsubst src/%.cpp,build/%.o,$(SRC))
BIN := build/3dt.exe

all: $(BIN)

build:
	-mkdir build

$(BIN): $(OBJ) | build
	$(CXX) $(LDFLAGS) -o $@ $(OBJ) $(LIBS)

build/%.o: src/%.cpp src/mesh.h src/platform.h | build
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	-rmdir /S /Q build

.PHONY: all clean
