# BOARD should be set by top-level makefile / build script.
ifndef BOARD
$(error BOARD is not set)
endif

# Define source files and objects
VULT_TARGET = dsp
VULT_SRC = src/$(VULT_TARGET).vult
CPP_SRC = gen/$(VULT_TARGET).cpp runtime/vultin.cpp
OBJ = $(CPP_SRC:.cpp=.o)
ARCH = rv32i

# Define compiler and flags
CC = riscv64-linux-gnu-gcc
CFLAGS = -O3 -mabi=ilp32 -march=$(ARCH) -fno-builtin -nostdlib
INCLUDES = \
   -I gen \
   -I runtime \
   -I../../build/$(BOARD)/software/libc \
   -I../../deps/pythondata-software-picolibc/pythondata_software_picolibc/data/newlib/libc/include

# Define the target
TARGET = build/libvult.a

# Phony targets
.PHONY: all clean

all: $(TARGET)

gen/$(VULT_TARGET).cpp gen/$(VULT_TARGET.h): $(VULT_SRC)
	./vultc -ccode $< -real fixed -o gen/$(VULT_TARGET)

%.o: %.cpp
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

$(TARGET): $(OBJ)
	mkdir -p build/
	cp gen/dsp.h build/dsp.hpp
	cp gen/dsp.tables.h build/dsp.tables.h
	ar cr $@ $^

clean:
	rm -f $(OBJ) $(TARGET) gen/* build/*
