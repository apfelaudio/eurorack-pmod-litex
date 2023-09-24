#!/bin/bash
riscv64-linux-gnu-gcc -O3 -mabi=ilp32 -march=rv32im -fno-builtin -nostdlib gen/dsp.cpp -I gen -I runtime -I../../deps/pythondata-software-picolibc/pythondata_software_picolibc/data/newlib/libc/include -I../../build/lambdaconcept_ecpix5/software/libc -c dsp.o
riscv64-linux-gnu-gcc -O3 -mabi=ilp32 -march=rv32im -fno-builtin -nostdlib runtime/vultin.cpp -I gen -I runtime -I../../deps/pythondata-software-picolibc/pythondata_software_picolibc/data/newlib/libc/include -I../../build/lambdaconcept_ecpix5/software/libc -c vultin.o
ar cr libvult.a dsp.o vultin.o
cp gen/dsp.h gen/dsp.hpp
