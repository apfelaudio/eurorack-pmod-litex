#!/bin/bash
echo "libvult: transpile dsp to cpp"
./vultc -ccode examples/dsp.vult -real fixed -o gen/dsp
