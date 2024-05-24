#!/bin/bash

mkdir -p build
cd build
cmake .. >/dev/null 2>&1
make -j$(( $(nproc) - 1 ))  -s || echo "Error occurred during make"
cd ..
./build/example/test1