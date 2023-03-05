#!/bin/bash

echo "Building all components with CMake"

if [ ! -d "build" ]; then
    mkdir build
    echo "Created build directory"
else
    echo "Build directory already exists"
fi

#configure build directory
echo Configuring the build directory...
cd build
cmake ..

#build the project
echo Building...
cmake --build .



