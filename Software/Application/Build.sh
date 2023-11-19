#!/bin/bash

echo "########## Build.sh - start ##########"
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
#When building for a board other than the Raspberry Pi Pico, you should pass `-DPICO_BOARD=board_name` 
#to the `cmake` command, e.g. `cmake -DPICO_BOARD=pico_w ..` to configure the SDK and build options accordingly for that particular board.
cmake -DPICO_BOARD=pico_w ..

#build the project
status=0
echo Building...
cmake --build .

if [ $? -eq 0 ]; then
    echo "Build successful"
    exit $status
else
    echo "Build failed"
    status=1
    echo "Press any key to exit..."
    read -n 1 -s  # Read only one character silently (-s option)
    exit $status
fi



