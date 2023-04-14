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
echo Building...
cmake --build .

echo "Press 'X' to exit the script."

while true; do
  read -n 1 key  # wait for user to press a key
  if [[ $key == "X" || $key == "x" ]]; then  # check if key pressed is 'X' or 'x'
    echo "Exiting the script."
    exit 0
  fi
done


