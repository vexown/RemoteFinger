
message("########## Application/Standard CMakeLists.txt - start ##########")
add_executable(blinky_main
        main.c
        blinky_main.c
        )

target_compile_definitions(blinky_main PRIVATE)

target_include_directories(blinky_main PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/../../Common/include)

#pull in common dependencies such as pico stdlib, FreeRTOS kernel stuff and additional i2c hardware support
target_link_libraries(blinky_main pico_stdlib FreeRTOS-Kernel FreeRTOS-Kernel-Heap1)
pico_add_extra_outputs(blinky_main)

message("########## Application/Standard CMakeLists.txt - end ##########")


