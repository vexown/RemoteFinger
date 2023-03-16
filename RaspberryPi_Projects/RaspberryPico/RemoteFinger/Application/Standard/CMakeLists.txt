
message("########## Application/Standard CMakeLists.txt - start ##########")
add_executable(RemoteFinger_main
        main.c
        RemoteFinger_main.c
        )

target_compile_definitions(RemoteFinger_main PRIVATE)

target_include_directories(RemoteFinger_main PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/../../Common/include)

#pull in common dependencies such as pico stdlib, FreeRTOS kernel stuff and additional i2c hardware support
target_link_libraries(RemoteFinger_main pico_stdlib hardware_i2c FreeRTOS-Kernel FreeRTOS-Kernel-Heap1)
pico_add_extra_outputs(RemoteFinger_main)

message("########## Application/Standard CMakeLists.txt - end ##########")


