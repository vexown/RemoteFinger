
message("########## Application/Standard CMakeLists.txt - start ##########")
add_executable(RemoteFinger_main
        Source/main.c
        Source/RemoteFinger_main.c
        Source/server_common.c
        )

target_compile_definitions(RemoteFinger_main PRIVATE)

target_include_directories(RemoteFinger_main PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/Include
        ${CMAKE_CURRENT_LIST_DIR}/../../Common/include)

#pull in common dependencies such as pico stdlib, FreeRTOS kernel stuff and additional i2c hardware support
target_link_libraries(RemoteFinger_main 
                        pico_stdlib hardware_i2c 
                        pico_cyw43_arch_none 
                        pico_btstack_ble
                        pico_btstack_cyw43
                        FreeRTOS-Kernel 
                        FreeRTOS-Kernel-Heap1)
pico_add_extra_outputs(RemoteFinger_main)

pico_btstack_make_gatt_header(RemoteFinger_main PRIVATE "${CMAKE_CURRENT_LIST_DIR}/Source/BluetoothProfileConfig.cfg")

message("########## Application/Standard CMakeLists.txt - end ##########")


