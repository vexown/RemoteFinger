
add_executable(main_blinky
        main.c
        main_blinky.c
        IntQueueTimer.c
        RegTest.s
        ../../Common/Minimal/blocktim.c
        ../../Common/Minimal/countsem.c
        ../../Common/Minimal/dynamic.c
        ../../Common/Minimal/recmutex.c
        ../../Common/Minimal/QueueOverwrite.c
        ../../Common/Minimal/EventGroupsDemo.c
        ../../Common/Minimal/IntSemTest.c
        ../../Common/Minimal/IntQueue.c
        ../../Common/Minimal/TaskNotify.c
        ../../Common/Minimal/TimerDemo.c
        ../../Common/Minimal/GenQTest.c
        ../../Common/Minimal/death.c
        ../../Common/Minimal/semtest.c
        ../../Common/Minimal/BlockQ.c
        ../../Common/Minimal/flop.c
        )

target_compile_definitions(main_blinky PRIVATE)

target_include_directories(main_blinky PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/../../Common/include)

#pull in common dependencies such as pico stdlib, FreeRTOS kernel stuff and additional i2c hardware support
target_link_libraries(main_blinky pico_stdlib hardware_i2c FreeRTOS-Kernel FreeRTOS-Kernel-Heap1)
pico_add_extra_outputs(main_blinky)



