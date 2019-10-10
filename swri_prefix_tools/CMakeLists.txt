cmake_minimum_required(VERSION 3.10)
project(swri_prefix_tools)

find_package(ament_cmake REQUIRED)

install(PROGRAMS xterm_prefix
                 xterm_prefix_callgrind
                 xterm_prefix_callgrind_wait
                 xterm_prefix_gdb
                 xterm_prefix_hold
                 xterm_prefix_normal
        DESTINATION bin
)

ament_package()
