cmake_minimum_required(VERSION 3.22)
project(vision_protocol_tests C)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
add_executable(vision_protocol_tests
    test_vision_protocol.c
    ../../Skywalker/Application/Src/app_protocol.c
    ../../Skywalker/Algorithm/Src/alg_crc.c
)
target_include_directories(vision_protocol_tests PRIVATE
    stubs
    ../../Skywalker/Application/Inc
    ../../Skywalker/Algorithm/Inc
)
target_compile_options(vision_protocol_tests PRIVATE -Wall -Wextra -Werror)
enable_testing()
add_test(NAME vision_protocol COMMAND vision_protocol_tests)
