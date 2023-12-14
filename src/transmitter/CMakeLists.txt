cmake_minimum_required(VERSION 3.22)
project(Stepper_Motor_Symphony_v2)

set(CMAKE_CXX_STANDARD 20)

add_executable(transmitter main.cpp)

target_link_libraries(transmitter libremidi)
