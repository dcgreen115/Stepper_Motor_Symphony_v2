cmake_minimum_required(VERSION 3.22)

set(CMAKE_CXX_STANDARD 20)

set(LIB_SOURCE
        Serial.hpp Serial.cpp
        MessageQueue.hpp MessageQueue.tpp
        Exceptions.hpp)

set(LIB_TEST_SOURCE
        SerialTests.cpp MessageQueueTests.cpp)

set(TRANSMITTER_SOURCE
        main.cpp)

add_executable(transmitter ${TRANSMITTER_SOURCE} ${LIB_SOURCE})

target_link_libraries(transmitter libremidi)

# Add tests
find_package(Catch2 3)
if (Catch2_FOUND)
    add_executable(unit_tests ${LIB_TEST_SOURCE} ${LIB_SOURCE})
    target_link_libraries(unit_tests PRIVATE Catch2::Catch2WithMain)

    include(CTest)
    include(Catch)
    catch_discover_tests(unit_tests)
endif()
