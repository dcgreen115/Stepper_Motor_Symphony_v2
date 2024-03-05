//
// Created by dylangreen on 12/14/23.
//

#include <catch2/catch_test_macros.hpp>
#include "Serial.hpp"
#include "Exceptions.hpp"

TEST_CASE("Test getDevices", "[Serial]") {
    auto devices = sms::Serial::getDevices();
    if (devices.size() != 1) {
        SKIP("Requires only the ESP8266 connected");
    }

    REQUIRE(devices.size() == 1);
    REQUIRE(devices.at(0) == "/dev/ttyUSB0");
}

TEST_CASE("Test starting serial connection in constructor", "[Serial]") {
    auto devices = sms::Serial::getDevices();
    if (devices.size() != 1) {
        SKIP("Requires only the ESP8266 connected");
    }

    struct termios tty{};
    memset(&tty, 0, sizeof(tty));
    tty.c_cflag |= (CS8 | CREAD | CLOCAL);

    // Set in/out baud rate to be 115200
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    SECTION("Constructor with valid serial port name") {
        REQUIRE_NOTHROW(sms::Serial("/dev/ttyUSB0", tty));
    }
    SECTION ("Constructor with invalid serial port name") {
        REQUIRE_THROWS_AS(sms::Serial("/dev/ttyUSB1000", tty), SerialPortException);
    }
}

TEST_CASE("Test sending data over Serial connection", "[Serial]") {
    auto devices = sms::Serial::getDevices();
    if (devices.size() != 1) {
        SKIP("Requires only the ESP8266 connected");
    }

    struct termios tty{};
    memset(&tty, 0, sizeof(tty));
    tty.c_cflag |= (CS8 | CREAD | CLOCAL);

    // Set in/out baud rate to be 115200
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    auto serial = sms::Serial("/dev/ttyUSB0", tty);

    unsigned char msg[] = {'H', 'e', 'l', 'l', 'o', '\n'};
    REQUIRE_NOTHROW(serial.write(msg, sizeof(msg)));
}

TEST_CASE("Test sending and receiving data over serial connection", "[Serial]") {
    auto devices = sms::Serial::getDevices();
    if (devices.size() != 1) {
        SKIP("Requires only the ESP8266 connected");
    }

    struct termios tty{};
    memset(&tty, 0, sizeof(tty));
    tty.c_cflag |= (CS8 | CREAD | CLOCAL);

    // Set in/out baud rate to be 115200
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    auto serial = sms::Serial("/dev/ttyUSB0", tty);

    char msg[] = {'H', 'e', 'l', 'l', 'o', '\n'};
    REQUIRE_NOTHROW(serial.write(msg, sizeof(msg)));

    char buffer[6];
    REQUIRE_NOTHROW([&](){
        while (serial.tryRead(buffer, sizeof(msg)) == -1);
    }());

    REQUIRE(buffer[0] == 'H');
    REQUIRE(buffer[1] == 'e');
    REQUIRE(buffer[2] == 'l');
    REQUIRE(buffer[3] == 'l');
    REQUIRE(buffer[4] == 'o');
    REQUIRE(buffer[5] == '\n');
}