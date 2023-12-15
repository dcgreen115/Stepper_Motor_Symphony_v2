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

    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

    tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

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

    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

    tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

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

    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

    tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    // Set in/out baud rate to be 115200
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    auto serial = sms::Serial("/dev/ttyUSB0", tty);

    unsigned char msg[] = {'H', 'e', 'l', 'l', 'o', '\n'};
    REQUIRE_NOTHROW(serial.write(msg, sizeof(msg)));

    std::vector<unsigned char> buffer;
    REQUIRE_NOTHROW([&](){
        while (serial.tryRead(buffer, sizeof(msg)) == -1);
    }());

    REQUIRE(buffer.size() == 6);
    REQUIRE(buffer.at(0) == 'H');
    REQUIRE(buffer.at(1) == 'e');
    REQUIRE(buffer.at(2) == 'l');
    REQUIRE(buffer.at(3) == 'l');
    REQUIRE(buffer.at(4) == 'o');
    REQUIRE(buffer.at(5) == '\n');
}