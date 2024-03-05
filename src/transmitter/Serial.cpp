//
// Created by dylangreen on 12/13/23.
//
#include <filesystem>
#include <iostream>
#include "Serial.hpp"
#include "Exceptions.hpp"

// Linux-specific headers
#include <fcntl.h>
#include <cerrno>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <sys/ioctl.h>

sms::Serial::Serial(const std::string& portName, const termios& tty) {
    serialPortFD = open(portName.c_str(), O_RDWR);

    // There was an error opening the port
    if (serialPortFD < 0) {
        throw SerialPortException(std::strerror(serialPortFD));
    }

    // See if the tty settings will work with the port
    if (tcsetattr(serialPortFD, TCSANOW, &tty) != 0) {
        throw SerialPortException(std::strerror(errno));
    }
}

sms::Serial::~Serial() {
    close(serialPortFD);
}

/**
 * Gets all the serial ports that currently have a device connected
 * Note: This method will only work on Linux-based machines
 * @return A vector of strings showing each serial port that has a device connected
 */
std::vector<std::string> sms::Serial::getDevices() {
    namespace fs = std::filesystem;
    std::vector<std::string> devices;

    fs::path p("/dev/serial/by-id");
    try {
        if (!exists(p)) {
            return devices;
        } else {
            for (const auto& de : fs::directory_iterator(p)) {
                if (is_symlink(de.symlink_status())) {
                    fs::path symlink_points_at = read_symlink(de);
                    fs::path canonical_path = fs::canonical(p / symlink_points_at);
                    devices.push_back(canonical_path.generic_string());
                }
            }
        }
    } catch (const fs::filesystem_error &ex) {
        std::cerr << ex.what() << '\n';
    }

    return devices;
}

void sms::Serial::write(const void* const data, unsigned int size) const {
    if (::write(serialPortFD, data, size) == -1) {
        throw SerialPortException(std::strerror(errno));
    }
}

int sms::Serial::tryRead(void* const buffer, unsigned int size) const {
    int bytesAvailable;
    ioctl(serialPortFD, FIONREAD, &bytesAvailable);

    if (bytesAvailable < size) {
        return -1;
    }

    if (::read(serialPortFD, buffer, size) == -1) {
        throw SerialPortException(std::strerror(errno));
    }
    return 0;
}

