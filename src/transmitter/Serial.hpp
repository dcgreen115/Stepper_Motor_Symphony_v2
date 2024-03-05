//
// Created by dylangreen on 12/13/23.
//

#ifndef STEPPER_MOTOR_SYMPHONY_V2_SERIAL_HPP
#define STEPPER_MOTOR_SYMPHONY_V2_SERIAL_HPP

#include <string>
#include <vector>
#include <termios.h>

namespace sms {
    class Serial {
    public:
        Serial() = delete;
        Serial(const std::string& portName, const termios& tty);

        ~Serial();

        void write(const void* data, unsigned int size) const;
        int tryRead(void* buffer, unsigned int size) const;

        static std::vector<std::string> getDevices();
    private:
        int serialPortFD;
    };
}

#endif //STEPPER_MOTOR_SYMPHONY_V2_SERIAL_HPP
