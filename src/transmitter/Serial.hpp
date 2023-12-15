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
        Serial(const std::string& port_name, const termios& tty);

        ~Serial();

        void write(unsigned char msg[], std::size_t length) const;
        int tryRead(std::vector<unsigned char>& buffer, std::size_t length) const;

        static std::vector<std::string> getDevices();
    private:
        int serialPortFD;
    };
}

#endif //STEPPER_MOTOR_SYMPHONY_V2_SERIAL_HPP
