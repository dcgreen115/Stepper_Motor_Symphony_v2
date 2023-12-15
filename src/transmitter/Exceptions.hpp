//
// Created by dylangreen on 12/14/23.
//

#ifndef STEPPER_MOTOR_SYMPHONY_V2_EXCEPTIONS_HPP
#define STEPPER_MOTOR_SYMPHONY_V2_EXCEPTIONS_HPP

#include <exception>

class SerialPortException : public std::exception {
public:
    explicit SerialPortException(const char* msg) : message(msg) {};

    const char* what() const noexcept override {
        return message;
    }
private:
    const char* message;
};

#endif //STEPPER_MOTOR_SYMPHONY_V2_EXCEPTIONS_HPP
