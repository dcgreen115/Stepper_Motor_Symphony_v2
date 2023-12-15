#ifndef STEPPER_MOTOR_SYMPHONY_V2_MESSAGEQUEUE_HPP
#define STEPPER_MOTOR_SYMPHONY_V2_MESSAGEQUEUE_HPP

#include <cstdlib>
#include <queue>
#include <mutex>
#include <condition_variable>

namespace sms {
    template<typename T>
    class MessageQueue {
    public:
        std::size_t size() const noexcept;

        void push(const T& value);

        bool tryPop(T& value) noexcept;

        void waitPop(T& value) noexcept;

    private:
        // The backing queue
        std::queue<T> data;

        // For thread safety
        mutable std::mutex mutex;
        std::condition_variable condVar;
    };
}

#include "MessageQueue.tpp"

#endif