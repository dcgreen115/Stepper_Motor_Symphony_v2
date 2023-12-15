#include "MessageQueue.hpp"

template<typename T>
std::size_t sms::MessageQueue<T>::size() const noexcept {
    std::lock_guard<std::mutex> lock(mutex);
    return data.size();
}

template<typename T>
void sms::MessageQueue<T>::push(const T &value) {
    std::lock_guard<std::mutex> lock(mutex);
    data.push(value);
    condVar.notify_one();
}

template<typename T>
bool sms::MessageQueue<T>::tryPop(T &value) noexcept {
    std::lock_guard<std::mutex> lock(mutex);
    if (data.empty()) {
        return false;
    }

     value= data.front();
    data.pop();
    return true;
}

template<typename T>
void sms::MessageQueue<T>::waitPop(T &value) noexcept {
    std::unique_lock<std::mutex> lock(mutex);
    while (data.empty()) {
        condVar.wait(lock);
    }

    value = data.front();
    data.pop();
}
